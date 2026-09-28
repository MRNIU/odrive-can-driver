// Copyright The odrive-can-driver Contributors
//! Embassy STM32 FDCAN split-endpoint adapter.
//!
//! [`EmbassyTx`] owns one [`crate::TxAttempt`], while each [`EmbassyTx::poll`] call only
//! temporarily borrows Embassy's [`CanTx`]. It authorizes the exact attempt immediately before
//! one native poll, so applications can keep RX, deadlines, and cancellation in their own loop.
//!
//! The cancellation proof is specific to Embassy revision
//! `7b08a9c7d9a9fe620f4be25c4e7b86dd29f09f54`: `TxMode::write_generic` returns
//! `Poll::Pending` only when `Registers::write` returned `WouldBlock`, before the new frame is
//! put in message RAM. This adapter drops that native future before returning `Pending`.

use core::{
    future::Future,
    task::{Context, Poll},
};

use embassy_stm32::can::{
    CanRx, CanTx,
    enums::BusError,
    frame::{FdEnvelope, FdFrame, Frame},
};

use crate::protocol::compat::embassy::FromEmbassyError;
use crate::{
    AttemptError, BeginSendError, Driver, IngestResult, Instant, SendPermit, TxAttempt,
    TxCompletion, TxOutcome,
};

/// A split FDCAN TX operation. It does not borrow [`Driver`] between [`Self::poll`] calls.
#[must_use]
pub struct EmbassyTx<'s> {
    attempt: Option<TxAttempt<'s>>,
}

impl<'s> EmbassyTx<'s> {
    /// Consumes a prepared permit and opens the first TX gate.
    pub fn begin(
        driver: &mut Driver<'s>,
        permit: SendPermit<'s>,
        processed_at: Instant,
    ) -> Result<Self, BeginSendError> {
        Ok(Self {
            attempt: Some(driver.begin_send(permit, processed_at)?),
        })
    }

    /// Polls one actual Embassy `CanTx::write` attempt.
    ///
    /// The adapter samples `now` immediately before native polling for the authorization gate and
    /// again only after the native future has returned `Ready`, so it never backfills a queue
    /// result with a speculative pre-poll time. All samples are monotonic microseconds in the
    /// driver's clock domain. A returned `Pending` owns no live Embassy future.
    pub fn poll<F>(
        &mut self,
        tx: &mut CanTx<'_>,
        driver: &mut Driver<'s>,
        context: &mut Context<'_>,
        mut now: F,
    ) -> Poll<Result<EmbassyTransmit<'s>, EmbassyTxError>>
    where
        F: FnMut() -> Instant,
    {
        let attempt = self
            .attempt
            .take()
            .expect("EmbassyTx polled after completion");
        let encoded = match driver.authorize_tx(&attempt, now()) {
            Ok(encoded) => encoded,
            Err(error) => {
                self.attempt = Some(attempt);
                return Poll::Ready(Err(EmbassyTxError::Attempt(error)));
            }
        };
        let frame = Frame::from(&encoded);
        let write = tx.write(&frame);
        let mut write = core::pin::pin!(write);
        match write.as_mut().poll(context) {
            Poll::Pending => {
                self.attempt = Some(attempt);
                Poll::Pending
            }
            Poll::Ready(displaced) => {
                let occurred_at = now();
                let processed_at = now();
                let completion =
                    driver.finish_tx(attempt, TxOutcome::Submitted { occurred_at }, processed_at);
                Poll::Ready(Ok(EmbassyTransmit::Submitted {
                    displaced,
                    completion,
                }))
            }
        }
    }

    /// Records proven cancellation after this wrapper's last native future has ended.
    ///
    /// Every native future is created and dropped inside [`Self::poll`]. Dropping this wrapper
    /// alone is deliberately conservative and leaves the core operation uncertain.
    pub fn cancel_unsubmitted(
        mut self,
        driver: &mut Driver<'s>,
        occurred_at: Instant,
        processed_at: Instant,
    ) -> Result<(), EmbassyTxError> {
        let attempt = self
            .attempt
            .take()
            .expect("EmbassyTx cancelled after completion");
        driver
            .cancel_unsubmitted(attempt, occurred_at, processed_at)
            .map_err(EmbassyTxError::Attempt)
    }

    /// Retains `Unknown` when the caller cannot provide cancellation evidence.
    pub fn abandon(
        mut self,
        driver: &mut Driver<'s>,
        processed_at: Instant,
    ) -> Result<(), EmbassyTxError> {
        let attempt = self
            .attempt
            .take()
            .expect("EmbassyTx abandoned after completion");
        driver
            .abandon_attempt(attempt, processed_at)
            .map_err(EmbassyTxError::Attempt)
    }
}

/// A terminal local FDCAN TX result and the core's next action.
#[derive(Debug)]
pub enum EmbassyTransmit<'s> {
    /// FDCAN accepted the new frame. The displaced frame remains caller-owned shared-bus data.
    Submitted {
        /// Original displaced Classic frame, if priority replacement occurred.
        displaced: Option<Frame>,
        /// Core completion, or a clock/deadline diagnostic after FDCAN has already accepted the
        /// frame. The native `displaced` frame remains available in either case.
        completion: Result<TxCompletion<'s>, AttemptError>,
    },
}

/// Failure before or while recording a TX result.
#[derive(Debug)]
pub enum EmbassyTxError {
    /// The core revoked the attempt or rejected time/backfill data.
    Attempt(AttemptError),
}

/// A raw FDCAN receive outcome. The full envelope is always retained.
#[derive(Debug)]
pub enum EmbassyReceive {
    /// A raw frame with its protocol classification.
    Frame {
        /// Original frame plus Embassy controller timestamp.
        envelope: FdEnvelope,
        /// Protocol classification; unrelated and decode-failed frames remain in `envelope`.
        classification: IngestResult,
    },
    /// Protocol view conversion failed but the complete native frame remains available.
    Invalid {
        /// Original frame plus Embassy controller timestamp.
        envelope: FdEnvelope,
        /// Conversion error.
        error: FromEmbassyError,
    },
}

/// Awaits one raw envelope from an application-owned split FDCAN RX endpoint.
///
/// This function deliberately does not borrow [`Driver`]. After awaiting it, map the intact
/// [`FdEnvelope::ts`] into the driver's monotonic microsecond domain and call
/// [`ingest_fd_envelope`] with only a short driver borrow. Raw [`BusError`] is returned directly.
pub async fn receive(rx: &mut CanRx<'_>) -> Result<FdEnvelope, BusError> {
    rx.read_fd().await
}

/// Classifies an application-owned raw envelope without dropping its timestamp or frame.
pub fn ingest_fd_envelope(
    driver: &mut Driver<'_>,
    envelope: FdEnvelope,
    received_at: Instant,
) -> EmbassyReceive {
    match crate::protocol::FrameRef::try_from(&envelope.frame) {
        Ok(view) => EmbassyReceive::Frame {
            classification: driver.ingest(view, received_at),
            envelope,
        },
        Err(error) => EmbassyReceive::Invalid { envelope, error },
    }
}

/// Split Embassy FDCAN transmitter type for application signatures.
pub type EmbassyCanTx<'d> = CanTx<'d>;
/// Split Embassy FDCAN receiver type for application signatures.
pub type EmbassyCanRx<'d> = CanRx<'d>;

/// Returns the raw FDCAN frame without changing its classification.
pub fn raw_frame(receive: &EmbassyReceive) -> &FdFrame {
    match receive {
        EmbassyReceive::Frame { envelope, .. } | EmbassyReceive::Invalid { envelope, .. } => {
            &envelope.frame
        }
    }
}
