// Copyright The odrive-can-driver Contributors
//! `embedded-can` 0.4 non-blocking endpoint adapter.
//!
//! [`CanTx`] and [`CanRx`] deliberately have separate ownership. The blanket
//! implementations let one `embedded_can::nb::Can` implement both, while a
//! shared-bus application can pass separate endpoint handles. This module
//! never drains RX or configures either endpoint.
//!
//! `embedded-can` Classic frames cannot represent CAN FD frames or bus-error
//! notifications. Applications which need those retain and distribute them
//! at their controller-specific boundary. A receive timestamp is the
//! application's observation timestamp in the driver's [`Instant`] clock
//! domain; it is not asserted to be a hardware edge timestamp.

use crate::protocol::compat::embedded_can::InvalidFrameLength;
use crate::{
    AttemptError, BeginSendError, Driver, IngestResult, Instant, SendPermit, TxAttempt,
    TxCompletion, TxOutcome,
};
use embedded_can::{Frame, nb};

/// A non-blocking TX endpoint owned by the application.
pub trait CanTx {
    /// Native Classic CAN frame type.
    type Frame: Frame;
    /// Native controller error type.
    type Error;

    /// Attempts to queue one frame.
    ///
    /// `WouldBlock` has the exact `embedded-can` 0.4 meaning: no TX buffer
    /// accepted this frame and no lower-priority pending frame was replaced.
    /// Returning `Ok(Some(frame))` means the new frame was accepted and the
    /// returned native frame was displaced.
    fn try_transmit(
        &mut self,
        frame: &Self::Frame,
    ) -> Result<Option<Self::Frame>, ::nb::Error<Self::Error>>;
}

/// A non-blocking RX endpoint owned by the application.
pub trait CanRx {
    /// Native Classic CAN frame type.
    type Frame: Frame;
    /// Native controller error type.
    type Error;

    /// Attempts to dequeue one frame without draining the endpoint.
    fn try_receive(&mut self) -> Result<Self::Frame, ::nb::Error<Self::Error>>;
}

impl<C> CanTx for C
where
    C: nb::Can,
{
    type Frame = C::Frame;
    type Error = C::Error;

    fn try_transmit(
        &mut self,
        frame: &Self::Frame,
    ) -> Result<Option<Self::Frame>, ::nb::Error<Self::Error>> {
        self.transmit(frame)
    }
}

impl<C> CanRx for C
where
    C: nb::Can,
{
    type Frame = C::Frame;
    type Error = C::Error;

    fn try_receive(&mut self) -> Result<Self::Frame, ::nb::Error<Self::Error>> {
        self.receive()
    }
}

/// A single native TX attempt whose driver borrow exists only during [`Self::poll`].
#[must_use]
pub struct NbTx<'s> {
    attempt: TxAttempt<'s>,
}

impl core::fmt::Debug for NbTx<'_> {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        formatter.debug_struct("NbTx").finish_non_exhaustive()
    }
}

impl<'s> NbTx<'s> {
    /// Consumes a prepared permit to begin one native non-blocking TX attempt.
    pub fn begin(
        driver: &mut Driver<'s>,
        permit: SendPermit<'s>,
        at: Instant,
    ) -> Result<Self, BeginSendError> {
        driver
            .begin_send(permit, at)
            .map(|attempt| Self { attempt })
    }

    /// Cancels before this adapter has called the native endpoint.
    ///
    /// Consuming `self` proves this adapter will not later poll this attempt.
    /// The application must only use it before [`Self::poll`]; it is not proof
    /// about a controller call made outside this object.
    pub fn cancel_unsubmitted(
        self,
        driver: &mut Driver<'s>,
        occurred_at: Instant,
        processed_at: Instant,
    ) -> Result<(), AttemptError> {
        driver.cancel_unsubmitted(self.attempt, occurred_at, processed_at)
    }

    /// Polls the native endpoint exactly once and consumes this attempt.
    ///
    /// The driver is borrowed only to authorize this one poll and to record its
    /// completed result. `now` is sampled immediately before native I/O, once
    /// immediately after it ends as the result event time, and again when the
    /// result is recorded. Thus the application may receive frames, advance a
    /// deadline, or cancel a retry permit between calls. If this function
    /// returns a controller error, the native call has ended but whether it
    /// queued the frame is not proved; the driver is changed to `Unknown`.
    pub fn poll<T>(
        self,
        driver: &mut Driver<'s>,
        tx: &mut T,
        mut now: impl FnMut() -> Instant,
    ) -> Result<NbTransmit<'s, T::Frame>, NbTransmitError<'s, T::Error>>
    where
        T: CanTx,
    {
        let frame = match driver.authorize_tx(&self.attempt, now()) {
            Ok(frame) => frame,
            Err(error) => {
                return Err(NbTransmitError::Authorize {
                    attempt: self,
                    error,
                });
            }
        };
        let Some(native) = frame.to_embedded_can::<T::Frame>() else {
            let occurred_at = now();
            let processed_at = now();
            return match finish_not_submitted(driver, self.attempt, occurred_at, processed_at) {
                Ok(_) => Err(NbTransmitError::FrameRejected),
                Err(error) => Err(NbTransmitError::Finish(error)),
            };
        };

        match tx.try_transmit(&native) {
            Ok(displaced) => {
                let occurred_at = now();
                let processed_at = now();
                match driver.finish_tx(
                    self.attempt,
                    TxOutcome::Submitted { occurred_at },
                    processed_at,
                ) {
                    Ok(TxCompletion::Submitted) => Ok(NbTransmit::Submitted { displaced }),
                    Ok(completion) => Ok(NbTransmit::Finished {
                        displaced,
                        completion,
                    }),
                    Err(error) => Ok(NbTransmit::Uncertain { displaced, error }),
                }
            }
            Err(::nb::Error::WouldBlock) => {
                let occurred_at = now();
                let processed_at = now();
                match driver.finish_tx(
                    self.attempt,
                    TxOutcome::WouldBlock { occurred_at },
                    processed_at,
                ) {
                    Ok(TxCompletion::Retry(retry)) => Ok(NbTransmit::WouldBlock { retry }),
                    Ok(completion) => Ok(NbTransmit::Finished {
                        displaced: None,
                        completion,
                    }),
                    Err(error) => Ok(NbTransmit::Uncertain {
                        displaced: None,
                        error,
                    }),
                }
            }
            Err(::nb::Error::Other(error)) => {
                let _ = driver.abandon_attempt(self.attempt, now());
                Err(NbTransmitError::Driver(error))
            }
        }
    }
}

fn finish_not_submitted<'s>(
    driver: &mut Driver<'s>,
    attempt: TxAttempt<'s>,
    occurred_at: Instant,
    processed_at: Instant,
) -> Result<TxCompletion<'s>, AttemptError> {
    driver.finish_tx(
        attempt,
        TxOutcome::NotSubmitted { occurred_at },
        processed_at,
    )
}

/// Completed non-blocking TX observation.
#[derive(Debug)]
pub enum NbTransmit<'s, F> {
    /// The controller accepted this frame. The displaced native frame stays
    /// with the application and must not be dropped by a shared-bus loop.
    Submitted {
        /// A lower-priority pending native frame displaced by this submission.
        displaced: Option<F>,
    },
    /// The completed synchronous native call proved this frame was not queued.
    ///
    /// The returned permit is the only valid way to start another attempt. The
    /// application may instead call [`Driver::cancel`] with `retry.id()`.
    WouldBlock {
        /// One-use permit for a later attempt of the same operation.
        retry: SendPermit<'s>,
    },
    /// A core time diagnostic after the native call. The operation report retains the actual
    /// submission or proved non-submission; consult it instead of inferring either from this
    /// variant. A displaced native frame is preserved even when recording submission failed.
    Uncertain {
        /// A displaced native frame, if the controller provided one.
        displaced: Option<F>,
        /// Core diagnostic for the late or inconsistent result processing.
        error: AttemptError,
    },
    /// Core reached a terminal completion while the native result was being
    /// processed. This preserves any displaced frame and never invents retry.
    Finished {
        /// A displaced native frame, if the controller provided one.
        displaced: Option<F>,
        /// The core's terminal completion.
        completion: TxCompletion<'s>,
    },
}

/// TX error which still preserves the real native outcome boundary.
#[derive(Debug)]
pub enum NbTransmitError<'s, E> {
    /// The driver rejected authorization before this native call began. The
    /// returned attempt can be cancelled with proved no native submission.
    Authorize {
        /// Unpolled attempt that remains the sole authority to close it.
        attempt: NbTx<'s>,
        /// Core authorization diagnostic.
        error: AttemptError,
    },
    /// The encoded Classic frame could not be represented by this native type;
    /// no endpoint call occurred and the operation is `Failed`.
    FrameRejected,
    /// Core could not record a proved pre-I/O rejection.
    Finish(AttemptError),
    /// A native controller error. It does not prove non-submission; the
    /// operation is retained as `Unknown`.
    Driver(E),
}

/// One non-blocking receive result, including its unmodified native frame.
#[derive(Debug)]
pub enum NbReceive<F> {
    /// No frame was available on this endpoint.
    Empty,
    /// One native Classic/RTR frame and this driver's protocol classification.
    Frame {
        /// Native frame for the application's shared-bus distribution.
        frame: F,
        /// Observation timestamp sampled after the endpoint returned this frame.
        received_at: Instant,
        /// This driver's classification only; it does not consume the frame.
        classification: IngestResult,
    },
}

/// Receive failure retaining raw controller errors and invalid native frames.
#[derive(Debug)]
pub enum NbReceiveError<F, E> {
    /// Native controller error.
    Driver(E),
    /// Invalid Classic length; the original frame was retained.
    InvalidFrame {
        /// Original native frame.
        frame: F,
        /// Observation timestamp sampled after the endpoint returned this frame.
        received_at: Instant,
        /// Conversion error.
        error: InvalidFrameLength,
    },
}

/// A complete non-blocking receive result for one endpoint's frame and error types.
pub type NbReceiveResult<R> = Result<
    NbReceive<<R as CanRx>::Frame>,
    NbReceiveError<<R as CanRx>::Frame, <R as CanRx>::Error>,
>;

/// Reads and classifies at most one native frame, sampling time after dequeue.
pub fn receive_nb<R>(
    driver: &mut Driver<'_>,
    rx: &mut R,
    observed_at: impl FnOnce() -> Instant,
) -> NbReceiveResult<R>
where
    R: CanRx,
{
    match rx.try_receive() {
        Ok(frame) => {
            let received_at = observed_at();
            ingest_classic_frame(driver, frame, received_at).map_err(|(frame, error)| {
                NbReceiveError::InvalidFrame {
                    frame,
                    received_at,
                    error,
                }
            })
        }
        Err(::nb::Error::WouldBlock) => Ok(NbReceive::Empty),
        Err(::nb::Error::Other(error)) => Err(NbReceiveError::Driver(error)),
    }
}

/// Classifies a Classic/RTR frame already dequeued by the application.
pub fn ingest_classic_frame<F>(
    driver: &mut Driver<'_>,
    frame: F,
    received_at: Instant,
) -> Result<NbReceive<F>, (F, InvalidFrameLength)>
where
    F: Frame,
{
    let view = match crate::protocol::FrameRef::from_classic_embedded_can(&frame) {
        Ok(view) => view,
        Err(error) => return Err((frame, error)),
    };
    let classification = driver.ingest(view, received_at);
    Ok(NbReceive::Frame {
        frame,
        received_at,
        classification,
    })
}
