// Copyright The odrive-can-driver Contributors
//! One shared-bus scheduling turn for split Embassy FDCAN endpoints.
//!
//! Split a configured `Can` with `can.split()`, prepare a permit, then construct
//! [`EmbassyTx`]. Call [`poll_once`] from the application's task, passing `None` after TX
//! completes while continuing RX, query admission and deadline handling. Each turn advances the
//! deadline, polls RX and TX once, and returns both observations together. Pending native futures
//! are dropped at the end of their one poll, so neither endpoint waits while borrowing `Driver`.

#![no_std]

use core::{
    future::Future,
    task::{Context, Poll},
};

use embassy_stm32::can::{CanRx, CanTx, enums::BusError, frame::FdEnvelope};
use odrive_can_driver::{
    Driver, Instant, PrepareError,
    embassy::{EmbassyReceive, EmbassyTransmit, EmbassyTx, EmbassyTxError, ingest_fd_envelope},
};

/// Results from one application-controlled bus turn.
#[derive(Debug)]
pub struct ServicePoll<'s> {
    /// One raw RX outcome, if FDCAN was ready. A `BusError` has no frame; otherwise the envelope
    /// and controller timestamp are retained in [`EmbassyReceive`].
    pub received: Option<Result<EmbassyReceive, BusError>>,
    /// One TX outcome or its registered pending state. A completed result always preserves a
    /// displaced native frame, including a core backfill diagnostic.
    pub transmitted: Option<Poll<Result<EmbassyTransmit<'s>, EmbassyTxError>>>,
}

/// Runs one nonblocking pass without holding a `Driver` borrow across endpoint waiting.
///
/// `received_at` maps the intact raw [`FdEnvelope`] into the driver's monotonic microsecond
/// domain. `now` is sampled by the core tick and by [`EmbassyTx::poll`] immediately around its
/// physical I/O boundary. Because the function does not return early for RX, a high RX rate does
/// not starve TX authorization; both native results remain visible to the application.
pub fn poll_once<'s, R, N>(
    tx: &mut CanTx<'_>,
    rx: &mut CanRx<'_>,
    tx_operation: Option<&mut EmbassyTx<'s>>,
    driver: &mut Driver<'s>,
    context: &mut Context<'_>,
    received_at: R,
    mut now: N,
) -> Result<ServicePoll<'s>, PrepareError>
where
    R: FnOnce(&FdEnvelope) -> Instant,
    N: FnMut() -> Instant,
{
    driver.tick(now())?;

    // The raw receive future has no Driver borrow. Poll it once so its waker can schedule the
    // next turn, then classify a ready envelope with a short, separate Driver borrow.
    let read = rx.read_fd();
    let mut read = core::pin::pin!(read);
    let received = match read.as_mut().poll(context) {
        Poll::Ready(Ok(envelope)) => {
            let event_at = received_at(&envelope);
            Some(Ok(ingest_fd_envelope(driver, envelope, event_at)))
        }
        Poll::Ready(Err(error)) => Some(Err(error)),
        Poll::Pending => None,
    };

    let transmitted = tx_operation.map(|operation| operation.poll(tx, driver, context, &mut now));
    Ok(ServicePoll {
        received,
        transmitted,
    })
}
