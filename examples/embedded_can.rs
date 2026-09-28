// Copyright The odrive-can-driver Contributors
//! A single-loop `embedded-can` shared-bus pattern with split endpoints.
//!
//! The application owns both endpoints, distributes every returned RX/displaced TX frame, and
//! decides whether to retry the fresh permit returned by a real `WouldBlock`. After each turn,
//! inspect `pending_response` and explicitly accept, ignore or reject it. Keep calling `tick`
//! even without traffic. Neither RX nor TX errors discard the other endpoint's observations.

#![no_std]

use odrive_can_driver::{
    Driver, Instant, PrepareError,
    embedded_can::{
        CanRx, CanTx, NbReceive, NbReceiveError, NbTransmit, NbTransmitError, NbTx, receive_nb,
    },
};

/// All observations from one scheduling turn, including simultaneous errors.
#[derive(Debug)]
pub struct Turn<'s, TF, RF, TE, RE> {
    /// Result of advancing the processing clock.
    pub clock: Result<(), PrepareError>,
    /// One native RX result; always return it to the shared-bus owner.
    pub received: Result<NbReceive<RF>, NbReceiveError<RF, RE>>,
    /// One native TX result. A revoked attempt is returned in the authorization error.
    pub transmitted: Option<Result<NbTransmit<'s, TF>, NbTransmitError<'s, TE>>>,
}

/// The endpoint-specific observations of a scheduling turn.
pub type EndpointTurn<'s, T, R> =
    Turn<'s, <T as CanTx>::Frame, <R as CanRx>::Frame, <T as CanTx>::Error, <R as CanRx>::Error>;

/// Advances time and polls each endpoint once, without dropping RX on a TX failure.
///
/// Build `pending_tx` using `NbTx::begin(driver, permit, now)`. The next turn's attempt comes
/// only from an explicit `WouldBlock` retry permit, or an authorization error returning the
/// unpolled `NbTx`. Every TX call separately checks its processing clock and deadline.
pub fn poll_once<'s, T: CanTx, R: CanRx>(
    driver: &mut Driver<'s>,
    tx: &mut T,
    rx: &mut R,
    pending_tx: Option<NbTx<'s>>,
    mut now: impl FnMut() -> Instant,
) -> EndpointTurn<'s, T, R> {
    let clock = driver.tick(now());
    let received = receive_nb(driver, rx, &mut now);
    let transmitted = pending_tx.map(|attempt| attempt.poll(driver, tx, &mut now));
    Turn {
        clock,
        received,
        transmitted,
    }
}
