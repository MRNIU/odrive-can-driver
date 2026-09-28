// Copyright The odrive-can-driver Contributors
//! `embedded-can` endpoint ownership and submission-evidence tests.

#![cfg(feature = "embedded-can")]

use core::convert::Infallible;

use embedded_can::{Frame, Id, StandardId};
use odrive_can_driver::{
    Driver, Instant, OperationState, Session,
    embedded_can::{CanRx, CanTx, NbReceive, NbTransmit, NbTx, receive_nb},
    protocol::{Command, NodeId, Query},
};

enum SendResult {
    Sent,
    Replaced(bxcan::Frame),
    WouldBlock,
}

struct ControlledTx {
    result: Option<SendResult>,
}

impl ControlledTx {
    fn new(result: SendResult) -> Self {
        Self {
            result: Some(result),
        }
    }
}

impl CanTx for ControlledTx {
    type Frame = bxcan::Frame;
    type Error = Infallible;

    fn try_transmit(
        &mut self,
        _: &Self::Frame,
    ) -> Result<Option<Self::Frame>, ::nb::Error<Self::Error>> {
        match self.result.take().expect("test transmits once") {
            SendResult::Sent => Ok(None),
            SendResult::Replaced(frame) => Ok(Some(frame)),
            SendResult::WouldBlock => Err(::nb::Error::WouldBlock),
        }
    }
}

struct ControlledRx {
    frame: Option<bxcan::Frame>,
}

impl ControlledRx {
    fn frame(frame: bxcan::Frame) -> Self {
        Self { frame: Some(frame) }
    }
}

impl CanRx for ControlledRx {
    type Frame = bxcan::Frame;
    type Error = Infallible;

    fn try_receive(&mut self) -> Result<Self::Frame, ::nb::Error<Self::Error>> {
        self.frame.take().ok_or(::nb::Error::WouldBlock)
    }
}

fn node() -> NodeId {
    NodeId::new(7).unwrap()
}

fn at(us: u64) -> Instant {
    Instant::from_micros(us)
}

#[test]
fn split_endpoints_return_displaced_frame_after_submission() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let permit = driver
        .prepare_command(Command::ClearErrors, at(0), at(20))
        .unwrap();
    let id = permit.id();
    let displaced = bxcan::Frame::new(StandardId::new(0x321).unwrap(), &[7]).unwrap();
    let mut tx = ControlledTx::new(SendResult::Replaced(displaced));

    let mut times = [at(2), at(3), at(4)].into_iter();
    let outcome = NbTx::begin(&mut driver, permit, at(1))
        .unwrap()
        .poll(&mut driver, &mut tx, || times.next().unwrap())
        .unwrap();

    let NbTransmit::Submitted {
        displaced: Some(displaced),
    } = outcome
    else {
        panic!("accepted TX must return its displaced native frame");
    };
    assert_eq!(
        Frame::id(&displaced),
        StandardId::new(0x321).unwrap().into()
    );
    let report = driver.report(id).unwrap();
    assert_eq!(report.state, OperationState::Submitted);
    assert_eq!(report.submitted_at, Some(at(3)));
    assert_eq!(report.processed_at, at(4));
}

#[test]
fn would_block_returns_only_retry_permit_then_allows_proved_cancel() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let permit = driver
        .prepare_query(Query::MotorError, at(0), at(20))
        .unwrap();
    let id = permit.id();
    let mut tx = ControlledTx::new(SendResult::WouldBlock);

    let outcome = NbTx::begin(&mut driver, permit, at(1))
        .unwrap()
        .poll(&mut driver, &mut tx, || at(2))
        .unwrap();
    let NbTransmit::WouldBlock { retry } = outcome else {
        panic!("only a completed WouldBlock call proves no queue submission");
    };
    assert_eq!(retry.id(), id);
    assert_eq!(driver.report(id).unwrap().state, OperationState::Prepared);
    driver.cancel(id, at(3)).unwrap();
    assert_eq!(
        driver.take_report(id).unwrap().state,
        OperationState::Cancelled
    );
}

#[test]
fn would_block_processed_after_deadline_never_panics_or_retries() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let permit = driver
        .prepare_query(Query::MotorError, at(0), at(20))
        .unwrap();
    let id = permit.id();
    let mut tx = ControlledTx::new(SendResult::WouldBlock);
    let mut times = [at(2), at(20), at(21)].into_iter();

    let outcome = NbTx::begin(&mut driver, permit, at(1))
        .unwrap()
        .poll(&mut driver, &mut tx, || times.next().unwrap())
        .unwrap();

    assert!(matches!(outcome, NbTransmit::Finished { .. }));
    assert_eq!(
        driver.take_report(id).unwrap().state,
        OperationState::TimedOut
    );
}

#[test]
fn unpolled_attempt_can_be_proved_cancelled_without_native_io() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let permit = driver
        .prepare_query(Query::MotorError, at(0), at(20))
        .unwrap();
    let id = permit.id();

    NbTx::begin(&mut driver, permit, at(1))
        .unwrap()
        .cancel_unsubmitted(&mut driver, at(2), at(3))
        .unwrap();

    assert_eq!(
        driver.take_report(id).unwrap().state,
        OperationState::Cancelled
    );
}

#[test]
fn receive_endpoint_keeps_unrelated_native_frame_and_microsecond_timestamp() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let frame = bxcan::Frame::new(StandardId::new((8 << 5) | 3).unwrap(), &[1, 2]).unwrap();
    let mut rx = ControlledRx::frame(frame);

    let outcome = receive_nb(&mut driver, &mut rx, || at(3)).unwrap();
    let NbReceive::Frame {
        frame,
        received_at,
        classification,
    } = outcome
    else {
        panic!("the endpoint supplied one frame");
    };
    assert_eq!(classification, odrive_can_driver::IngestResult::Unrelated);
    assert_eq!(
        Frame::id(&frame),
        StandardId::new((8 << 5) | 3).unwrap().into()
    );
    assert_eq!(Frame::data(&frame), &[1, 2]);
    assert_eq!(received_at, at(3));
}

#[test]
fn receive_endpoint_retains_extended_and_rtr_native_frames() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let extended = bxcan::ExtendedId::new(0x12_345).unwrap();
    let frame = bxcan::Frame::new_remote(extended, 8);
    let mut rx = ControlledRx::frame(frame);

    let outcome = receive_nb(&mut driver, &mut rx, || at(3)).unwrap();
    let NbReceive::Frame {
        frame,
        classification,
        ..
    } = outcome
    else {
        panic!("the endpoint supplied one RTR frame");
    };
    assert!(matches!(
        classification,
        odrive_can_driver::IngestResult::DecodeError(_)
    ));
    assert!(Frame::is_remote_frame(&frame));
    let Id::Extended(id) = Frame::id(&frame) else {
        panic!("the original extended ID must be retained");
    };
    assert_eq!(id.as_raw(), 0x12_345);
}

#[test]
fn separate_rx_can_run_before_tx_submission() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let permit = driver
        .prepare_query(Query::MotorError, at(0), at(20))
        .unwrap();
    let id = permit.id();
    let mut tx = ControlledTx::new(SendResult::Sent);
    let request = bxcan::Frame::new(StandardId::new((7 << 5) | 3).unwrap(), &[0; 8]).unwrap();
    let mut rx = ControlledRx::frame(request);

    let received = receive_nb(&mut driver, &mut rx, || at(2)).unwrap();
    assert!(matches!(received, NbReceive::Frame { .. }));
    let outcome = NbTx::begin(&mut driver, permit, at(3))
        .unwrap()
        .poll(&mut driver, &mut tx, || at(4))
        .unwrap();
    assert!(matches!(outcome, NbTransmit::Submitted { .. }));
    assert_eq!(driver.report(id).unwrap().state, OperationState::Submitted);
}
