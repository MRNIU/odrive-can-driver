// Copyright The odrive-can-driver Contributors
//! 验证 `embedded-can` 后端保留置换与无关原生帧，并把发送状态交给共享 core。

#![cfg(feature = "embedded-can")]

use core::convert::Infallible;

use embedded_can::{ExtendedId, Frame, StandardId, blocking, nb::Can};
use odrive_can_driver::{
    Driver, IngestResult, OperationState,
    embedded_can::{
        BlockingTransmitError, NbReceive, NbTransmit, receive_nb, transmit_blocking, transmit_nb,
    },
    protocol::{Command, NodeId, Query},
};

enum SendResult {
    Sent,
    Replaced(bxcan::Frame),
    WouldBlock,
}

struct ControlledCan {
    send: Option<SendResult>,
    receive: Option<bxcan::Frame>,
}

impl ControlledCan {
    fn sender(send: SendResult) -> Self {
        Self {
            send: Some(send),
            receive: None,
        }
    }

    fn receiver(receive: bxcan::Frame) -> Self {
        Self {
            send: None,
            receive: Some(receive),
        }
    }
}

impl Can for ControlledCan {
    type Frame = bxcan::Frame;
    type Error = Infallible;

    fn transmit(&mut self, _: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        match self.send.take().expect("test sends once") {
            SendResult::Sent => Ok(None),
            SendResult::Replaced(frame) => Ok(Some(frame)),
            SendResult::WouldBlock => Err(nb::Error::WouldBlock),
        }
    }

    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        self.receive.take().ok_or(nb::Error::WouldBlock)
    }
}

struct ControlledBlockingCan;

impl blocking::Can for ControlledBlockingCan {
    type Frame = bxcan::Frame;
    type Error = Infallible;

    fn transmit(&mut self, _: &Self::Frame) -> Result<(), Self::Error> {
        Ok(())
    }

    fn receive(&mut self) -> Result<Self::Frame, Self::Error> {
        unreachable!("this controlled transport only sends")
    }
}

fn node() -> NodeId {
    NodeId::new(7).unwrap()
}

#[test]
fn replaced_frame_is_returned_after_our_operation_is_submitted() {
    let mut driver = Driver::new(node());
    let id = driver.prepare_command(Command::ClearErrors, 0, 20).unwrap();
    let replaced = bxcan::Frame::new(StandardId::new(0x321).unwrap(), &[7]).unwrap();
    let mut can = ControlledCan::sender(SendResult::Replaced(replaced));

    let outcome = transmit_nb(&mut driver, id, &mut can, || 1).unwrap();

    let NbTransmit::Submitted {
        displaced: Some(displaced),
    } = outcome
    else {
        panic!("the replaced native frame must be returned");
    };
    assert_eq!(
        Frame::id(&displaced),
        StandardId::new(0x321).unwrap().into()
    );
    assert_eq!(driver.report(id).unwrap().state, OperationState::Submitted);
}

#[test]
fn explicit_backpressure_keeps_the_operation_prepared_for_retry() {
    let mut driver = Driver::new(node());
    let id = driver.prepare_query(Query::MotorError, 0, 20).unwrap();
    let mut can = ControlledCan::sender(SendResult::WouldBlock);

    assert_eq!(
        transmit_nb(&mut driver, id, &mut can, || 1).unwrap(),
        NbTransmit::WouldBlock
    );

    assert_eq!(driver.report(id).unwrap().state, OperationState::Prepared);
}

#[test]
fn unrelated_received_frame_is_returned_with_its_native_ownership() {
    let mut driver = Driver::new(node());
    let frame = bxcan::Frame::new(StandardId::new((8 << 5) | 3).unwrap(), &[1, 2]).unwrap();
    let mut can = ControlledCan::receiver(frame);

    let outcome = receive_nb(&mut driver, &mut can, || 3).unwrap();

    let NbReceive::Frame {
        frame,
        classification,
    } = outcome
    else {
        panic!("a received frame must retain native ownership");
    };
    assert_eq!(classification, IngestResult::Unrelated);
    assert_eq!(
        Frame::id(&frame),
        StandardId::new((8 << 5) | 3).unwrap().into()
    );
    assert_eq!(Frame::data(&frame), &[1, 2]);
}

#[test]
fn extended_frame_is_returned_even_when_protocol_rejects_it_before_node_filtering() {
    let mut driver = Driver::new(node());
    let frame = bxcan::Frame::new(ExtendedId::new(0x12_345).unwrap(), &[1]).unwrap();
    let mut can = ControlledCan::receiver(frame);

    let outcome = receive_nb(&mut driver, &mut can, || 3).unwrap();

    let NbReceive::Frame {
        frame,
        classification,
    } = outcome
    else {
        panic!("an extended native frame must be returned");
    };
    assert_eq!(
        classification,
        IngestResult::DecodeError(odrive_can_driver::protocol::DecodeError::UnsupportedExtendedId)
    );
    assert_eq!(Frame::id(&frame), ExtendedId::new(0x12_345).unwrap().into());
}

#[test]
fn timestamp_after_native_submission_marks_a_late_operation_unknown() {
    let mut driver = Driver::new(node());
    let id = driver.prepare_command(Command::ClearErrors, 0, 10).unwrap();
    let mut can = ControlledCan::sender(SendResult::Sent);
    let mut timestamps = [1, 10].into_iter();

    let outcome = transmit_nb(&mut driver, id, &mut can, || timestamps.next().unwrap()).unwrap();

    assert!(matches!(outcome, NbTransmit::Uncertain { .. }));
    assert_eq!(driver.report(id).unwrap().state, OperationState::Unknown);
}

#[test]
fn displaced_frame_survives_a_late_submission_clock_result() {
    let mut driver = Driver::new(node());
    let id = driver.prepare_command(Command::ClearErrors, 0, 10).unwrap();
    let replaced = bxcan::Frame::new(StandardId::new(0x321).unwrap(), &[7]).unwrap();
    let mut can = ControlledCan::sender(SendResult::Replaced(replaced));
    let mut timestamps = [1, 10].into_iter();

    let outcome = transmit_nb(&mut driver, id, &mut can, || timestamps.next().unwrap()).unwrap();

    let NbTransmit::Uncertain {
        displaced: Some(displaced),
        ..
    } = outcome
    else {
        panic!("a displaced frame must survive an uncertain late submission");
    };
    assert_eq!(
        Frame::id(&displaced),
        StandardId::new(0x321).unwrap().into()
    );
    assert_eq!(driver.report(id).unwrap().state, OperationState::Unknown);
}

#[test]
fn blocking_submission_uses_the_clock_sample_after_native_transmit_returns() {
    let mut driver = Driver::new(node());
    let id = driver.prepare_command(Command::ClearErrors, 0, 10).unwrap();
    let mut can = ControlledBlockingCan;
    let mut timestamps = [1, 10].into_iter();

    assert!(matches!(
        transmit_blocking(&mut driver, id, &mut can, || timestamps.next().unwrap()),
        Err(BlockingTransmitError::Attempt(_))
    ));
    assert_eq!(driver.report(id).unwrap().state, OperationState::Unknown);
}
