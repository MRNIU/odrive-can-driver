// Copyright The odrive-can-driver Contributors
//! 验证 Embassy FDCAN 接收路径把无关原生帧归还给共享总线调用方。

#![cfg(feature = "embassy-stm32")]

use embassy_stm32::can::frame::{FdFrame, Header};
use odrive_can_driver::{
    Driver, IngestResult,
    embassy::{EmbassyReceive, ingest_fd_frame},
    protocol::NodeId,
};

fn node() -> NodeId {
    NodeId::new(7).unwrap()
}

#[test]
fn unrelated_fd_container_is_returned_without_losing_its_classic_header() {
    let mut driver = Driver::new(node());
    let fd_frame = FdFrame::new_standard((8 << 5) | 3, &[1, 2]).unwrap();
    let frame = FdFrame::new(Header::new(*fd_frame.header().id(), 2, false), &[1, 2]).unwrap();

    let outcome = ingest_fd_frame(&mut driver, frame, 3);

    let EmbassyReceive::Frame {
        frame,
        classification,
    } = outcome
    else {
        panic!("a received frame must retain native ownership");
    };
    assert_eq!(classification, IngestResult::Unrelated);
    assert!(!frame.header().fdcan());
    assert_eq!(frame.header().len(), 2);
    assert_eq!(frame.data(), &[1, 2]);
}

#[test]
fn unrelated_short_fd_container_is_returned_as_fd() {
    let mut driver = Driver::new(node());
    let frame = FdFrame::new_standard((8 << 5) | 3, &[1, 2]).unwrap();

    let outcome = ingest_fd_frame(&mut driver, frame, 3);

    let EmbassyReceive::Frame {
        frame,
        classification,
    } = outcome
    else {
        panic!("a received frame must retain native ownership");
    };
    assert_eq!(classification, IngestResult::Unrelated);
    assert!(frame.header().fdcan());
    assert_eq!(frame.header().len(), 2);
    assert_eq!(frame.data(), &[1, 2]);
}

#[test]
fn extended_container_is_returned_when_protocol_rejects_it_before_node_filtering() {
    let mut driver = Driver::new(node());
    let frame = FdFrame::new_extended(0x12_345, &[1]).unwrap();

    let outcome = ingest_fd_frame(&mut driver, frame, 3);

    let EmbassyReceive::Frame {
        frame,
        classification,
    } = outcome
    else {
        panic!("an extended FDCAN container must remain available to the bus owner");
    };
    assert_eq!(
        classification,
        IngestResult::DecodeError(odrive_can_driver::protocol::DecodeError::UnsupportedExtendedId)
    );
    assert_eq!(frame.header().len(), 1);
    assert_eq!(frame.data(), &[1]);
}
