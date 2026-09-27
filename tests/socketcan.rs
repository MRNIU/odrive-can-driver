// Copyright The odrive-can-driver Contributors
//! 验证 Linux SocketCAN 后端保留原生帧并将已知 I/O 结果交给共享 core。

#![cfg(all(feature = "socketcan", target_os = "linux"))]

use std::io::ErrorKind;

use odrive_can_driver::{
    Driver, OperationState,
    protocol::{self, Command, NodeId},
    socketcan::{SocketCan, SocketCanReceive, SocketCanSendError, receive_frame},
};
use socketcan::{
    CanAnyFrame, CanDataFrame, CanErrorFrame, CanFdFrame, CanFdSocket, CanRemoteFrame,
    EmbeddedFrame, ExtendedId, Socket, SocketOptions, StandardId, id::ERR_MASK_ALL,
};

fn node() -> NodeId {
    NodeId::new(7).unwrap()
}

fn unrelated_id() -> StandardId {
    StandardId::new((8 << 5) | 3).unwrap()
}

#[test]
fn receive_frame_keeps_every_native_frame_with_its_classification() {
    let mut driver = Driver::new(node());
    let normal = CanAnyFrame::Normal(
        CanDataFrame::new(StandardId::new((7 << 5) | 3).unwrap(), &[0; 8]).unwrap(),
    );
    let remote = CanAnyFrame::Remote(CanRemoteFrame::new_remote(unrelated_id(), 4).unwrap());
    let fd =
        CanAnyFrame::Fd(CanFdFrame::new(StandardId::new((7 << 5) | 1).unwrap(), &[3; 12]).unwrap());
    let extended =
        CanAnyFrame::Normal(CanDataFrame::new(ExtendedId::new(0x12_345).unwrap(), &[9]).unwrap());
    let error = CanAnyFrame::Error(CanErrorFrame::new_error(0x21, &[4; 8]).unwrap());

    assert_eq!(
        receive_frame(&mut driver, normal, 1),
        SocketCanReceive {
            frame: normal,
            classification: Ok(odrive_can_driver::IngestResult::Message(
                protocol::Message::Response(protocol::Response::MotorError(0),)
            )),
        }
    );
    assert_eq!(
        receive_frame(&mut driver, remote, 2),
        SocketCanReceive {
            frame: remote,
            classification: Ok(odrive_can_driver::IngestResult::Unrelated),
        }
    );
    assert_eq!(
        receive_frame(&mut driver, fd, 3),
        SocketCanReceive {
            frame: fd,
            classification: Ok(odrive_can_driver::IngestResult::DecodeError(
                protocol::DecodeError::UnsupportedCanFd,
            )),
        }
    );
    assert_eq!(
        receive_frame(&mut driver, extended, 4),
        SocketCanReceive {
            frame: extended,
            classification: Ok(odrive_can_driver::IngestResult::DecodeError(
                protocol::DecodeError::UnsupportedExtendedId,
            )),
        }
    );
    assert_eq!(
        receive_frame(&mut driver, error, 5),
        SocketCanReceive {
            frame: error,
            classification: Err(protocol::compat::socketcan::FromSocketcanError::ErrorFrame),
        }
    );
}

#[test]
fn caller_can_configure_its_socket_error_filter() {
    let backend = SocketCan::from_socket(CanFdSocket::open_iface(0).unwrap()).unwrap();

    assert!(backend.socket().nonblocking().unwrap());
    backend.socket().set_error_filter(ERR_MASK_ALL).unwrap();
    assert_eq!(backend.socket().error_filter().unwrap(), ERR_MASK_ALL);
}

#[test]
fn other_socket_io_error_leaves_the_send_attempt_unknown() {
    let backend = SocketCan::from_socket(CanFdSocket::open_iface(0).unwrap()).unwrap();
    let mut driver = Driver::new(node());
    let id = driver.prepare_command(Command::ClearErrors, 0, 20).unwrap();
    let mut samples = [1, 4].into_iter();

    assert!(matches!(
        backend.send(&mut driver, id, || samples.next().unwrap()),
        Err(SocketCanSendError::Io(_))
    ));
    let report = driver.report(id).unwrap();
    assert_eq!(report.state, OperationState::Unknown);
    assert_eq!(report.dispatching_at_ms, Some(1));
    assert_eq!(report.submitted_at_ms, None);
}

#[test]
#[ignore = "requires the explicitly created odrive-vcan interface"]
fn vcan_socket_is_nonblocking_and_reports_native_rx_and_local_submission() {
    let rx = CanFdSocket::open("odrive-vcan").unwrap();
    rx.set_nonblocking(true).unwrap();
    let backend = SocketCan::open("odrive-vcan").unwrap();
    let mut driver = Driver::new(node());

    assert!(matches!(
        backend.receive(&mut driver, || 1),
        Err(error) if error.kind() == ErrorKind::WouldBlock
    ));

    let unrelated = CanDataFrame::new(unrelated_id(), &[1, 2]).unwrap();
    rx.write_frame(&unrelated).unwrap();
    let received = backend.receive(&mut driver, || 2).unwrap();
    assert_eq!(received.frame, CanAnyFrame::Normal(unrelated));
    assert_eq!(
        received.classification,
        Ok(odrive_can_driver::IngestResult::Unrelated)
    );

    let id = driver.prepare_command(Command::ClearErrors, 3, 20).unwrap();
    let mut samples = [4, 6].into_iter();
    backend
        .send(&mut driver, id, || samples.next().unwrap())
        .unwrap();
    let report = driver.report(id).unwrap();
    assert_eq!(report.state, OperationState::Submitted);
    assert_eq!(report.dispatching_at_ms, Some(4));
    assert_eq!(report.submitted_at_ms, Some(6));
}
