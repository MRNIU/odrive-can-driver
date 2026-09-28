// Copyright The odrive-can-driver Contributors
//! SocketCAN keeps native frames and only claims a proved local result.

#![cfg(all(feature = "socketcan", target_os = "linux"))]

use std::{io::ErrorKind, time::SystemTime};

use odrive_can_driver::{
    Driver, Instant, OperationState, Session,
    protocol::{self, Command, NodeId},
    socketcan::{CanRx, CanTx, SocketCanTxError, receive_frame},
};
use socketcan::{
    CanAnyFrame, CanDataFrame, CanErrorFrame, CanFdFrame, CanFdSocket, CanRemoteFrame,
    CanTimestamps, EmbeddedFrame, ExtendedId, Socket, SocketOptions, StandardId, id::ERR_MASK_ALL,
};

fn node() -> NodeId {
    NodeId::new(7).unwrap()
}

fn instant(micros: u64) -> Instant {
    Instant::from_micros(micros)
}

fn unrelated_id() -> StandardId {
    StandardId::new((8 << 5) | 3).unwrap()
}

fn timestamps() -> CanTimestamps {
    CanTimestamps {
        socket: Some(SystemTime::UNIX_EPOCH),
        sw: None,
        hw: Some(std::time::Duration::from_nanos(42)),
    }
}

#[test]
fn receive_frame_keeps_every_native_frame_timestamp_and_classification() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let normal = CanAnyFrame::Normal(
        CanDataFrame::new(StandardId::new((7 << 5) | 3).unwrap(), &[0; 8]).unwrap(),
    );
    let remote = CanAnyFrame::Remote(CanRemoteFrame::new_remote(unrelated_id(), 4).unwrap());
    let fd =
        CanAnyFrame::Fd(CanFdFrame::new(StandardId::new((7 << 5) | 1).unwrap(), &[3; 12]).unwrap());
    let extended =
        CanAnyFrame::Normal(CanDataFrame::new(ExtendedId::new(0x12_345).unwrap(), &[9]).unwrap());
    let error = CanAnyFrame::Error(CanErrorFrame::new_error(0x21, &[4; 8]).unwrap());

    let stamp = timestamps();
    let received = receive_frame(&mut driver, normal, stamp, instant(1));
    assert_eq!(received.frame, normal);
    assert_eq!(received.timestamps.socket, stamp.socket);
    assert_eq!(received.timestamps.sw, stamp.sw);
    assert_eq!(received.timestamps.hw, stamp.hw);
    assert_eq!(received.received_at, instant(1));
    assert_eq!(
        received.classification,
        Ok(odrive_can_driver::IngestResult::Message(
            protocol::Message::Response(protocol::Response::MotorError(0)),
        ))
    );

    let received = receive_frame(&mut driver, remote, stamp, instant(2));
    assert_eq!(received.frame, remote);
    assert_eq!(received.timestamps.socket, stamp.socket);
    assert_eq!(received.timestamps.sw, stamp.sw);
    assert_eq!(received.timestamps.hw, stamp.hw);
    assert_eq!(received.received_at, instant(2));
    assert_eq!(
        received.classification,
        Ok(odrive_can_driver::IngestResult::Unrelated)
    );

    let received = receive_frame(&mut driver, fd, stamp, instant(3));
    assert_eq!(received.frame, fd);
    assert_eq!(received.timestamps.socket, stamp.socket);
    assert_eq!(received.timestamps.sw, stamp.sw);
    assert_eq!(received.timestamps.hw, stamp.hw);
    assert_eq!(received.received_at, instant(3));
    assert_eq!(
        received.classification,
        Ok(odrive_can_driver::IngestResult::DecodeError(
            protocol::DecodeError::UnsupportedCanFd,
        ))
    );

    let received = receive_frame(&mut driver, extended, stamp, instant(4));
    assert_eq!(received.frame, extended);
    assert_eq!(received.timestamps.socket, stamp.socket);
    assert_eq!(received.timestamps.sw, stamp.sw);
    assert_eq!(received.timestamps.hw, stamp.hw);
    assert_eq!(received.received_at, instant(4));
    assert_eq!(
        received.classification,
        Ok(odrive_can_driver::IngestResult::DecodeError(
            protocol::DecodeError::UnsupportedExtendedId,
        ))
    );

    let received = receive_frame(&mut driver, error, stamp, instant(5));
    assert_eq!(received.frame, error);
    assert_eq!(received.timestamps.socket, stamp.socket);
    assert_eq!(received.timestamps.sw, stamp.sw);
    assert_eq!(received.timestamps.hw, stamp.hw);
    assert_eq!(received.received_at, instant(5));
    assert_eq!(
        received.classification,
        Err(protocol::compat::socketcan::FromSocketcanError::ErrorFrame)
    );
}

#[test]
fn endpoints_are_separate_nonblocking_descriptors_and_rx_options_remain_app_owned() {
    let tx = CanTx::from_socket(CanFdSocket::open_iface(0).unwrap()).unwrap();
    let rx = CanRx::from_socket(CanFdSocket::open_iface(0).unwrap()).unwrap();

    assert!(tx.socket().nonblocking().unwrap());
    assert!(rx.socket().nonblocking().unwrap());
    rx.socket().set_error_filter(ERR_MASK_ALL).unwrap();
    assert_eq!(rx.socket().error_filter().unwrap(), ERR_MASK_ALL);
}

#[test]
fn non_would_block_write_error_keeps_unknown_submission() {
    let tx = CanTx::from_socket(CanFdSocket::open_iface(0).unwrap()).unwrap();
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let permit = driver
        .prepare_command(Command::ClearErrors, instant(0), instant(20))
        .unwrap();
    let attempt = driver.begin_send(permit, instant(1)).unwrap();
    let id = attempt.id();
    let mut socket_attempt = tx.attempt(attempt);
    let mut times = [instant(4), instant(5)].into_iter();

    assert!(matches!(
        socket_attempt.poll(&mut driver, || times.next().unwrap()),
        Err(SocketCanTxError::Io(_))
    ));
    assert_eq!(driver.report(id).unwrap().state, OperationState::Unknown);
}

#[test]
fn unpolled_synchronous_attempt_can_be_proved_cancelled() {
    let tx = CanTx::from_socket(CanFdSocket::open_iface(0).unwrap()).unwrap();
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let permit = driver
        .prepare_command(Command::ClearErrors, instant(0), instant(20))
        .unwrap();
    let attempt = driver.begin_send(permit, instant(1)).unwrap();
    let id = attempt.id();

    tx.attempt(attempt)
        .cancel_unsubmitted(&mut driver, instant(2), instant(3))
        .unwrap();
    assert_eq!(driver.report(id).unwrap().state, OperationState::Cancelled);
}

#[test]
fn deadline_revokes_an_old_socketcan_attempt_before_any_late_write() {
    let tx = CanTx::from_socket(CanFdSocket::open_iface(0).unwrap()).unwrap();
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let permit = driver
        .prepare_command(Command::ClearErrors, instant(0), instant(5))
        .unwrap();
    let attempt = driver.begin_send(permit, instant(1)).unwrap();
    let id = attempt.id();
    let mut socket_attempt = tx.attempt(attempt);

    driver.tick(instant(5)).unwrap();
    assert!(matches!(
        socket_attempt.poll(&mut driver, || instant(6)),
        Err(SocketCanTxError::Authorize(_))
    ));
    assert_eq!(driver.report(id).unwrap().state, OperationState::Unknown);
}

#[test]
fn unopened_receive_descriptor_reports_its_real_nonblocking_error() {
    let rx = CanRx::from_socket(CanFdSocket::open_iface(0).unwrap()).unwrap();
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());

    assert!(matches!(
        rx.receive(&mut driver, |_| instant(1)),
        Err(error) if error.kind() == ErrorKind::WouldBlock
    ));
}

#[test]
#[ignore = "requires the explicitly created odrive-vcan interface"]
fn vcan_peer_preserves_raw_rx_timestamp_and_local_submission() {
    use std::{thread, time::Duration};

    let peer = CanFdSocket::open("odrive-vcan").unwrap();
    peer.set_nonblocking(true).unwrap();
    let tx = CanTx::open("odrive-vcan").unwrap();
    let rx = CanRx::open("odrive-vcan").unwrap();
    rx.socket().set_recv_timestamp(true).unwrap();
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());

    let unrelated = CanDataFrame::new(unrelated_id(), &[1, 2]).unwrap();
    peer.write_frame(&unrelated).unwrap();
    let received = (0..100)
        .find_map(|_| {
            match rx.receive(&mut driver, |timestamps| {
                assert!(timestamps.socket.is_some());
                instant(2)
            }) {
                Ok(received) => Some(received),
                Err(error) if error.kind() == ErrorKind::WouldBlock => {
                    thread::sleep(Duration::from_millis(1));
                    None
                }
                Err(error) => panic!("vcan receive failed: {error}"),
            }
        })
        .expect("vcan peer frame was not received");
    assert_eq!(received.frame, CanAnyFrame::Normal(unrelated));
    assert_eq!(received.received_at, instant(2));
    assert_eq!(
        received.classification,
        Ok(odrive_can_driver::IngestResult::Unrelated)
    );

    let permit = driver
        .prepare_command(Command::ClearErrors, instant(3), instant(20))
        .unwrap();
    let id = permit.id();
    let attempt = driver.begin_send(permit, instant(4)).unwrap();
    let mut attempt = tx.attempt(attempt);
    let mut times = [instant(5), instant(6), instant(7)].into_iter();
    assert!(matches!(
        attempt.poll(&mut driver, || times.next().unwrap()).unwrap(),
        odrive_can_driver::TxCompletion::Submitted
    ));
    let report = driver.report(id).unwrap();
    assert_eq!(report.state, OperationState::Submitted);
    assert_eq!(report.submitted_at, Some(instant(6)));
    assert_eq!(report.tx_event_at, Some(instant(6)));
    assert_eq!(report.processed_at, instant(7));

    let sent = (0..100)
        .find_map(|_| match peer.read_frame() {
            Ok(frame) => Some(frame),
            Err(error) if error.kind() == ErrorKind::WouldBlock => {
                thread::sleep(Duration::from_millis(1));
                None
            }
            Err(error) => panic!("vcan peer read failed: {error}"),
        })
        .expect("vcan peer did not observe local submission");
    let encoded =
        protocol::encode(node(), protocol::Message::Command(Command::ClearErrors)).unwrap();
    let expected: socketcan::CanFrame = (&encoded).into();
    assert_eq!(sent, CanAnyFrame::from(expected));
}
