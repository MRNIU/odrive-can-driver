// Copyright The odrive-can-driver Contributors

//! Shared-core behavior tests.

use odrive_can_driver::{
    Driver, IngestResult, OperationKind, OperationState, PrepareError, ReportError, ResponseKind,
    protocol::{self, Command, FramePayload, FrameRef, NodeId, Query, Response},
};
use std::{
    future::Future,
    task::{Context, Poll, Waker},
};

fn node() -> NodeId {
    NodeId::new(7).unwrap()
}

fn response_frame(response: Response) -> protocol::EncodedFrame {
    protocol::encode(node(), protocol::Message::Response(response)).unwrap()
}

#[test]
fn invalid_prepare_rejects_before_encoding_or_consuming_an_id() {
    let mut driver = Driver::new(node());
    let invalid = Command::SetInputVel {
        velocity: f32::NAN,
        torque_ff: 0.0,
    };

    assert!(matches!(
        driver.prepare_command(invalid, 4, 10),
        Err(PrepareError::Encode(
            protocol::EncodeError::NonFinite { .. }
        ))
    ));
    assert!(matches!(
        driver.prepare_query(Query::MotorError, 10, 10),
        Err(PrepareError::DeadlineElapsed { .. })
    ));

    assert_eq!(
        driver
            .prepare_command(Command::ClearErrors, 4, 10)
            .unwrap()
            .get(),
        0
    );
}

#[test]
fn ingest_classifies_unrelated_remote_and_fd_frames() {
    let mut driver = Driver::new(node());
    let unrelated = FrameRef {
        id: protocol::FrameId::Standard((8 << 5) | 3),
        payload: FramePayload::Data(&[0; 8]),
    };
    assert_eq!(driver.ingest(unrelated, 2), IngestResult::Unrelated);

    let request = FrameRef {
        id: protocol::FrameId::Standard((7 << 5) | 3),
        payload: FramePayload::Remote { dlc: 8 },
    };
    assert_eq!(
        driver.ingest(request, 3),
        IngestResult::Message(protocol::Message::Request(Query::MotorError))
    );

    let fd = FrameRef {
        id: protocol::FrameId::Standard((7 << 5) | 1),
        payload: FramePayload::Fd(&[0; 8]),
    };
    assert_eq!(
        driver.ingest(fd, 4),
        IngestResult::DecodeError(protocol::DecodeError::UnsupportedCanFd)
    );
}

#[test]
fn cache_keeps_newest_heartbeat_and_all_error_bits() {
    let mut driver = Driver::new(node());
    let newest = response_frame(Response::Heartbeat {
        axis_error: 0x8000_0001,
        axis_state: protocol::AxisState::IDLE,
    });
    let older = response_frame(Response::Heartbeat {
        axis_error: 0,
        axis_state: protocol::AxisState::CLOSED_LOOP_CONTROL,
    });

    driver.ingest(newest.as_ref(), 20);
    driver.ingest(older.as_ref(), 19);
    let cached = driver.cache().get(ResponseKind::Heartbeat).unwrap();
    assert_eq!(cached.received_at_ms, 20);
    assert_eq!(
        cached.response,
        Response::Heartbeat {
            axis_error: 0x8000_0001,
            axis_state: protocol::AxisState::IDLE,
        }
    );
    assert_eq!(driver.cache().heartbeat_age_ms(25), Some(5));
    assert!(driver.cache().heartbeat_is_fresh(25, 5));
    assert!(!driver.cache().heartbeat_is_fresh(26, 5));
    assert_eq!(driver.cache().heartbeat_age_ms(19), None);
}

#[test]
fn would_block_returns_the_operation_to_prepared_for_a_retry() {
    let mut driver = Driver::new(node());
    let id = driver.prepare_query(Query::MotorError, 0, 20).unwrap();
    driver.begin_send(id, 1).unwrap().would_block(2).unwrap();
    assert_eq!(driver.report(id).unwrap().state, OperationState::Prepared);
    let attempt = driver.begin_send(id, 3).unwrap();
    assert!(attempt.frame().is_remote());
    attempt.not_sent(4).unwrap();
}

#[test]
fn dropped_send_guard_is_unknown_and_blocks_the_single_slot_until_acknowledged() {
    let mut driver = Driver::new(node());
    let id = driver.prepare_command(Command::ClearErrors, 0, 20).unwrap();
    {
        let _attempt = driver.begin_send(id, 1).unwrap();
    }
    assert_eq!(driver.report(id).unwrap().state, OperationState::Unknown);
    assert!(matches!(
        driver.prepare_command(Command::ClearErrors, 2, 20),
        Err(PrepareError::Busy)
    ));
    assert_eq!(driver.take_report(id), Err(ReportError::UnknownPending));
    let report = driver.acknowledge_unknown(id).unwrap();
    assert_eq!(report.state, OperationState::Unknown);
    assert_eq!(
        driver.take_report(id).unwrap().state,
        OperationState::Unknown
    );
}

#[test]
fn deadlines_distinguish_never_dispatched_from_submitted_queries() {
    let mut unsent = Driver::new(node());
    let unstarted = unsent.prepare_query(Query::MotorError, 0, 10).unwrap();
    unsent.tick(10).unwrap();
    let report = unsent.take_report(unstarted).unwrap();
    assert_eq!(report.state, OperationState::TimedOut);
    assert_eq!(report.submitted_at_ms, None);

    let mut submitted = Driver::new(node());
    let id = submitted.prepare_query(Query::MotorError, 0, 10).unwrap();
    submitted.begin_send(id, 9).unwrap().submitted(9).unwrap();
    submitted.tick(10).unwrap();
    let report = submitted.take_report(id).unwrap();
    assert_eq!(report.state, OperationState::TimedOut);
    assert_eq!(report.submitted_at_ms, Some(9));
}

#[test]
fn submitted_at_deadline_or_with_a_rolled_clock_is_unknown_but_keeps_submission_evidence() {
    let mut at_deadline = Driver::new(node());
    let id = at_deadline
        .prepare_command(Command::ClearErrors, 0, 10)
        .unwrap();
    assert_eq!(
        at_deadline.begin_send(id, 9).unwrap().submitted(10),
        Err(odrive_can_driver::AttemptError::DeadlineElapsed)
    );
    let report = at_deadline.report(id).unwrap();
    assert_eq!(report.state, OperationState::Unknown);
    assert_eq!(report.submitted_at_ms, Some(10));
    at_deadline.acknowledge_unknown(id).unwrap();
    at_deadline.take_report(id).unwrap();
    assert!(matches!(
        at_deadline.prepare_command(Command::ClearErrors, 9, 20),
        Err(PrepareError::ClockRollback { .. })
    ));
    at_deadline
        .prepare_command(Command::ClearErrors, 10, 20)
        .unwrap();

    let mut rollback = Driver::new(node());
    let rollback_id = rollback
        .prepare_command(Command::ClearErrors, 5, 20)
        .unwrap();
    assert_eq!(
        rollback.begin_send(rollback_id, 6).unwrap().submitted(5),
        Err(odrive_can_driver::AttemptError::ClockRollback)
    );
    let report = rollback.report(rollback_id).unwrap();
    assert_eq!(report.state, OperationState::Unknown);
    assert_eq!(report.submitted_at_ms, Some(5));
}

#[test]
fn known_not_sent_with_a_rolled_clock_is_failed_at_the_last_known_time() {
    let mut driver = Driver::new(node());
    let id = driver.prepare_command(Command::ClearErrors, 5, 20).unwrap();
    assert_eq!(
        driver.begin_send(id, 6).unwrap().not_sent(5),
        Err(odrive_can_driver::AttemptError::ClockRollback)
    );
    let report = driver.take_report(id).unwrap();
    assert_eq!(report.state, OperationState::Failed);
    assert_eq!(report.terminal_at_ms, Some(6));
}

#[test]
fn would_block_with_a_rolled_clock_returns_to_prepared_without_rewinding_time() {
    let mut driver = Driver::new(node());
    let id = driver.prepare_query(Query::MotorError, 5, 20).unwrap();
    assert_eq!(
        driver.begin_send(id, 6).unwrap().would_block(5),
        Err(odrive_can_driver::AttemptError::ClockRollback)
    );
    assert_eq!(driver.report(id).unwrap().state, OperationState::Prepared);
    assert!(matches!(
        driver.begin_send(id, 5),
        Err(odrive_can_driver::BeginSendError::ClockRollback { .. })
    ));
    driver.begin_send(id, 6).unwrap().not_sent(7).unwrap();
}

#[test]
fn known_not_sent_at_deadline_is_failed_not_timed_out() {
    let mut driver = Driver::new(node());
    let id = driver.prepare_command(Command::ClearErrors, 0, 10).unwrap();
    assert_eq!(
        driver.begin_send(id, 9).unwrap().not_sent(10),
        Err(odrive_can_driver::AttemptError::DeadlineElapsed)
    );
    assert_eq!(
        driver.take_report(id).unwrap().state,
        OperationState::Failed
    );
}

#[test]
fn cancel_only_applies_to_prepared_operations() {
    let mut prepared = Driver::new(node());
    let id = prepared.prepare_query(Query::MotorError, 0, 10).unwrap();
    prepared.cancel(id, 1).unwrap();
    assert_eq!(
        prepared.take_report(id).unwrap().state,
        OperationState::Cancelled
    );

    let mut submitted = Driver::new(node());
    let id = submitted.prepare_query(Query::MotorError, 0, 10).unwrap();
    submitted.begin_send(id, 1).unwrap().submitted(1).unwrap();
    assert!(matches!(
        submitted.cancel(id, 2),
        Err(odrive_can_driver::BeginSendError::NotPrepared(
            OperationState::Submitted
        ))
    ));
}

#[test]
fn heartbeat_never_observes_a_submitted_write() {
    let mut driver = Driver::new(node());
    let id = driver.prepare_command(Command::ClearErrors, 0, 10).unwrap();
    driver.begin_send(id, 1).unwrap().submitted(1).unwrap();
    let heartbeat = response_frame(Response::Heartbeat {
        axis_error: 0,
        axis_state: protocol::AxisState::IDLE,
    });
    driver.ingest(heartbeat.as_ref(), 2);
    assert_eq!(driver.report(id).unwrap().state, OperationState::Submitted);
}

#[test]
fn an_unpolled_send_future_keeps_the_operation_prepared() {
    let mut driver = Driver::new(node());
    let id = driver.prepare_command(Command::ClearErrors, 0, 10).unwrap();
    let future = async {
        let _attempt = driver.begin_send(id, 1).unwrap();
        core::future::pending::<()>().await;
    };

    drop(future);
    assert_eq!(driver.report(id).unwrap().state, OperationState::Prepared);
}

#[test]
fn a_polled_send_future_dropped_while_dispatching_is_unknown() {
    let mut driver = Driver::new(node());
    let id = driver.prepare_command(Command::ClearErrors, 0, 10).unwrap();
    {
        let future = async {
            let _attempt = driver.begin_send(id, 1).unwrap();
            core::future::pending::<()>().await;
        };
        let mut future = core::pin::pin!(future);
        let waker = Waker::noop();
        let mut context = Context::from_waker(waker);
        assert_eq!(future.as_mut().poll(&mut context), Poll::Pending);
    }
    assert_eq!(driver.report(id).unwrap().state, OperationState::Unknown);
}

#[test]
fn query_observation_requires_a_matching_response_after_submit_and_before_deadline() {
    let mut driver = Driver::new(node());
    let id = driver.prepare_query(Query::MotorError, 0, 20).unwrap();
    driver.begin_send(id, 10).unwrap().submitted(10).unwrap();
    let response = response_frame(Response::MotorError(0x4000_0000));

    driver.ingest(response.as_ref(), 10);
    let report = driver.take_report(id).unwrap();
    assert_eq!(report.state, OperationState::Observed);
    assert_eq!(report.response, Some(Response::MotorError(0x4000_0000)));

    let mut late = Driver::new(node());
    let late_id = late.prepare_query(Query::MotorError, 0, 20).unwrap();
    late.begin_send(late_id, 10).unwrap().submitted(10).unwrap();
    late.ingest(response.as_ref(), 20);
    assert_eq!(
        late.report(late_id).unwrap().state,
        OperationState::Submitted
    );
    late.tick(20).unwrap();
    assert_eq!(
        late.take_report(late_id).unwrap().state,
        OperationState::TimedOut
    );
}

#[test]
fn completed_report_ids_become_stale_after_a_later_completion() {
    let mut driver = Driver::new(node());
    let first = driver.prepare_command(Command::ClearErrors, 0, 10).unwrap();
    driver.begin_send(first, 1).unwrap().submitted(1).unwrap();
    assert!(matches!(
        driver.prepare_command(Command::ClearErrors, 2, 10),
        Err(PrepareError::Busy)
    ));
    assert_eq!(
        driver.take_report(first).unwrap().state,
        OperationState::Submitted
    );
    let second = driver.prepare_command(Command::ClearErrors, 2, 10).unwrap();
    driver.begin_send(second, 3).unwrap().submitted(3).unwrap();

    assert_eq!(driver.report(first), Err(ReportError::StaleOperation));
    let report = driver.take_report(second).unwrap();
    assert_eq!(report.kind, OperationKind::Command(Command::ClearErrors));
}
