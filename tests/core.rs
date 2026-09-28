// Copyright The odrive-can-driver Contributors
//! Core timing and ownership regression tests.

use odrive_can_driver::{
    Driver, Instant, OperationState, ResponseKind, Session, TxCompletion, TxOutcome,
    protocol::{self, Command, NodeId, Query, Response},
};

fn node() -> NodeId {
    NodeId::new(7).unwrap()
}
fn at(us: u64) -> Instant {
    Instant::from_micros(us)
}
fn response_frame(response: Response) -> protocol::EncodedFrame {
    protocol::encode(node(), protocol::Message::Response(response)).unwrap()
}

#[test]
fn rx_before_delayed_tx_backfill_is_retained_and_explicitly_accepted() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let permit = driver
        .prepare_query(Query::MotorError, at(1), at(100))
        .unwrap();
    let id = permit.id();
    let attempt = driver.begin_send(permit, at(2)).unwrap();
    let frame = response_frame(Response::MotorError(0x8000_0001));
    driver.ingest(frame.as_ref(), at(12));
    assert!(driver.pending_response(id).unwrap().is_none());
    assert!(matches!(
        driver
            .finish_tx(
                attempt,
                TxOutcome::Submitted {
                    occurred_at: at(10)
                },
                at(20)
            )
            .unwrap(),
        TxCompletion::Submitted
    ));
    let candidate = driver.pending_response(id).unwrap().unwrap();
    driver.accept_response(candidate, at(21)).unwrap();
    let report = driver.take_report(id).unwrap();
    assert_eq!(report.state, OperationState::Observed);
    assert_eq!(report.submitted_at, Some(at(10)));
    assert_eq!(report.response.unwrap().received_at, at(12));
}

#[test]
fn same_microsecond_response_is_not_auto_matched() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let permit = driver
        .prepare_query(Query::MotorError, at(1), at(100))
        .unwrap();
    let id = permit.id();
    let attempt = driver.begin_send(permit, at(2)).unwrap();
    driver.ingest(response_frame(Response::MotorError(1)).as_ref(), at(10));
    driver
        .finish_tx(
            attempt,
            TxOutcome::Submitted {
                occurred_at: at(10),
            },
            at(11),
        )
        .unwrap();
    assert!(driver.pending_response(id).unwrap().is_none());
}

#[test]
fn rejection_ends_query_without_erasing_submission() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let permit = driver
        .prepare_query(Query::VbusVoltage, at(1), at(100))
        .unwrap();
    let id = permit.id();
    let attempt = driver.begin_send(permit, at(2)).unwrap();
    driver
        .finish_tx(attempt, TxOutcome::Submitted { occurred_at: at(3) }, at(4))
        .unwrap();
    driver.ingest(response_frame(Response::VbusVoltage(48.0)).as_ref(), at(5));
    driver
        .reject_response(driver.pending_response(id).unwrap().unwrap(), at(6))
        .unwrap();
    let report = driver.take_report(id).unwrap();
    assert_eq!(report.state, OperationState::Rejected);
    assert_eq!(report.submitted_at, Some(at(3)));
}

#[test]
fn deadline_revokes_gate_but_preserves_unknown_slot() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let permit = driver
        .prepare_command(Command::ClearErrors, at(1), at(10))
        .unwrap();
    let id = permit.id();
    let attempt = driver.begin_send(permit, at(2)).unwrap();
    driver.tick(at(10)).unwrap();
    assert_eq!(driver.report(id).unwrap().state, OperationState::Unknown);
    assert!(driver.authorize_tx(&attempt, at(11)).is_err());
}

#[test]
fn cache_preserves_timestamp_and_error_bits() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let heartbeat = response_frame(Response::Heartbeat {
        axis_error: 0x8000_0001,
        axis_state: protocol::AxisState::IDLE,
    });
    driver.ingest(heartbeat.as_ref(), at(20));
    let cached = driver.cache().get(ResponseKind::Heartbeat).unwrap();
    assert_eq!(cached.received_at, at(20));
    assert_eq!(
        cached.response,
        Response::Heartbeat {
            axis_error: 0x8000_0001,
            axis_state: protocol::AxisState::IDLE
        }
    );
    assert_eq!(driver.cache().heartbeat_age(at(25)).unwrap().as_micros(), 5);
}

#[test]
fn lost_attempt_cancel_is_unknown_until_acknowledged_then_released() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let permit = driver
        .prepare_command(Command::ClearErrors, at(1), at(100))
        .unwrap();
    let id = permit.id();
    drop(driver.begin_send(permit, at(2)).unwrap());

    driver.cancel(id, at(3)).unwrap();
    assert_eq!(driver.report(id).unwrap().state, OperationState::Unknown);
    assert!(driver.take_report(id).is_err());
    assert_eq!(
        driver.acknowledge_unknown(id).unwrap().state,
        OperationState::Unknown
    );
    assert_eq!(
        driver.take_report(id).unwrap().state,
        OperationState::Unknown
    );
}

#[test]
fn acknowledged_old_attempt_cannot_authorize_after_new_operation() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let permit = driver
        .prepare_command(Command::ClearErrors, at(1), at(100))
        .unwrap();
    let id = permit.id();
    let attempt = driver.begin_send(permit, at(2)).unwrap();
    driver.cancel(id, at(3)).unwrap();
    driver.acknowledge_unknown(id).unwrap();
    driver.take_report(id).unwrap();
    let next = driver
        .prepare_command(Command::ClearErrors, at(4), at(100))
        .unwrap();
    let next_id = next.id();
    assert_ne!(id.get(), next_id.get());
    assert!(driver.authorize_tx(&attempt, at(4)).is_err());
}

#[test]
fn submitted_at_deadline_is_retained_even_when_operation_is_unknown() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let permit = driver
        .prepare_command(Command::ClearErrors, at(1), at(10))
        .unwrap();
    let id = permit.id();
    let attempt = driver.begin_send(permit, at(2)).unwrap();
    assert!(
        driver
            .finish_tx(
                attempt,
                TxOutcome::Submitted {
                    occurred_at: at(10)
                },
                at(11)
            )
            .is_err()
    );
    let report = driver.report(id).unwrap();
    assert_eq!(report.state, OperationState::Unknown);
    assert_eq!(report.submitted_at, Some(at(10)));
    assert_eq!(report.tx_event_at, Some(at(10)));
}

#[test]
fn rollback_keeps_report_processing_time_monotonic_while_retaining_event() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let permit = driver
        .prepare_command(Command::ClearErrors, at(5), at(100))
        .unwrap();
    let id = permit.id();
    let attempt = driver.begin_send(permit, at(6)).unwrap();
    assert!(
        driver
            .finish_tx(attempt, TxOutcome::Submitted { occurred_at: at(7) }, at(4))
            .is_err()
    );
    let report = driver.report(id).unwrap();
    assert_eq!(report.processed_at, at(6));
    assert_eq!(report.submitted_at, Some(at(7)));
}

#[test]
fn would_block_after_cancellation_cannot_reopen_a_retry() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let permit = driver
        .prepare_query(Query::MotorError, at(1), at(100))
        .unwrap();
    let id = permit.id();
    let attempt = driver.begin_send(permit, at(2)).unwrap();
    driver.cancel(id, at(3)).unwrap();
    assert!(matches!(
        driver
            .finish_tx(attempt, TxOutcome::WouldBlock { occurred_at: at(4) }, at(4))
            .unwrap(),
        TxCompletion::Failed
    ));
    assert_eq!(
        driver.take_report(id).unwrap().state,
        OperationState::Cancelled
    );
}

#[test]
fn permit_from_another_session_is_rejected() {
    let mut first_session = Session::new();
    let mut second_session = Session::new();
    let mut first = Driver::new(&mut first_session, node());
    let permit = first
        .prepare_command(Command::ClearErrors, at(1), at(100))
        .unwrap();
    let mut second = Driver::new(&mut second_session, node());
    assert!(second.begin_send(permit, at(2)).is_err());
}

#[test]
fn ignored_candidate_cannot_be_reused_for_an_identical_new_response() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let permit = driver
        .prepare_query(Query::MotorError, at(1), at(100))
        .unwrap();
    let id = permit.id();
    let attempt = driver.begin_send(permit, at(2)).unwrap();
    driver
        .finish_tx(attempt, TxOutcome::Submitted { occurred_at: at(3) }, at(4))
        .unwrap();
    let frame = response_frame(Response::MotorError(7));
    driver.ingest(frame.as_ref(), at(5));
    let old = driver.pending_response(id).unwrap().unwrap();
    driver.ignore_response(old, at(6)).unwrap();
    driver.ingest(frame.as_ref(), at(6));
    let current = driver.pending_response(id).unwrap().unwrap();
    assert!(driver.accept_response(old, at(7)).is_err());
    driver.accept_response(current, at(7)).unwrap();
}

#[test]
fn timeout_allows_late_processing_of_event_time_valid_response() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let permit = driver
        .prepare_query(Query::MotorError, at(1), at(10))
        .unwrap();
    let id = permit.id();
    let attempt = driver.begin_send(permit, at(2)).unwrap();
    driver
        .finish_tx(attempt, TxOutcome::Submitted { occurred_at: at(3) }, at(4))
        .unwrap();
    driver.tick(at(10)).unwrap();
    assert_eq!(driver.report(id).unwrap().state, OperationState::TimedOut);
    driver.ingest(response_frame(Response::MotorError(9)).as_ref(), at(5));
    let candidate = driver.pending_response(id).unwrap().unwrap();
    driver.accept_response(candidate, at(11)).unwrap();
    assert_eq!(
        driver.take_report(id).unwrap().state,
        OperationState::Observed
    );
}

#[test]
fn admission_never_uses_old_cache_wrong_node_type_or_deadline_frames() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let response = response_frame(Response::MotorError(1));
    driver.ingest(response.as_ref(), at(4));
    let permit = driver
        .prepare_query(Query::MotorError, at(5), at(30))
        .unwrap();
    let id = permit.id();
    let attempt = driver.begin_send(permit, at(6)).unwrap();
    driver
        .finish_tx(
            attempt,
            TxOutcome::Submitted {
                occurred_at: at(10),
            },
            at(11),
        )
        .unwrap();
    assert!(driver.pending_response(id).unwrap().is_none());
    let foreign = protocol::encode(
        NodeId::new(8).unwrap(),
        protocol::Message::Response(Response::MotorError(2)),
    )
    .unwrap();
    driver.ingest(foreign.as_ref(), at(12));
    driver.ingest(
        response_frame(Response::Heartbeat {
            axis_error: 0,
            axis_state: protocol::AxisState::IDLE,
        })
        .as_ref(),
        at(13),
    );
    driver.ingest(response.as_ref(), at(10));
    driver.ingest(response.as_ref(), at(30));
    assert!(driver.pending_response(id).unwrap().is_none());
    driver.ingest(response_frame(Response::MotorError(3)).as_ref(), at(20));
    driver.ingest(response.as_ref(), at(9));
    let candidate = driver.pending_response(id).unwrap().unwrap();
    assert_eq!(candidate.response().response, Response::MotorError(3));
    driver.accept_response(candidate, at(21)).unwrap();
}

#[test]
fn cancellation_racing_with_same_microsecond_submission_preserves_the_submission() {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node());
    let permit = driver
        .prepare_command(Command::ClearErrors, at(1), at(30))
        .unwrap();
    let id = permit.id();
    let attempt = driver.begin_send(permit, at(2)).unwrap();
    driver.authorize_tx(&attempt, at(3)).unwrap();
    // A native synchronous call completed before cancellation was processed, but both clocks
    // have the same microsecond value. Its result was backfilled only afterward.
    driver.cancel(id, at(4)).unwrap();
    driver
        .finish_tx(attempt, TxOutcome::Submitted { occurred_at: at(4) }, at(5))
        .unwrap();
    let report = driver.take_report(id).unwrap();
    assert_eq!(report.state, OperationState::Submitted);
    assert_eq!(report.submitted_at, Some(at(4)));
    assert_eq!(report.cancel_requested_at, Some(at(4)));
}
