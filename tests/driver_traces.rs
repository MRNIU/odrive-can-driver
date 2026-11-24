// Copyright The odrive-can-driver Contributors
//! 用与 C Driver 共用的测试向量验证操作时序和报告。

use odrive_can_driver::{
    AttemptError, CachedResponse, Driver, IngestResult, Instant, OperationKind, OperationReport,
    OperationState, ReportError, Session, TxCompletion, TxOutcome,
    protocol::{Command, FrameId, FramePayload, FrameRef, Message, NodeId, Query, Response},
};

const FIXTURE: &str = include_str!("fixtures/driver_traces.tsv");
const MOTOR_ERROR_ID: u16 = 0x0e3;

fn instant(field: &str) -> Instant {
    Instant::from_micros(field.parse().unwrap())
}

fn expected_time(field: &str) -> Option<Instant> {
    (field != "-").then(|| instant(field))
}

fn expected_response(time: &str, bits: &str) -> Option<CachedResponse> {
    if time == "-" {
        assert_eq!(bits, "-");
        return None;
    }
    Some(CachedResponse {
        received_at: instant(time),
        response: Response::MotorError(bits.parse().unwrap()),
    })
}

fn check_report(
    report: OperationReport<'_>,
    kind: OperationKind,
    expectation: &[&str],
    location: &str,
) {
    let expected_state = match expectation[0] {
        "Prepared" => OperationState::Prepared,
        "Cancelled" => OperationState::Cancelled,
        "Unknown" => OperationState::Unknown,
        "Submitted" => OperationState::Submitted,
        "Observed" => OperationState::Observed,
        "TimedOut" => OperationState::TimedOut,
        other => panic!("{location}: invalid report state {other}"),
    };
    assert_eq!(
        report,
        OperationReport {
            id: report.id,
            kind,
            state: expected_state,
            prepared_at: instant(expectation[1]),
            deadline: instant(expectation[2]),
            dispatching_at: expected_time(expectation[3]),
            submitted_at: expected_time(expectation[4]),
            tx_event_at: expected_time(expectation[5]),
            cancel_requested_at: expected_time(expectation[6]),
            terminal_at: expected_time(expectation[7]),
            processed_at: instant(expectation[8]),
            response: expected_response(expectation[9], expectation[10]),
        },
        "{location}"
    );
}

fn run_case<'a>(name: &str, events: &mut impl Iterator<Item = (usize, &'a str)>) {
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, NodeId::new(7).unwrap());
    let (line_number, line) = events.next().unwrap();
    let prepare: Vec<_> = line.split('\t').collect();
    assert_eq!(prepare[0], "prepare", "{name}:{line_number}");
    let now = instant(prepare[2]);
    let deadline = instant(prepare[3]);
    let kind = match prepare[1] {
        "command" => OperationKind::Command(Command::ClearErrors),
        "query" => OperationKind::Query(Query::MotorError),
        other => panic!("{name}:{line_number}: invalid operation kind {other}"),
    };
    let permit = match kind {
        OperationKind::Command(command) => driver.prepare_command(command, now, deadline),
        OperationKind::Query(query) => driver.prepare_query(query, now, deadline),
    }
    .unwrap();
    let id = permit.id();
    let mut permit = Some(permit);
    let mut attempt = None;
    for (line_number, line) in events {
        let location = format!("{name}:{line_number}");
        let fields: Vec<_> = line.split('\t').collect();
        match fields[0] {
            "begin" => {
                attempt = Some(
                    driver
                        .begin_send(permit.take().unwrap(), instant(fields[1]))
                        .unwrap(),
                );
            }
            "gate" => {
                let result = driver.authorize_tx(attempt.as_ref().unwrap(), instant(fields[1]));
                match fields[2] {
                    "ok" => assert!(result.is_ok(), "{location}: {result:?}"),
                    "revoked" => {
                        assert_eq!(
                            result.unwrap_err(),
                            AttemptError::Revoked(OperationState::Unknown),
                            "{location}"
                        );
                    }
                    other => panic!("{location}: invalid gate result {other}"),
                }
            }
            "rx" => {
                let bits: u32 = fields[2].parse().unwrap();
                let mut bytes = [0; 8];
                bytes[..4].copy_from_slice(&bits.to_le_bytes());
                let frame = FrameRef {
                    id: FrameId::Standard(MOTOR_ERROR_ID),
                    payload: FramePayload::Data(&bytes),
                };
                assert_eq!(
                    driver.ingest(frame, instant(fields[1])),
                    IngestResult::Message(Message::Response(Response::MotorError(bits))),
                    "{location}"
                );
            }
            "submit" | "block" => {
                let occurred_at = instant(fields[1]);
                let outcome = if fields[0] == "submit" {
                    TxOutcome::Submitted { occurred_at }
                } else {
                    TxOutcome::WouldBlock { occurred_at }
                };
                let result = driver.finish_tx(attempt.take().unwrap(), outcome, instant(fields[2]));
                match fields[3] {
                    "ok" => {
                        assert!(
                            matches!(result, Ok(TxCompletion::Submitted)),
                            "{location}: {result:?}"
                        );
                    }
                    "rollback" => {
                        assert!(
                            matches!(result, Err(AttemptError::ClockRollback { .. })),
                            "{location}: {result:?}"
                        );
                    }
                    "late" => {
                        assert_eq!(
                            result.unwrap_err(),
                            AttemptError::EventAfterDeadline,
                            "{location}"
                        );
                    }
                    "retry" => {
                        let Ok(TxCompletion::Retry(retry)) = result else {
                            panic!("{location}: expected retry, got {result:?}");
                        };
                        assert_eq!(retry.id(), id, "{location}");
                        permit = Some(retry);
                    }
                    "failed" => {
                        assert!(
                            matches!(result, Ok(TxCompletion::Failed)),
                            "{location}: {result:?}"
                        );
                    }
                    other => panic!("{location}: invalid TX result {other}"),
                }
            }
            "tick" => driver.tick(instant(fields[1])).unwrap(),
            "cancel" => driver.cancel(id, instant(fields[1])).unwrap(),
            "peek" => {
                let candidate = driver.pending_response(id).unwrap();
                assert_eq!(
                    candidate.map(|current| current.response()),
                    expected_response(fields[1], fields[2]),
                    "{location}"
                );
            }
            "accept" => {
                let candidate = driver.pending_response(id).unwrap().unwrap();
                driver
                    .accept_response(candidate, instant(fields[1]))
                    .unwrap();
            }
            "report" | "take" => {
                let result = if fields[0] == "report" {
                    driver.report(id)
                } else {
                    driver.take_report(id)
                };
                if fields[1] == "unknown_pending" {
                    assert_eq!(result, Err(ReportError::UnknownPending), "{location}");
                } else {
                    let report = result.unwrap();
                    assert_eq!(report.id, id, "{location}");
                    check_report(report, kind, &fields[1..], &location);
                }
            }
            "missing" => {
                assert_eq!(
                    driver.report(id),
                    Err(ReportError::StaleOperation),
                    "{location}"
                );
                assert_eq!(
                    driver.pending_response(id).unwrap_err(),
                    ReportError::StaleOperation,
                    "{location}"
                );
            }
            "ack" => {
                let report = driver.acknowledge_unknown(id).unwrap();
                assert_eq!(report.state, OperationState::Unknown, "{location}");
            }
            other => panic!("{location}: invalid event {other}"),
        }
    }
}

#[test]
fn shared_driver_traces() {
    let mut events = FIXTURE
        .lines()
        .enumerate()
        .filter(|(_, line)| !line.is_empty() && !line.starts_with('#'))
        .map(|(number, line)| (number + 1, line))
        .peekable();
    let mut cases = 0;
    while let Some((line_number, line)) = events.next() {
        let name = line
            .strip_prefix("case\t")
            .unwrap_or_else(|| panic!("line {line_number}: expected case"));
        run_case(
            name,
            &mut std::iter::from_fn(|| events.next_if(|(_, next)| !next.starts_with("case\t"))),
        );
        cases += 1;
    }
    assert!(cases > 0);
}
