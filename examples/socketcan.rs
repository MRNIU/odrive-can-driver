// Copyright The odrive-can-driver Contributors
//! Linux SocketCAN split-endpoint ODrive CLI.
//!
//! ```text
//! socketcan <interface> <node> <heartbeat|vbus|encoder|motor-error|encoder-error|clear-errors|idle|velocity> [velocity_turn_per_s [torque_ff_nm]]
//! ```
//!
//! The TX and RX sockets are independently owned. This single loop drains one RX frame, advances
//! deadlines, and advances one short TX syscall without allowing a waiting send to hold `Driver`.
//! It retains every native RX frame and its `CanTimestamps`; the simple query policy accepts the
//! first core-eligible response, while a product may ignore or reject it using source-continuity
//! or old-frame rules.

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("socketcan example only supports Linux SocketCAN");
    std::process::exit(2);
}

#[cfg(target_os = "linux")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    use std::{
        env, fmt, io, thread,
        time::{Duration, Instant as StdInstant},
    };

    use odrive_can_driver::{
        Driver, Instant, OperationState, ResponseKind, Session, TxCompletion,
        protocol::{AxisState, Command, NodeId, Query},
        socketcan::{CanRx, CanRxFrame, CanTx, SocketCanTxError},
    };
    use socketcan::{SocketOptions, id::ERR_MASK_ALL};

    const TIMEOUT: Duration = Duration::from_secs(1);
    const IDLE_WAIT: Duration = Duration::from_millis(1);

    enum Action {
        Heartbeat,
        Query(Query),
        Command(Command),
    }

    fn usage() -> &'static str {
        "usage: socketcan <interface> <node 0..63> \\
         <heartbeat|vbus|encoder|motor-error|encoder-error|clear-errors|idle|velocity> \\
         [velocity_turn_per_s [torque_ff_nm]]"
    }

    fn core_error(error: impl fmt::Debug) -> io::Error {
        io::Error::other(format!("driver state error: {error:?}"))
    }

    fn parse_number(value: Option<String>, name: &str) -> Result<f32, Box<dyn std::error::Error>> {
        value
            .ok_or_else(|| format!("missing {name}"))?
            .parse()
            .map_err(|error| format!("invalid {name}: {error}").into())
    }

    fn parse_action(
        operation: String,
        arguments: &mut impl Iterator<Item = String>,
    ) -> Result<Action, Box<dyn std::error::Error>> {
        let action = match operation.as_str() {
            "heartbeat" | "state" => Action::Heartbeat,
            "vbus" => Action::Query(Query::VbusVoltage),
            "encoder" => Action::Query(Query::EncoderEstimates),
            "motor-error" => Action::Query(Query::MotorError),
            "encoder-error" => Action::Query(Query::EncoderError),
            "clear-errors" => Action::Command(Command::ClearErrors),
            "idle" => Action::Command(Command::SetAxisRequestedState {
                state: AxisState::IDLE,
            }),
            "velocity" => {
                let velocity = parse_number(arguments.next(), "velocity_turn_per_s")?;
                let torque_ff = match arguments.next() {
                    Some(value) => value
                        .parse()
                        .map_err(|error| format!("invalid torque_ff_nm: {error}"))?,
                    None => 0.0,
                };
                Action::Command(Command::SetInputVel {
                    velocity,
                    torque_ff,
                })
            }
            _ => return Err(usage().into()),
        };
        if arguments.next().is_some() {
            return Err(usage().into());
        }
        Ok(action)
    }

    fn log_receive(received: CanRxFrame) {
        match received.classification {
            Ok(odrive_can_driver::IngestResult::Message(message)) => {
                eprintln!(
                    "ODrive message at {} us: {message:?}; native={:?}; timestamps={:?}",
                    received.received_at.as_micros(),
                    received.frame,
                    received.timestamps
                );
            }
            Ok(odrive_can_driver::IngestResult::Unrelated) => {
                eprintln!(
                    "unrelated native SocketCAN frame={:?}; timestamps={:?}",
                    received.frame, received.timestamps
                );
            }
            Ok(odrive_can_driver::IngestResult::DecodeError(error)) => {
                eprintln!(
                    "unsupported ODrive frame ({error:?}); native={:?}; timestamps={:?}",
                    received.frame, received.timestamps
                );
            }
            Err(error) => eprintln!(
                "SocketCAN error notification ({error}); native={:?}; timestamps={:?}",
                received.frame, received.timestamps
            ),
        }
    }

    let mut arguments = env::args().skip(1);
    let interface = arguments.next().ok_or_else(usage)?;
    let node_raw = arguments.next().ok_or_else(usage)?;
    let operation = arguments.next().ok_or_else(usage)?;
    let node = NodeId::new(node_raw.parse()?).map_err(core_error)?;
    let action = parse_action(operation, &mut arguments)?;

    let started = StdInstant::now();
    let now =
        || Instant::from_micros(u64::try_from(started.elapsed().as_micros()).unwrap_or(u64::MAX));
    let tx = CanTx::open(&interface)?;
    let rx = CanRx::open(&interface)?;
    // These options apply only to this RX descriptor, never to the shared interface.
    rx.socket().set_error_filter(ERR_MASK_ALL)?;
    rx.socket().set_recv_timestamp(true)?;
    let mut session = Session::new();
    let mut driver = Driver::new(&mut session, node);

    if matches!(action, Action::Heartbeat) {
        let deadline = Instant::from_micros(
            now()
                .as_micros()
                .saturating_add(u64::try_from(TIMEOUT.as_micros()).unwrap_or(u64::MAX)),
        );
        loop {
            if now() >= deadline {
                return Err(
                    io::Error::new(io::ErrorKind::TimedOut, "no new ODrive Heartbeat").into(),
                );
            }
            match rx.receive(&mut driver, |_| now()) {
                Ok(received) => log_receive(received),
                Err(error) if error.kind() == io::ErrorKind::WouldBlock => thread::sleep(IDLE_WAIT),
                Err(error) => return Err(error.into()),
            }
            if let Some(entry) = driver
                .cache()
                .get(ResponseKind::Heartbeat)
                .filter(|entry| entry.received_at < deadline)
            {
                println!(
                    "observed Heartbeat at {} us: {:?}",
                    entry.received_at.as_micros(),
                    entry.response
                );
                return Ok(());
            }
        }
    }

    let deadline = Instant::from_micros(
        now()
            .as_micros()
            .saturating_add(u64::try_from(TIMEOUT.as_micros()).unwrap_or(u64::MAX)),
    );
    let (permit, is_query) = match action {
        Action::Query(query) => (
            driver
                .prepare_query(query, now(), deadline)
                .map_err(core_error)?,
            true,
        ),
        Action::Command(command) => (
            driver
                .prepare_command(command, now(), deadline)
                .map_err(core_error)?,
            false,
        ),
        Action::Heartbeat => unreachable!(),
    };
    let id = permit.id();
    let mut sending = Some(tx.attempt(driver.begin_send(permit, now()).map_err(core_error)?));

    loop {
        driver.tick(now()).map_err(core_error)?;

        if sending.is_some() {
            let result = sending.as_mut().expect("checked").poll(&mut driver, &now);
            match result {
                Ok(TxCompletion::Submitted) => sending = None,
                Ok(TxCompletion::Retry(permit)) => {
                    sending =
                        Some(tx.attempt(driver.begin_send(permit, now()).map_err(core_error)?));
                }
                Ok(TxCompletion::Failed) => sending = None,
                Err(SocketCanTxError::Io(error)) => {
                    return Err(io::Error::other(format!(
                        "SocketCAN write is Unknown ({error}); do not retry automatically"
                    ))
                    .into());
                }
                Err(error) => return Err(io::Error::other(error).into()),
            }
        }

        match rx.receive(&mut driver, |_| now()) {
            Ok(received) => log_receive(received),
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => {}
            Err(error) => return Err(error.into()),
        }

        if is_query && let Some(candidate) = driver.pending_response(id).map_err(core_error)? {
            // This CLI has no source-continuity rule, so it accepts only the core-eligible
            // candidate. A product can call ignore_response or reject_response here instead.
            driver
                .accept_response(candidate, now())
                .map_err(core_error)?;
        }

        let report = driver.report(id).map_err(core_error)?;
        match report.state {
            OperationState::Submitted if !is_query => {
                let report = driver.take_report(id).map_err(core_error)?;
                println!(
                    "locally Submitted {:?} at {:?} us; this is not an ODrive ACK",
                    report.kind,
                    report.submitted_at.map(Instant::as_micros)
                );
                return Ok(());
            }
            OperationState::Observed | OperationState::Rejected => {
                let report = driver.take_report(id).map_err(core_error)?;
                println!(
                    "query {:?} ended as {:?}; response={:?}",
                    report.kind, report.state, report.response
                );
                return Ok(());
            }
            OperationState::TimedOut | OperationState::Failed | OperationState::Cancelled => {
                let report = driver.take_report(id).map_err(core_error)?;
                return Err(io::Error::other(format!(
                    "operation ended as {:?}: {:?}",
                    report.state, report
                ))
                .into());
            }
            OperationState::Unknown => {
                // Every SocketCAN call above has already returned. Discard the old wrapper, then
                // explicitly release isolation; this never establishes that the command was absent.
                let _ = sending.take();
                driver.acknowledge_unknown(id).map_err(core_error)?;
                let report = driver.take_report(id).map_err(core_error)?;
                return Err(io::Error::other(format!(
                    "operation is Unknown ({report:?}); do not retry automatically"
                ))
                .into());
            }
            OperationState::Prepared | OperationState::Dispatching | OperationState::Submitted => {
                thread::sleep(IDLE_WAIT);
            }
        }
    }
}
