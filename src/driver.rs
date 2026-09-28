// Copyright The odrive-can-driver Contributors
//! Core state, time, TX authorization and RX cache.
use crate::protocol::{self, Command, EncodedFrame, FrameRef, Message, NodeId, Query, Response};
use core::ptr;

/// A microsecond timestamp in one monotonic clock domain.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Instant(u64);
impl Instant {
    /** Builds a timestamp from microseconds. */
    pub const fn from_micros(v: u64) -> Self {
        Self(v)
    }
    /** Returns microseconds. */
    pub const fn as_micros(self) -> u64 {
        self.0
    }
    /** Computes non-negative elapsed time. */
    pub const fn duration_since(self, before: Self) -> Option<Duration> {
        match self.0.checked_sub(before.0) {
            Some(v) => Some(Duration(v)),
            None => None,
        }
    }
}
/// A microsecond duration.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Duration(u64);
impl Duration {
    /** Builds a duration from microseconds. */
    pub const fn from_micros(v: u64) -> Self {
        Self(v)
    }
    /** Returns microseconds. */
    pub const fn as_micros(self) -> u64 {
        self.0
    }
}

/// Linear identity owned by the application for the lifetime of its driver.
///
/// One session is exclusively leased by a driver and all identities it creates. An old permit
/// cannot be carried into a reconstructed driver, even after dropping the original driver:
///
/// ```compile_fail
/// use odrive_can_driver::{Driver, Instant, Session, protocol::{Command, NodeId}};
/// let mut session = Session::new();
/// let node = NodeId::new(1).unwrap();
/// let mut first = Driver::new(&mut session, node);
/// let permit = first.prepare_command(Command::ClearErrors,
///     Instant::from_micros(1), Instant::from_micros(100)).unwrap();
/// drop(first);
/// let mut replacement = Driver::new(&mut session, node);
/// replacement.begin_send(permit, Instant::from_micros(2)).unwrap();
/// ```
#[derive(Debug)]
pub struct Session {
    _identity: u8,
}
impl Default for Session {
    fn default() -> Self {
        Self::new()
    }
}
impl Session {
    /** Creates a session. */
    pub const fn new() -> Self {
        Self { _identity: 0 }
    }
}
/// A session-bound operation identity.
#[derive(Clone, Copy, Debug)]
pub struct OperationId<'s> {
    session: &'s Session,
    sequence: u64,
}
impl PartialEq for OperationId<'_> {
    fn eq(&self, other: &Self) -> bool {
        self.sequence == other.sequence && ptr::eq(self.session, other.session)
    }
}
impl Eq for OperationId<'_> {}
impl OperationId<'_> {
    /** Returns the session-local log sequence. */
    pub const fn get(self) -> u64 {
        self.sequence
    }
}
/// The direction and payload of a local operation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum OperationKind {
    /** A host command. */
    Command(Command),
    /** An RTR query. */
    Query(Query),
}
/// Local operation state; `Submitted` is never device acknowledgement.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OperationState {
    /// Encoded; no native attempt is active.
    Prepared,
    /// An independent TX attempt is unresolved.
    Dispatching,
    /// Locally accepted; terminal for commands, waiting for queries.
    Submitted,
    /// The application accepted a matching response.
    Observed,
    /// The application explicitly ended the query by rejecting a response.
    Rejected,
    /// A backend proved no submission, or preparation for I/O failed.
    Failed,
    /// The deadline expired; inspect submitted_at for existing submission.
    TimedOut,
    /// Cancellation completed; submitted_at distinguishes a submitted query from unsent work.
    Cancelled,
    /// A side effect may have occurred; explicit acknowledgement is required to free the slot.
    Unknown,
}
/// A snapshot with event times and separately recorded processing time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OperationReport<'s> {
    /** Operation identity. */
    pub id: OperationId<'s>,
    /** Protocol direction. */
    pub kind: OperationKind,
    /** Absolute deadline. */
    pub deadline: Instant,
    /** Preparation event time. */
    pub prepared_at: Instant,
    /** Most recent dispatch processing time. */
    pub dispatching_at: Option<Instant>,
    /** Local submit event time. */
    pub submitted_at: Option<Instant>,
    /// Last TX completion event, including a proven unsubmitted result.
    pub tx_event_at: Option<Instant>,
    /// Time of the application's cancellation request; submission may have won the race.
    pub cancel_requested_at: Option<Instant>,
    /// Terminal event time (deadline, accepted RX, cancellation, or TX event).
    pub terminal_at: Option<Instant>,
    /** Last core processing time, which never moves backwards. */
    pub processed_at: Instant,
    /** Current state. */
    pub state: OperationState,
    /** Accepted or explicitly rejected response; unrelated cache entries are not copied here. */
    pub response: Option<CachedResponse>,
}
/// Preparation failure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PrepareError {
    /// An operation or untaken report occupies the sole slot.
    Busy,
    /// The absolute deadline is no later than preparation.
    DeadlineElapsed {
        /// Preparation time.
        now: Instant,
        /// Exclusive absolute deadline.
        deadline: Instant,
    },
    /// The processing clock moved backwards.
    ClockRollback {
        /// Last accepted processing time.
        previous: Instant,
        /// Rejected processing time.
        now: Instant,
    },
    /// The protocol rejected the payload.
    Encode(protocol::EncodeError),
    /// Operation identity space is exhausted; identities never wrap.
    IdExhausted,
}
/// Cannot consume a permit to start TX.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BeginSendError {
    /// The permit belongs to another session or an old operation.
    StalePermit,
    /// The operation is no longer prepared.
    NotPrepared(OperationState),
    /// Clock rollback; the consumed permit becomes an unsent Failed report.
    ClockRollback {
        /// Last accepted processing time.
        previous: Instant,
        /// Rejected processing time.
        now: Instant,
    },
    /// The deadline elapsed before I/O; a TimedOut report is available.
    DeadlineElapsed,
}
/// TX authorization or backfill failure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AttemptError {
    /// Another session, operation or attempt owns this identity.
    StaleAttempt,
    /// Do not poll native TX again.
    Revoked(OperationState),
    /// The result clock regressed; known submission and monotonic report time are retained.
    ClockRollback {
        /// Last accepted processing time.
        previous: Instant,
        /// Rejected processing time.
        now: Instant,
    },
    /// A proven submission occurred at or after the deadline; submission evidence is retained.
    EventAfterDeadline,
    /// The supplied TX event precedes dispatch or is later than result processing.
    InvalidEventTime,
}
/// Report access failure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReportError {
    /// The identity or candidate is foreign, stale, or already taken.
    StaleOperation,
    /// The operation has not reached a takeable terminal state.
    Pending,
    /// The caller must first resolve possible late TX and acknowledge uncertainty.
    UnknownPending,
    /// Result processing uses a clock earlier than the last processing call.
    ClockRollback {
        /// Last processing time.
        previous: Instant,
        /// Supplied processing time.
        now: Instant,
    },
    /// A receive event cannot be accepted before it has occurred in the shared clock domain.
    FutureResponse,
}
/// Decoding result, while caller retains its complete native frame/error.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum IngestResult {
    /// Different node or unsupported command number.
    Unrelated,
    /// Decoded message; responses update cache but require explicit query admission.
    Message(Message),
    /// Malformed or unsupported frame; the original native frame stays caller-owned.
    DecodeError(protocol::DecodeError),
}
/// Fixed cache category.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResponseKind {
    /// Periodic heartbeat.
    Heartbeat,
    /// Motor error bits.
    MotorError,
    /// Encoder error bits.
    EncoderError,
    /// Sensorless error bits.
    SensorlessError,
    /// Encoder position and velocity.
    EncoderEstimates,
    /// Encoder counts.
    EncoderCount,
    /// Setpoint and measured q-axis current.
    Iq,
    /// Sensorless position and velocity.
    SensorlessEstimates,
    /// DC bus voltage.
    VbusVoltage,
}
impl ResponseKind {
    fn ix(self) -> usize {
        match self {
            Self::Heartbeat => 0,
            Self::MotorError => 1,
            Self::EncoderError => 2,
            Self::SensorlessError => 3,
            Self::EncoderEstimates => 4,
            Self::EncoderCount => 5,
            Self::Iq => 6,
            Self::SensorlessEstimates => 7,
            Self::VbusVoltage => 8,
        }
    }
    fn of(r: Response) -> Self {
        match r {
            Response::Heartbeat { .. } => Self::Heartbeat,
            Response::MotorError(_) => Self::MotorError,
            Response::EncoderError(_) => Self::EncoderError,
            Response::SensorlessError(_) => Self::SensorlessError,
            Response::EncoderEstimates { .. } => Self::EncoderEstimates,
            Response::EncoderCount { .. } => Self::EncoderCount,
            Response::Iq { .. } => Self::Iq,
            Response::SensorlessEstimates { .. } => Self::SensorlessEstimates,
            Response::VbusVoltage(_) => Self::VbusVoltage,
        }
    }
}
/// A decoded response and the application-supplied receive timestamp.
///
/// This may be a mapped native timestamp or a dequeue observation; it does not prove device freshness.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CachedResponse {
    /** Decoded response. */
    pub response: Response,
    /** Receive event timestamp. */
    pub received_at: Instant,
}
/// One most-recent response per protocol type.
#[derive(Clone, Copy, Debug, Default)]
pub struct ResponseCache {
    entries: [Option<CachedResponse>; 9],
}
impl ResponseCache {
    /** Gets cached response. */
    pub fn get(&self, k: ResponseKind) -> Option<CachedResponse> {
        self.entries[k.ix()]
    }
    /** Gets heartbeat age. */
    pub fn heartbeat_age(&self, n: Instant) -> Option<Duration> {
        self.get(ResponseKind::Heartbeat)
            .and_then(|x| n.duration_since(x.received_at))
    }
    /** Tests heartbeat freshness. */
    pub fn heartbeat_is_fresh(&self, n: Instant, max: Duration) -> bool {
        matches!(self.heartbeat_age(n),Some(v) if v<=max)
    }
    fn put(&mut self, r: Response, t: Instant) {
        let i = ResponseKind::of(r).ix();
        if self.entries[i].is_none_or(|x| x.received_at <= t) {
            self.entries[i] = Some(CachedResponse {
                response: r,
                received_at: t,
            })
        }
    }
}
/// Non-copy single-use permission to initiate one TX attempt.
#[derive(Debug)]
#[must_use]
pub struct SendPermit<'s> {
    id: OperationId<'s>,
}
impl<'s> SendPermit<'s> {
    /** Gets owning operation. */
    pub const fn id(&self) -> OperationId<'s> {
        self.id
    }
}
/// Non-copy native TX attempt; its frame is only available through the per-poll gate.
#[derive(Debug)]
#[must_use]
pub struct TxAttempt<'s> {
    id: OperationId<'s>,
    sequence: u64,
}
impl<'s> TxAttempt<'s> {
    /** Gets owning operation. */
    pub const fn id(&self) -> OperationId<'s> {
        self.id
    }
}
/// Result supplied only after native TX future has ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TxOutcome {
    /// Known local acceptance; not an ODrive acknowledgement.
    Submitted {
        /// Time the native result was observed in the shared monotonic clock domain.
        occurred_at: Instant,
    },
    /// The ended attempt provably did not enqueue; the caller may retry only with the returned permit.
    WouldBlock {
        /// Time the native result was observed in the shared monotonic clock domain.
        occurred_at: Instant,
    },
    /// The ended attempt provably did not enqueue and failed.
    NotSubmitted {
        /// Time the native result was observed in the shared monotonic clock domain.
        occurred_at: Instant,
    },
}
/// Next local step after final TX future result.
#[derive(Debug)]
pub enum TxCompletion<'s> {
    /// Submission evidence was recorded; consult the operation report for query/cancellation state.
    Submitted,
    /// A new single-use permit after proof of no enqueue; no automatic retry is performed.
    Retry(SendPermit<'s>),
    /// No retry permit is returned; inspect the terminal report.
    Failed,
}
/// A particular cached query response presented to application policy.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ResponseCandidate<'s> {
    id: OperationId<'s>,
    sequence: u64,
    response: CachedResponse,
}
impl ResponseCandidate<'_> {
    /** Gets cached response. */
    pub const fn response(self) -> CachedResponse {
        self.response
    }
}
struct Active<'s> {
    report: OperationReport<'s>,
    frame: EncodedFrame,
    attempt: u64,
    unknown_acknowledged: bool,
    candidate_sequence: u64,
    candidate: Option<ResponseCandidate<'s>>,
}
/// One-slot node driver with no CAN controller, allocator, lock or executor.
///
/// The exclusive Session borrow remains live through every permit, attempt and operation ID.
/// Native I/O is polled outside the driver, after a short [`Self::authorize_tx`] call.
/// All timestamps share one monotonic microsecond domain. Event times may precede processing.
pub struct Driver<'s> {
    session: &'s Session,
    node: NodeId,
    next_id: u64,
    last: Option<Instant>,
    active: Option<Active<'s>>,
    cache: ResponseCache,
}
impl<'s> Driver<'s> {
    /// Exclusively leases a session. It cannot be rebuilt while any old identity remains usable.
    pub const fn new(session: &'s mut Session, node: NodeId) -> Self {
        Self {
            session,
            node,
            next_id: 0,
            last: None,
            active: None,
            cache: ResponseCache { entries: [None; 9] },
        }
    }
    /// Returns the node filtered by the protocol decoder.
    pub const fn node(&self) -> NodeId {
        self.node
    }
    /// Returns latest response values independently of query completion.
    pub const fn cache(&self) -> &ResponseCache {
        &self.cache
    }
    /// Encodes a command without I/O. The deadline is an exclusive absolute timestamp.
    ///
    /// Actually sending an addressed command feeds the supported firmware's watchdog.
    /// Local submission is not device execution or acknowledgement.
    pub fn prepare_command(
        &mut self,
        command: Command,
        now: Instant,
        deadline: Instant,
    ) -> Result<SendPermit<'s>, PrepareError> {
        self.prepare(OperationKind::Command(command), now, deadline)
    }
    /// Encodes an RTR query without I/O. Sending it also feeds the device watchdog.
    pub fn prepare_query(
        &mut self,
        query: Query,
        now: Instant,
        deadline: Instant,
    ) -> Result<SendPermit<'s>, PrepareError> {
        self.prepare(OperationKind::Query(query), now, deadline)
    }
    /// Consumes the one-use permit. No native I/O has occurred on an error.
    ///
    /// A consumed permit rejected for rollback leaves a retrievable `Failed` report.
    pub fn begin_send(
        &mut self,
        permit: SendPermit<'s>,
        at: Instant,
    ) -> Result<TxAttempt<'s>, BeginSendError> {
        if !self.owns(permit.id) {
            return Err(BeginSendError::StalePermit);
        }
        let active = self
            .active
            .as_ref()
            .filter(|x| x.report.id == permit.id)
            .ok_or(BeginSendError::StalePermit)?;
        if active.report.state != OperationState::Prepared {
            return Err(BeginSendError::NotPrepared(active.report.state));
        }
        if let Err((previous, now)) = self.time(at) {
            self.finish(OperationState::Failed, previous, previous, None);
            return Err(BeginSendError::ClockRollback { previous, now });
        }
        if at >= self.active.as_ref().unwrap().report.deadline {
            let deadline = self.active.as_ref().unwrap().report.deadline;
            self.finish(OperationState::TimedOut, deadline, at, None);
            return Err(BeginSendError::DeadlineElapsed);
        }
        let active = self.active.as_mut().unwrap();
        let Some(sequence) = active.attempt.checked_add(1) else {
            self.finish(OperationState::Failed, at, at, None);
            return Err(BeginSendError::StalePermit);
        };
        active.attempt = sequence;
        active.candidate = None;
        active.report.state = OperationState::Dispatching;
        active.report.dispatching_at = Some(at);
        active.report.processed_at = at;
        Ok(TxAttempt {
            id: permit.id,
            sequence,
        })
    }
    /// Authorizes exactly the next native poll; the returned frame is not an independent permit.
    ///
    /// A backend MUST call this immediately before EVERY native TX poll, without a suspension
    /// between authorization and poll. It must not retain the frame for an ungated later send.
    /// On error, stop/drop the native future. Already queued or independently progressing I/O
    /// remains unknown; closing this gate does not retract a hardware queue.
    pub fn authorize_tx(
        &mut self,
        attempt: &TxAttempt<'s>,
        at: Instant,
    ) -> Result<EncodedFrame, AttemptError> {
        self.check(attempt)?;
        if let Err((previous, now)) = self.time(at) {
            self.mark_unknown(previous);
            return Err(AttemptError::ClockRollback { previous, now });
        }
        let active = self.active.as_mut().unwrap();
        if at >= active.report.deadline {
            active.report.state = OperationState::Unknown;
        }
        active.report.processed_at = at;
        if active.report.state != OperationState::Dispatching {
            return Err(AttemptError::Revoked(active.report.state));
        }
        Ok(active.frame)
    }
    /// Consumes an ended native attempt and records its event separately from processing time.
    ///
    /// `WouldBlock` and `NotSubmitted` require backend proof of no enqueue and no future later
    /// submission. A late processing call is legal. Invalid event clocks retain actual submission
    /// evidence but isolate the operation as `Unknown`; no error authorizes an automatic retry.
    pub fn finish_tx(
        &mut self,
        attempt: TxAttempt<'s>,
        outcome: TxOutcome,
        at: Instant,
    ) -> Result<TxCompletion<'s>, AttemptError> {
        self.check(&attempt)?;
        let rollback = self.time(at).err();
        let processed = self.last.unwrap();
        let event = match outcome {
            TxOutcome::Submitted { occurred_at }
            | TxOutcome::WouldBlock { occurred_at }
            | TxOutcome::NotSubmitted { occurred_at } => occurred_at,
        };
        let active = self.active.as_mut().unwrap();
        active.report.tx_event_at = Some(event);
        active.report.processed_at = processed;
        if matches!(outcome, TxOutcome::Submitted { .. }) {
            active.report.submitted_at = Some(event);
        }
        let invalid = event < active.report.dispatching_at.unwrap() || event > at;
        let cancelled = active.report.cancel_requested_at;
        let deadline = active.report.deadline;
        let submitted = matches!(outcome, TxOutcome::Submitted { .. });
        if submitted && (invalid || rollback.is_some() || event >= deadline) {
            self.mark_unknown(processed);
        } else if submitted {
            let command = matches!(active.report.kind, OperationKind::Command(_));
            if command {
                self.finish(OperationState::Submitted, event, processed, None);
            } else if let Some(cancelled) = cancelled {
                self.finish(OperationState::Cancelled, cancelled, processed, None);
            } else if processed >= deadline {
                self.finish(OperationState::TimedOut, deadline, processed, None);
            } else {
                active.report.state = OperationState::Submitted;
                active.report.terminal_at = None;
            }
        } else if let Some(cancelled) = cancelled {
            self.finish(OperationState::Cancelled, cancelled, processed, None);
        } else if processed >= deadline {
            self.finish(OperationState::TimedOut, deadline, processed, None);
        } else if invalid || rollback.is_some() || matches!(outcome, TxOutcome::NotSubmitted { .. })
        {
            self.finish(OperationState::Failed, event, processed, None);
        } else {
            active.report.state = OperationState::Prepared;
            active.report.terminal_at = None;
            active.candidate = None;
        }
        if let Some((previous, now)) = rollback {
            return Err(AttemptError::ClockRollback { previous, now });
        }
        if invalid {
            return Err(AttemptError::InvalidEventTime);
        }
        if submitted && event >= deadline {
            return Err(AttemptError::EventAfterDeadline);
        }
        if submitted {
            return Ok(TxCompletion::Submitted);
        }
        if self.active.as_ref().unwrap().report.state == OperationState::Prepared {
            Ok(TxCompletion::Retry(SendPermit { id: attempt.id }))
        } else {
            Ok(TxCompletion::Failed)
        }
    }
    /// Records proved cancellation without submission, after the native future has ended.
    ///
    /// Only a backend with evidence for this exact attempt may call this. Ordinary cancellation,
    /// dropping a generic future, or reaching a deadline is not that evidence. A proof after the
    /// deadline remains valid: the report is `TimedOut`, with no submission, rather than unknown.
    pub fn cancel_unsubmitted(
        &mut self,
        attempt: TxAttempt<'s>,
        occurred_at: Instant,
        at: Instant,
    ) -> Result<(), AttemptError> {
        self.check(&attempt)?;
        let rollback = self.time(at).err();
        let processed = self.last.unwrap();
        let active = self.active.as_mut().unwrap();
        active.report.tx_event_at = Some(occurred_at);
        let invalid = occurred_at < active.report.dispatching_at.unwrap() || occurred_at > at;
        let deadline = active.report.deadline;
        let cancelled = active.report.cancel_requested_at;
        let (state, event) = if let Some(cancelled) = cancelled {
            (OperationState::Cancelled, cancelled)
        } else if processed >= deadline {
            (OperationState::TimedOut, deadline)
        } else {
            (OperationState::Cancelled, occurred_at)
        };
        self.finish(state, event, processed, None);
        if let Some((previous, now)) = rollback {
            Err(AttemptError::ClockRollback { previous, now })
        } else if invalid {
            Err(AttemptError::InvalidEventTime)
        } else {
            Ok(())
        }
    }
    /// Consumes an ended native future whose submission result is uncertain.
    ///
    /// This records `Unknown`, not cancellation. It does not retract queued frames.
    pub fn abandon_attempt(
        &mut self,
        attempt: TxAttempt<'s>,
        at: Instant,
    ) -> Result<(), AttemptError> {
        self.check(&attempt)?;
        let rollback = self.time(at).err();
        self.mark_unknown(self.last.unwrap());
        match rollback {
            Some((previous, now)) => Err(AttemptError::ClockRollback { previous, now }),
            None => Ok(()),
        }
    }
    /// Revokes future sends. A live/lost attempt remains unknown until its I/O is resolved.
    ///
    /// A prepared operation is provably unsent. Cancelling an already submitted query ends its
    /// wait and retains `submitted_at`. At equal microsecond values actual submission still wins.
    pub fn cancel(&mut self, id: OperationId<'s>, at: Instant) -> Result<(), ReportError> {
        self.report(id)?;
        self.time(at)
            .map_err(|(previous, now)| ReportError::ClockRollback { previous, now })?;
        let active = self.active.as_mut().unwrap();
        match active.report.state {
            OperationState::Prepared => {
                active.report.cancel_requested_at = Some(at);
                self.finish(OperationState::Cancelled, at, at, None);
            }
            OperationState::Dispatching | OperationState::Unknown => {
                active.report.cancel_requested_at.get_or_insert(at);
                self.mark_unknown(at);
            }
            OperationState::Submitted if matches!(active.report.kind, OperationKind::Query(_)) => {
                active.report.cancel_requested_at = Some(at);
                self.finish(OperationState::Cancelled, at, at, None);
            }
            _ => {}
        }
        Ok(())
    }
    /// Releases unknown isolation only after the application has ended all original TX work
    /// and dealt with any controller queue which could still emit that frame.
    ///
    /// This is an explicit application assertion, NOT proof of non-submission. The returned
    /// report remains `Unknown`. Subsequent polls with the old attempt fail the gate even if a
    /// new operation is prepared. Do not use this as permission to retry an unknown side effect.
    pub fn acknowledge_unknown(
        &mut self,
        id: OperationId<'s>,
    ) -> Result<OperationReport<'s>, ReportError> {
        let report = self.report(id)?;
        if report.state != OperationState::Unknown {
            return Err(ReportError::Pending);
        }
        // Keep the terminal report until take_report, but distinguish acknowledged uncertainty.
        self.active.as_mut().unwrap().unknown_acknowledged = true;
        Ok(report)
    }
    /// Decodes and caches a received frame. No successful decode completes a query by itself.
    ///
    /// Native frames/errors remain caller-owned. Timestamp mapping and source continuity are
    /// application responsibilities. Responses arriving before TX backfill are retained for
    /// later matching; older frames cannot replace a newer provisional candidate.
    pub fn ingest(&mut self, frame: FrameRef<'_>, received_at: Instant) -> IngestResult {
        let message = match protocol::decode(self.node, frame) {
            Ok(Some(m)) => m,
            Ok(None) => return IngestResult::Unrelated,
            Err(e) => return IngestResult::DecodeError(e),
        };
        if let Message::Response(response) = message {
            self.cache.put(response, received_at);
            self.candidate(response, received_at);
        }
        IngestResult::Message(message)
    }
    /// Returns the latest eligible candidate after known submission.
    ///
    /// RX must be strictly later than submission and strictly before deadline. Equal microsecond
    /// timestamps are ambiguous and excluded. Event-time-valid delayed RX/TX may still be accepted
    /// after tick reports a timeout, until the application takes that report. Cancellation/rejection
    /// cannot be undone by later feedback. No pre-existing cache is searched to complete a query.
    pub fn pending_response(
        &self,
        id: OperationId<'s>,
    ) -> Result<Option<ResponseCandidate<'s>>, ReportError> {
        self.report(id)?;
        let active = self.active.as_ref().unwrap();
        Ok(active.candidate.filter(|c| Self::eligible(active, c)))
    }
    /// Accepts only the current eligible candidate; caller policy can restrict but never broaden it.
    pub fn accept_response(
        &mut self,
        candidate: ResponseCandidate<'s>,
        at: Instant,
    ) -> Result<(), ReportError> {
        self.decide(candidate, at, Some(OperationState::Observed))
    }
    /// Discards this candidate while preserving the cache and prior submission.
    pub fn ignore_response(
        &mut self,
        candidate: ResponseCandidate<'s>,
        at: Instant,
    ) -> Result<(), ReportError> {
        self.decide(candidate, at, None)
    }
    /// Ends a query as `Rejected`, preserving both the rejected response and submission evidence.
    pub fn reject_response(
        &mut self,
        candidate: ResponseCandidate<'s>,
        at: Instant,
    ) -> Result<(), ReportError> {
        self.decide(candidate, at, Some(OperationState::Rejected))
    }
    /// Advances the processing clock and deadline without performing I/O.
    ///
    /// An unresolved attempt becomes `Unknown`, closing future polls while retaining its slot.
    /// A query timeout is event-time tentative until take_report, permitting delayed RX backfill.
    pub fn tick(&mut self, at: Instant) -> Result<(), PrepareError> {
        self.time(at)
            .map_err(|(previous, now)| PrepareError::ClockRollback { previous, now })?;
        let Some(active) = self.active.as_mut() else {
            return Ok(());
        };
        active.report.processed_at = at;
        if at < active.report.deadline {
            return Ok(());
        }
        let deadline = active.report.deadline;
        match active.report.state {
            OperationState::Prepared | OperationState::Submitted
                if !matches!(active.report.kind, OperationKind::Command(_))
                    || active.report.state == OperationState::Prepared =>
            {
                self.finish(OperationState::TimedOut, deadline, at, None);
            }
            OperationState::Dispatching => self.mark_unknown(at),
            _ => {}
        }
        Ok(())
    }
    /// Reads the current operation, including an untaken terminal report.
    pub fn report(&self, id: OperationId<'s>) -> Result<OperationReport<'s>, ReportError> {
        self.active
            .as_ref()
            .filter(|x| self.owns(id) && x.report.id == id)
            .map(|x| x.report)
            .ok_or(ReportError::StaleOperation)
    }
    /// Takes a terminal report and releases the sole slot. This finalizes any tentative timeout.
    pub fn take_report(&mut self, id: OperationId<'s>) -> Result<OperationReport<'s>, ReportError> {
        let report = self.report(id)?;
        match report.state {
            OperationState::Unknown if !self.active.as_ref().unwrap().unknown_acknowledged => {
                return Err(ReportError::UnknownPending);
            }
            OperationState::Dispatching => return Err(ReportError::UnknownPending),
            OperationState::Prepared => return Err(ReportError::Pending),
            OperationState::Submitted if matches!(report.kind, OperationKind::Query(_)) => {
                return Err(ReportError::Pending);
            }
            _ => {}
        }
        self.active = None;
        Ok(report)
    }
    fn prepare(
        &mut self,
        kind: OperationKind,
        now: Instant,
        deadline: Instant,
    ) -> Result<SendPermit<'s>, PrepareError> {
        if self.active.is_some() {
            return Err(PrepareError::Busy);
        }
        if let Some(previous) = self.last.filter(|p| now < *p) {
            return Err(PrepareError::ClockRollback { previous, now });
        }
        if now >= deadline {
            return Err(PrepareError::DeadlineElapsed { now, deadline });
        }
        let frame = protocol::encode(
            self.node,
            match kind {
                OperationKind::Command(c) => Message::Command(c),
                OperationKind::Query(q) => Message::Request(q),
            },
        )
        .map_err(PrepareError::Encode)?;
        let sequence = self.next_id;
        self.next_id = sequence.checked_add(1).ok_or(PrepareError::IdExhausted)?;
        self.last = Some(now);
        let id = OperationId {
            session: self.session,
            sequence,
        };
        self.active = Some(Active {
            report: OperationReport {
                id,
                kind,
                deadline,
                prepared_at: now,
                dispatching_at: None,
                submitted_at: None,
                tx_event_at: None,
                cancel_requested_at: None,
                terminal_at: None,
                processed_at: now,
                state: OperationState::Prepared,
                response: None,
            },
            frame,
            attempt: 0,
            candidate_sequence: 0,
            candidate: None,
            unknown_acknowledged: false,
        });
        Ok(SendPermit { id })
    }
    fn candidate(&mut self, response: Response, received_at: Instant) {
        let Some(active) = self.active.as_mut() else {
            return;
        };
        if !matches!(active.report.kind, OperationKind::Query(q) if matches_response(q, response))
            || !matches!(
                active.report.state,
                OperationState::Dispatching
                    | OperationState::Unknown
                    | OperationState::Submitted
                    | OperationState::TimedOut
            )
            || !active
                .report
                .dispatching_at
                .is_some_and(|t| received_at >= t)
            || received_at >= active.report.deadline
            || active
                .candidate
                .is_some_and(|c| c.response.received_at > received_at)
        {
            return;
        }
        let Some(sequence) = active.candidate_sequence.checked_add(1) else {
            return;
        };
        active.candidate_sequence = sequence;
        active.candidate = Some(ResponseCandidate {
            id: active.report.id,
            sequence,
            response: CachedResponse {
                response,
                received_at,
            },
        });
    }
    fn eligible(active: &Active<'s>, candidate: &ResponseCandidate<'s>) -> bool {
        matches!(
            active.report.state,
            OperationState::Submitted | OperationState::TimedOut
        ) && active.report.cancel_requested_at.is_none()
            && active
                .report
                .submitted_at
                .is_some_and(|t| candidate.response.received_at > t)
            && candidate.response.received_at < active.report.deadline
    }
    fn decide(
        &mut self,
        candidate: ResponseCandidate<'s>,
        at: Instant,
        decision: Option<OperationState>,
    ) -> Result<(), ReportError> {
        self.report(candidate.id)?;
        let active = self.active.as_ref().unwrap();
        if !active
            .candidate
            .is_some_and(|c| c.id == candidate.id && c.sequence == candidate.sequence)
            || !Self::eligible(active, &candidate)
        {
            return Err(ReportError::StaleOperation);
        }
        if candidate.response.received_at > at {
            return Err(ReportError::FutureResponse);
        }
        self.time(at)
            .map_err(|(previous, now)| ReportError::ClockRollback { previous, now })?;
        if let Some(state) = decision {
            self.finish(
                state,
                candidate.response.received_at,
                at,
                Some(candidate.response),
            );
        } else {
            let active = self.active.as_mut().unwrap();
            active.candidate = None;
            active.report.processed_at = at;
        }
        Ok(())
    }
    fn check(&self, attempt: &TxAttempt<'s>) -> Result<(), AttemptError> {
        let active = self.active.as_ref().ok_or(AttemptError::StaleAttempt)?;
        if !self.owns(attempt.id)
            || active.report.id != attempt.id
            || active.attempt != attempt.sequence
        {
            return Err(AttemptError::StaleAttempt);
        }
        if !matches!(
            active.report.state,
            OperationState::Dispatching | OperationState::Unknown
        ) || active.unknown_acknowledged
        {
            return Err(AttemptError::Revoked(active.report.state));
        }
        Ok(())
    }
    fn mark_unknown(&mut self, at: Instant) {
        let active = self.active.as_mut().unwrap();
        active.report.state = OperationState::Unknown;
        active.report.terminal_at = Some(at);
        active.report.processed_at = at;
    }
    fn finish(
        &mut self,
        state: OperationState,
        event: Instant,
        processed: Instant,
        response: Option<CachedResponse>,
    ) {
        let active = self.active.as_mut().unwrap();
        active.report.state = state;
        active.report.terminal_at = Some(event);
        active.report.processed_at = processed;
        active.report.response = response;
    }
    fn owns(&self, id: OperationId<'s>) -> bool {
        ptr::eq(id.session, self.session)
    }
    fn time(&mut self, now: Instant) -> Result<(), (Instant, Instant)> {
        if let Some(previous) = self.last.filter(|p| now < *p) {
            return Err((previous, now));
        }
        self.last = Some(now);
        Ok(())
    }
}
fn matches_response(q: Query, r: Response) -> bool {
    matches!(
        (q, r),
        (Query::MotorError, Response::MotorError(_))
            | (Query::EncoderError, Response::EncoderError(_))
            | (Query::SensorlessError, Response::SensorlessError(_))
            | (Query::EncoderEstimates, Response::EncoderEstimates { .. })
            | (Query::EncoderCount, Response::EncoderCount { .. })
            | (Query::Iq, Response::Iq { .. })
            | (
                Query::SensorlessEstimates,
                Response::SensorlessEstimates { .. }
            )
            | (Query::VbusVoltage, Response::VbusVoltage(_))
    )
}
