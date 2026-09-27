// Copyright The odrive-can-driver Contributors

//! 单节点操作追踪与回复缓存。

use crate::protocol::{self, Command, EncodedFrame, FrameRef, Message, NodeId, Query, Response};

/// 单调时钟域中的不透明操作标识。
///
/// 此值只在创建它的同一 [`Driver`] 实例内有效；不得跨 driver、重建后的 driver 或设备
/// 会话保存后复用。
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct OperationId(u64);

impl OperationId {
    /// 返回仅用于日志和报告关联的数值。
    pub const fn get(self) -> u64 {
        self.0
    }
}

/// 已准备操作的协议方向。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum OperationKind {
    /// 主机写入命令。
    Command(Command),
    /// 主机 RTR 查询。
    Query(Query),
}

/// 操作当前或最终的本地状态。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OperationState {
    /// 已编码，但尚未交给收发后端。
    Prepared,
    /// `SendAttempt` 正在借用 driver；其析构前无法创建另一操作。
    Dispatching,
    /// 本地收发后端已接受帧。写命令到此终态；查询仍等待回复或超时。
    Submitted,
    /// 提交后的匹配回复已被观察到。
    Observed,
    /// 收发后端明确报告该次尝试失败。
    Failed,
    /// 在期限内没有得到所需的本地进展或查询回复。
    ///
    /// `submitted_at_ms == None` 表示后端确定没有接受该帧；`Some(_)` 表示查询曾在本地
    /// 提交但未及时观察到回复。发送结果不确定时使用 [`OperationState::Unknown`]。
    TimedOut,
    /// 尚未发送的操作被调用方取消。
    Cancelled,
    /// 发送可能已经发生，但 driver 无法证明最终收发结果。
    Unknown,
}

/// 单一操作的可复制快照。
///
/// `Submitted` 是本地队列接受结果，不是设备 ACK。`Observed` 只表示在提交后观察到
/// 同节点、同回复类型的帧；CANSimple 没有线上请求 ID，因此它也不是因果 ACK。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OperationReport {
    /// 操作标识。
    pub id: OperationId,
    /// 操作的协议方向和内容。
    pub kind: OperationKind,
    /// 调用方提供的独立传输期限，使用与 `now_ms` 相同的单调时钟。
    pub deadline_ms: u64,
    /// 成功准备帧的时刻。
    pub prepared_at_ms: u64,
    /// 最近一次开始发送尝试的时刻；此值不表示后端已经接受该帧。
    pub dispatching_at_ms: Option<u64>,
    /// 已知本地后端接受帧的时刻。
    pub submitted_at_ms: Option<u64>,
    /// 终态被记录的时刻；已知未发送但回调时间回退时使用最后已知的单调时刻。
    pub terminal_at_ms: Option<u64>,
    /// 当前或最终的操作状态。
    pub state: OperationState,
    /// `Observed` 查询的帧内容。
    pub response: Option<Response>,
}

/// 准备操作时的错误。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PrepareError {
    /// 上一个 `Unknown` 或未完成操作仍占用唯一槽位。
    Busy,
    /// `now_ms` 已达到或超过期限，不能建立可发送操作。
    DeadlineElapsed {
        /// 调用方的当前时刻。
        now_ms: u64,
        /// 调用方的期限。
        deadline_ms: u64,
    },
    /// 调用方的本地时钟回退。
    ClockRollback {
        /// 上次成功提交给 driver 的本地时刻。
        previous_ms: u64,
        /// 本次调用的时刻。
        now_ms: u64,
    },
    /// 协议编码拒绝输入，例如 NaN 或无穷浮点。
    Encode(protocol::EncodeError),
    /// 不透明标识空间已经耗尽。
    IdExhausted,
}

/// 开始发送时的错误。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BeginSendError {
    /// 标识不属于当前活动操作。
    StaleOperation,
    /// 操作不再处于 `Prepared`。
    NotPrepared(OperationState),
    /// 调用方的本地时钟回退。
    ClockRollback {
        /// 上次成功提交给 driver 的本地时刻。
        previous_ms: u64,
        /// 本次调用的时刻。
        now_ms: u64,
    },
    /// 尚未调用后端前期限已到；操作已成为 `TimedOut` 报告。
    DeadlineElapsed,
}

/// 结束一个发送尝试时的错误。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AttemptError {
    /// 回调时钟回退；已提交为 `Unknown`，明确未送为 `Failed`，会阻塞则保持 `Prepared`。
    ClockRollback,
    /// 回调达到期限；已提交为 `Unknown`，明确未送为 `Failed`，会阻塞则为 `TimedOut`。
    DeadlineElapsed,
}

/// 查询或提取报告时的错误。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReportError {
    /// 标识已被更新的报告淘汰，或从未属于此 driver。
    StaleOperation,
    /// 操作仍未到可提取的终态。
    Pending,
    /// `Unknown` 必须先由调用方确认底层没有待发或迟到发送，再提取报告。
    UnknownPending,
}

/// 输入帧的分类结果。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum IngestResult {
    /// 帧属于其他节点或当前协议版本未知的命令号。
    Unrelated,
    /// 成功解码的同节点协议消息。
    Message(Message),
    /// 帧 ID、种类或长度不符合当前协议版本。
    DecodeError(protocol::DecodeError),
}

/// 回复在固定缓存中的类别。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResponseKind {
    /// 周期性心跳。
    Heartbeat,
    /// 电机错误位图。
    MotorError,
    /// 编码器错误位图。
    EncoderError,
    /// 无感估算器错误位图。
    SensorlessError,
    /// 编码器位置和速度。
    EncoderEstimates,
    /// 编码器计数。
    EncoderCount,
    /// q 轴电流。
    Iq,
    /// 无感估算位置和速度。
    SensorlessEstimates,
    /// 母线电压。
    VbusVoltage,
}

impl ResponseKind {
    const fn index(self) -> usize {
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

    fn from_response(response: Response) -> Self {
        match response {
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

/// 带接收时间戳的缓存回复。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CachedResponse {
    /// 解码后的完整回复；错误位和未知轴状态按协议层原样保留。
    pub response: Response,
    /// CAN 接收路径提供的时间戳。
    pub received_at_ms: u64,
}

/// 每种协议回复各一项的无分配缓存。
#[derive(Clone, Copy, Debug, Default)]
pub struct ResponseCache {
    entries: [Option<CachedResponse>; 9],
}

impl ResponseCache {
    /// 返回一种回复的最新条目。
    pub fn get(&self, kind: ResponseKind) -> Option<CachedResponse> {
        self.entries[kind.index()]
    }

    /// 返回心跳的年龄；若 `now_ms` 早于其接收时间戳则返回 `None`。
    ///
    /// 此方法不更新 driver 的操作时钟。`now_ms` 必须与接收时间戳使用同一单调时钟域。
    pub fn heartbeat_age_ms(&self, now_ms: u64) -> Option<u64> {
        self.get(ResponseKind::Heartbeat)
            .and_then(|entry| now_ms.checked_sub(entry.received_at_ms))
    }

    /// 返回是否存在年龄不超过 `max_age_ms` 的心跳。
    pub fn heartbeat_is_fresh(&self, now_ms: u64, max_age_ms: u64) -> bool {
        matches!(self.heartbeat_age_ms(now_ms), Some(age) if age <= max_age_ms)
    }

    fn update(&mut self, response: Response, received_at_ms: u64) {
        let index = ResponseKind::from_response(response).index();
        if self.entries[index].is_none_or(|cached| cached.received_at_ms <= received_at_ms) {
            self.entries[index] = Some(CachedResponse {
                response,
                received_at_ms,
            });
        }
    }
}

struct ActiveOperation {
    report: OperationReport,
    frame: EncodedFrame,
}

/// 一个 ODrive 节点的单槽 CANSimple driver。
///
/// `now_ms`、`deadline_ms` 和接收时间戳必须来自同一不回退单调时钟。`ingest` 不推进也不
/// 回退检查操作时钟，因此硬件 RX 时间早于随后处理它的循环 `now_ms` 仍是合法样本。
pub struct Driver {
    node: NodeId,
    next_id: u64,
    last_now_ms: Option<u64>,
    active: Option<ActiveOperation>,
    completed: Option<OperationReport>,
    cache: ResponseCache,
}

impl Driver {
    /// 为一个已验证的 ODrive 节点创建 driver。
    pub const fn new(node: NodeId) -> Self {
        Self {
            node,
            next_id: 0,
            last_now_ms: None,
            active: None,
            completed: None,
            cache: ResponseCache { entries: [None; 9] },
        }
    }

    /// 返回此 driver 过滤的节点号。
    pub const fn node(&self) -> NodeId {
        self.node
    }

    /// 返回只读的九类回复缓存。
    pub const fn cache(&self) -> &ResponseCache {
        &self.cache
    }

    /// 准备一个写命令，不发送帧也不借用传输后端。
    ///
    /// 时间或编码无效会在分配操作 ID 前返回，因此不会占用 ID 或改变活动操作。
    pub fn prepare_command(
        &mut self,
        command: Command,
        now_ms: u64,
        deadline_ms: u64,
    ) -> Result<OperationId, PrepareError> {
        self.prepare(OperationKind::Command(command), now_ms, deadline_ms)
    }

    /// 准备一个 RTR 查询，不发送帧也不借用传输后端。
    pub fn prepare_query(
        &mut self,
        query: Query,
        now_ms: u64,
        deadline_ms: u64,
    ) -> Result<OperationId, PrepareError> {
        self.prepare(OperationKind::Query(query), now_ms, deadline_ms)
    }

    /// 将一个已准备操作转换为独占 `SendAttempt`。
    ///
    /// 只有拥有该 guard 才能读取编码帧和提交收发结果。guard 未以结果方法结束时会把操作
    /// 标记为 `Unknown`，防止取消 future 或不确定的驱动错误被误认为未发送。
    pub fn begin_send(
        &mut self,
        id: OperationId,
        now_ms: u64,
    ) -> Result<SendAttempt<'_>, BeginSendError> {
        let Some(active) = self.active.as_ref() else {
            return Err(BeginSendError::StaleOperation);
        };
        if active.report.id != id {
            return Err(BeginSendError::StaleOperation);
        }
        if active.report.state != OperationState::Prepared {
            return Err(BeginSendError::NotPrepared(active.report.state));
        }
        if let Some(previous_ms) = self.clock_rollback(now_ms) {
            return Err(BeginSendError::ClockRollback {
                previous_ms,
                now_ms,
            });
        }
        if now_ms >= active.report.deadline_ms {
            self.commit_clock(now_ms);
            self.finish_active(OperationState::TimedOut, now_ms, None);
            return Err(BeginSendError::DeadlineElapsed);
        }

        self.commit_clock(now_ms);
        let active = self
            .active
            .as_mut()
            .expect("active operation checked above");
        active.report.state = OperationState::Dispatching;
        active.report.dispatching_at_ms = Some(now_ms);
        Ok(SendAttempt {
            driver: self,
            id,
            settled: false,
        })
    }

    /// 处理一个硬件接收路径借用的帧。
    ///
    /// 每个成功解码的回复更新对应缓存。查询仅在提交后、收到严格早于 deadline 的同类型
    /// 回复时成为 `Observed`；这是一种观察关联，不能证明回复由该次 RTR 触发。
    pub fn ingest(&mut self, frame: FrameRef<'_>, received_at_ms: u64) -> IngestResult {
        let message = match protocol::decode(self.node, frame) {
            Ok(Some(message)) => message,
            Ok(None) => return IngestResult::Unrelated,
            Err(error) => return IngestResult::DecodeError(error),
        };

        if let Message::Response(response) = message {
            self.cache.update(response, received_at_ms);
            self.observe_query(response, received_at_ms);
        }
        IngestResult::Message(message)
    }

    /// 推进操作时钟，并将到期的已准备或已提交查询变为 `TimedOut`。
    ///
    /// `Dispatching` 只能在 `SendAttempt` 借用期间存在；该 guard 被取消或遗失会先变为
    /// `Unknown`，而不是由此方法伪造未发送结论。
    pub fn tick(&mut self, now_ms: u64) -> Result<(), PrepareError> {
        if let Some(previous_ms) = self.clock_rollback(now_ms) {
            return Err(PrepareError::ClockRollback {
                previous_ms,
                now_ms,
            });
        }
        self.commit_clock(now_ms);
        if self.active.as_ref().is_some_and(|active| {
            now_ms >= active.report.deadline_ms
                && matches!(
                    active.report.state,
                    OperationState::Prepared | OperationState::Submitted
                )
        }) {
            self.finish_active(OperationState::TimedOut, now_ms, None);
        }
        Ok(())
    }

    /// 取消尚未发送的操作。
    pub fn cancel(&mut self, id: OperationId, now_ms: u64) -> Result<(), BeginSendError> {
        let Some(active) = self.active.as_ref() else {
            return Err(BeginSendError::StaleOperation);
        };
        if active.report.id != id {
            return Err(BeginSendError::StaleOperation);
        }
        if active.report.state != OperationState::Prepared {
            return Err(BeginSendError::NotPrepared(active.report.state));
        }
        if let Some(previous_ms) = self.clock_rollback(now_ms) {
            return Err(BeginSendError::ClockRollback {
                previous_ms,
                now_ms,
            });
        }
        self.commit_clock(now_ms);
        self.finish_active(OperationState::Cancelled, now_ms, None);
        Ok(())
    }

    /// 返回活动操作或最近完成操作的快照。
    pub fn report(&self, id: OperationId) -> Result<OperationReport, ReportError> {
        if let Some(active) = self.active.as_ref().filter(|active| active.report.id == id) {
            return Ok(active.report);
        }
        if let Some(report) = self.completed.filter(|report| report.id == id) {
            return Ok(report);
        }
        Err(ReportError::StaleOperation)
    }

    /// 取走最近完成操作的报告。
    ///
    /// `Unknown` 保持占用唯一槽位，直到调用方以 [`Driver::acknowledge_unknown`] 确认底层
    /// 不会再发送此帧。该确认不取消设备端可能已经执行的命令。
    pub fn take_report(&mut self, id: OperationId) -> Result<OperationReport, ReportError> {
        if let Some(active) = self.active.as_ref().filter(|active| active.report.id == id) {
            return match active.report.state {
                OperationState::Unknown => Err(ReportError::UnknownPending),
                _ => Err(ReportError::Pending),
            };
        }
        if self.completed.is_some_and(|report| report.id == id) {
            return Ok(self
                .completed
                .take()
                .expect("completed report checked above"));
        }
        Err(ReportError::StaleOperation)
    }

    /// 确认未知发送已经不可能再由底层提交，并释放唯一槽位。
    ///
    /// 调用方必须先确认其 CAN 控制器、future 或队列没有此帧待发，也不会在稍后发送。
    /// 此方法仅释放本地追踪，绝不声明设备命令已取消或未执行。
    pub fn acknowledge_unknown(&mut self, id: OperationId) -> Result<OperationReport, ReportError> {
        let Some(active) = self.active.as_ref() else {
            return Err(ReportError::StaleOperation);
        };
        if active.report.id != id {
            return Err(ReportError::StaleOperation);
        }
        if active.report.state != OperationState::Unknown {
            return Err(ReportError::Pending);
        }
        let report = self
            .active
            .take()
            .expect("active operation checked above")
            .report;
        self.completed = Some(report);
        Ok(report)
    }

    fn prepare(
        &mut self,
        kind: OperationKind,
        now_ms: u64,
        deadline_ms: u64,
    ) -> Result<OperationId, PrepareError> {
        if self.active.is_some() || self.completed.is_some() {
            return Err(PrepareError::Busy);
        }
        if let Some(previous_ms) = self.clock_rollback(now_ms) {
            return Err(PrepareError::ClockRollback {
                previous_ms,
                now_ms,
            });
        }
        if now_ms >= deadline_ms {
            return Err(PrepareError::DeadlineElapsed {
                now_ms,
                deadline_ms,
            });
        }
        let message = match kind {
            OperationKind::Command(command) => Message::Command(command),
            OperationKind::Query(query) => Message::Request(query),
        };
        let frame = protocol::encode(self.node, message).map_err(PrepareError::Encode)?;
        let id = OperationId(self.next_id);
        self.next_id = self
            .next_id
            .checked_add(1)
            .ok_or(PrepareError::IdExhausted)?;
        self.commit_clock(now_ms);
        self.active = Some(ActiveOperation {
            report: OperationReport {
                id,
                kind,
                deadline_ms,
                prepared_at_ms: now_ms,
                dispatching_at_ms: None,
                submitted_at_ms: None,
                terminal_at_ms: None,
                state: OperationState::Prepared,
                response: None,
            },
            frame,
        });
        Ok(id)
    }

    fn observe_query(&mut self, response: Response, received_at_ms: u64) {
        let Some(active) = self.active.as_ref() else {
            return;
        };
        let is_match = matches!(active.report.kind, OperationKind::Query(query) if response_matches(query, response))
            && active.report.state == OperationState::Submitted
            && active
                .report
                .submitted_at_ms
                .is_some_and(|submitted_at_ms| received_at_ms >= submitted_at_ms)
            && received_at_ms < active.report.deadline_ms;
        if is_match {
            self.finish_active(OperationState::Observed, received_at_ms, Some(response));
        }
    }

    fn finish_active(
        &mut self,
        state: OperationState,
        terminal_at_ms: u64,
        response: Option<Response>,
    ) {
        let mut active = self.active.take().expect("active operation must exist");
        active.report.state = state;
        active.report.terminal_at_ms = Some(terminal_at_ms);
        active.report.response = response;
        self.completed = Some(active.report);
    }

    fn mark_unknown(&mut self, id: OperationId) {
        let terminal_at_ms = self
            .active
            .as_ref()
            .and_then(|active| active.report.dispatching_at_ms)
            .unwrap_or(0);
        self.mark_unknown_at(id, terminal_at_ms);
    }

    fn mark_unknown_at(&mut self, id: OperationId, terminal_at_ms: u64) {
        if let Some(active) = self.active.as_mut().filter(|active| active.report.id == id) {
            active.report.state = OperationState::Unknown;
            active.report.terminal_at_ms = Some(terminal_at_ms);
        }
    }

    fn record_submitted(&mut self, id: OperationId, submitted_at_ms: u64) {
        if let Some(active) = self.active.as_mut().filter(|active| active.report.id == id) {
            active.report.submitted_at_ms = Some(submitted_at_ms);
        }
    }

    fn clock_rollback(&self, now_ms: u64) -> Option<u64> {
        self.last_now_ms.filter(|previous_ms| now_ms < *previous_ms)
    }

    fn commit_clock(&mut self, now_ms: u64) {
        self.last_now_ms = Some(now_ms);
    }
}

/// 独占一个已经进入 `Dispatching` 的发送尝试。
///
/// 后端应先持有此 guard，再调用控制器发送；已知结果以对应方法结束 guard。对
/// `embedded-can` 的 `Ok(Some(displaced))`，后端必须先对本 attempt 调用
/// [`SendAttempt::submitted`]，再把 `displaced` 原样作为结果完整返回。后端不得在这两个
/// 步骤之间调用可能 panic 的用户 handler，否则已被控制器接受的新帧会错误地遗失为
/// `Unknown`。无法完整交还 `displaced` 时不得提交此 attempt；让 guard 析构会保守地生成
/// `Unknown`。
pub struct SendAttempt<'a> {
    driver: &'a mut Driver,
    id: OperationId,
    settled: bool,
}

impl SendAttempt<'_> {
    /// 返回可交给经典 CAN 后端的协议编码帧。
    pub fn frame(&self) -> &EncodedFrame {
        &self
            .driver
            .active
            .as_ref()
            .expect("send attempt owns active operation")
            .frame
    }

    /// 记录后端已在本地接受该帧。
    ///
    /// 写命令立刻完成为 `Submitted`；查询保持活动状态，等待同类型回复或到期。
    pub fn submitted(mut self, now_ms: u64) -> Result<(), AttemptError> {
        if self.driver.clock_rollback(now_ms).is_some() {
            self.driver.record_submitted(self.id, now_ms);
            self.driver.mark_unknown_at(self.id, now_ms);
            self.settled = true;
            return Err(AttemptError::ClockRollback);
        }
        let deadline_ms = self.active_deadline();
        if now_ms >= deadline_ms {
            self.driver.commit_clock(now_ms);
            self.driver.record_submitted(self.id, now_ms);
            self.driver.mark_unknown_at(self.id, now_ms);
            self.settled = true;
            return Err(AttemptError::DeadlineElapsed);
        }
        self.driver.commit_clock(now_ms);
        let command = {
            let active = self
                .driver
                .active
                .as_mut()
                .expect("send attempt owns active operation");
            active.report.state = OperationState::Submitted;
            active.report.submitted_at_ms = Some(now_ms);
            matches!(active.report.kind, OperationKind::Command(_))
        };
        if command {
            self.driver
                .finish_active(OperationState::Submitted, now_ms, None);
        }
        self.settled = true;
        Ok(())
    }

    /// 记录收发后端明确未发送且暂时会阻塞。
    ///
    /// 期限前操作返回 `Prepared`，调用方可再次调用 [`Driver::begin_send`]。若 `now_ms`
    /// 回退，该明确未发送事实仍保留为 `Prepared`，但返回时钟诊断且不会回退内部时钟。
    pub fn would_block(mut self, now_ms: u64) -> Result<(), AttemptError> {
        self.not_sent_inner(now_ms)
    }

    /// 记录收发后端明确未发送。
    ///
    /// 该次发送尝试终态为 `Failed`；调用方只可在明确的
    /// [`SendAttempt::would_block`] 后重新开始一次发送尝试。若 `now_ms` 回退，driver 仍
    /// 保留明确未发送事实并以最后已知单调时刻记录终态，但返回时钟诊断而不会回退内部时钟。
    pub fn not_sent(mut self, now_ms: u64) -> Result<(), AttemptError> {
        if self.driver.clock_rollback(now_ms).is_some() {
            let terminal_at_ms = self
                .driver
                .last_now_ms
                .expect("begin_send committed the clock");
            self.driver
                .finish_active(OperationState::Failed, terminal_at_ms, None);
            self.settled = true;
            return Err(AttemptError::ClockRollback);
        }
        if now_ms >= self.active_deadline() {
            self.driver.commit_clock(now_ms);
            self.driver
                .finish_active(OperationState::Failed, now_ms, None);
            self.settled = true;
            return Err(AttemptError::DeadlineElapsed);
        }
        self.driver.commit_clock(now_ms);
        self.driver
            .finish_active(OperationState::Failed, now_ms, None);
        self.settled = true;
        Ok(())
    }

    fn not_sent_inner(&mut self, now_ms: u64) -> Result<(), AttemptError> {
        if self.driver.clock_rollback(now_ms).is_some() {
            let active = self
                .driver
                .active
                .as_mut()
                .expect("send attempt owns active operation");
            active.report.state = OperationState::Prepared;
            self.settled = true;
            return Err(AttemptError::ClockRollback);
        }
        if now_ms >= self.active_deadline() {
            self.driver.commit_clock(now_ms);
            self.driver
                .finish_active(OperationState::TimedOut, now_ms, None);
            self.settled = true;
            return Err(AttemptError::DeadlineElapsed);
        }
        self.driver.commit_clock(now_ms);
        let active = self
            .driver
            .active
            .as_mut()
            .expect("send attempt owns active operation");
        active.report.state = OperationState::Prepared;
        self.settled = true;
        Ok(())
    }

    fn active_deadline(&self) -> u64 {
        self.driver
            .active
            .as_ref()
            .expect("send attempt owns active operation")
            .report
            .deadline_ms
    }
}

impl Drop for SendAttempt<'_> {
    fn drop(&mut self) {
        if !self.settled {
            self.driver.mark_unknown(self.id);
        }
    }
}

fn response_matches(query: Query, response: Response) -> bool {
    matches!(
        (query, response),
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
