/* Copyright The odrive-can-driver Contributors */

/**
 * @file
 * @brief 独立于 CAN 收发接口的单节点 CANSimple 操作状态机。
 */

#ifndef ODRIVE_CAN_DRIVER_DRIVER_H
#define ODRIVE_CAN_DRIVER_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#include <odrive_can_driver/fw_v0_5_1.h>
#include <odrive_can_driver/response_cache.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 状态机接口的返回结果。部分错误仍会将发送证据写入报告。 */
typedef enum {
    ODRIVE_CAN_DRIVER_OK, /**< 所请求的调用成功。 */
    ODRIVE_CAN_DRIVER_INVALID_ARGUMENT, /**< 必需指针为空或接口参数无效。 */
    ODRIVE_CAN_DRIVER_BUSY, /**< 操作仍在进行，或其报告尚未取走，槽位被占用。 */
    ODRIVE_CAN_DRIVER_DEADLINE_ELAPSED, /**< 准备或开始发送尝试时，已到达或超过期限。 */
    ODRIVE_CAN_DRIVER_CLOCK_ROLLBACK, /**< 本次处理时刻早于上一次有效的处理时刻。 */
    ODRIVE_CAN_DRIVER_ENCODE_ERROR, /**< 协议编码器拒绝输入的消息。 */
    ODRIVE_CAN_DRIVER_ID_EXHAUSTED, /**< 已耗尽操作序号，继续递增会回绕。 */
    ODRIVE_CAN_DRIVER_STALE_PERMIT, /**< 凭证来自其他实例、已失效或已无法签发下一次尝试。 */
    ODRIVE_CAN_DRIVER_NOT_PREPARED, /**< 当前操作已离开准备状态。 */
    ODRIVE_CAN_DRIVER_STALE_ATTEMPT, /**< 尝试来自其他实例、已过期或其回填资格已被消费。 */
    ODRIVE_CAN_DRIVER_REVOKED, /**< 后续底层发送资格已撤销，或 Unknown 确认后回填资格已撤销。 */
    ODRIVE_CAN_DRIVER_EVENT_AFTER_DEADLINE, /**< 已知的本地提交时刻到达或超过期限。 */
    ODRIVE_CAN_DRIVER_INVALID_EVENT_TIME, /**< 发送事件时刻早于开始尝试或晚于本次处理。 */
    ODRIVE_CAN_DRIVER_STALE_OPERATION, /**< 操作或候选回复已非当前可用项。 */
    ODRIVE_CAN_DRIVER_PENDING, /**< 当前状态无法取走报告或确认未知结果。 */
    ODRIVE_CAN_DRIVER_UNKNOWN_PENDING, /**< 发送尝试未回填，或未知结果尚未确认处置。 */
    ODRIVE_CAN_DRIVER_FUTURE_RESPONSE, /**< 候选回复的接收时刻晚于接纳、忽略或拒绝的处理时刻。 */
    ODRIVE_CAN_DRIVER_NO_RESPONSE, /**< 当前操作没有符合条件的候选回复。 */
    ODRIVE_CAN_DRIVER_UNRELATED, /**< 帧属于其他节点，或命令编号在该协议版本中未知或保留。 */
    ODRIVE_CAN_DRIVER_DECODE_ERROR, /**< 协议解码器拒绝了接收帧。 */
} odrive_can_driver_result_t;

/** 在对应实例的生命周期内唯一的操作标识。 */
typedef struct {
    uint64_t incarnation; /**< 初始化时传入的非零实例编号。 */
    uint64_t sequence; /**< 实例内的操作序号。 */
} odrive_can_driver_operation_id_t;

/** 开始一次发送尝试的单次凭证。调用方应将其视为只读。 */
typedef struct {
    odrive_can_driver_operation_id_t id; /**< 用于查询、取消操作或取走报告的标识。 */
    uint64_t sequence; /**< 由 Driver 签发的凭证序号。 */
} odrive_can_driver_send_permit_t;

/** 底层发送尝试的标识。一次结果回填会消费其回填资格。 */
typedef struct {
    odrive_can_driver_operation_id_t id; /**< 所属操作的标识。 */
    uint64_t sequence; /**< 由 Driver 签发的尝试序号。 */
} odrive_can_driver_tx_attempt_t;

/** 候选回复的标识。保持原值并传给接纳、忽略或拒绝接口。 */
typedef struct {
    odrive_can_driver_operation_id_t id; /**< 所属查询的标识。 */
    uint64_t sequence; /**< 由 Driver 签发的候选序号。 */
} odrive_can_driver_response_candidate_t;

/** 本地操作的类型。 */
typedef enum {
    ODRIVE_CAN_DRIVER_OPERATION_COMMAND, /**< 主机发送的写命令。 */
    ODRIVE_CAN_DRIVER_OPERATION_QUERY, /**< RTR 查询请求。 */
} odrive_can_driver_operation_kind_t;

/** 唯一操作槽的状态。 */
typedef enum {
    ODRIVE_CAN_DRIVER_STATE_PREPARED, /**< 帧已编码，当前没有待回填的发送尝试。 */
    ODRIVE_CAN_DRIVER_STATE_DISPATCHING, /**< 底层发送尝试待回填。 */
    ODRIVE_CAN_DRIVER_STATE_SUBMITTED, /**< 本地底层已接受帧。写命令达到终态，查询继续等待反馈。 */
    ODRIVE_CAN_DRIVER_STATE_OBSERVED, /**< 调用方已接纳查询的候选回复。 */
    ODRIVE_CAN_DRIVER_STATE_REJECTED, /**< 调用方已拒绝候选回复并结束查询。 */
    ODRIVE_CAN_DRIVER_STATE_FAILED, /**< 未能开始发送尝试，或底层证明发送失败且未提交。 */
    ODRIVE_CAN_DRIVER_STATE_TIMED_OUT, /**< 已到达或超过期限。查阅提交时间以确认本地是否已接受帧。 */
    ODRIVE_CAN_DRIVER_STATE_CANCELLED, /**< 未发送的操作已取消，或已提交查询的等待已取消。 */
    ODRIVE_CAN_DRIVER_STATE_UNKNOWN, /**< 发送结果有待确认处置，槽位仍被占用。 */
} odrive_can_driver_operation_state_t;

/** 调用方单调微秒时钟域中的可选时刻。 */
typedef struct {
    bool present; /**< 是否已记录该事件的时刻。 */
    uint64_t us; /**< present 为真时的微秒时刻。 */
} odrive_can_driver_optional_time_t;

/** 操作报告快照。提交字段记录本地底层接受帧的事件，设备侧状态需另取反馈。 */
typedef struct {
    odrive_can_driver_operation_id_t id; /**< 操作标识。 */
    odrive_can_driver_operation_kind_t kind; /**< 指定应读取哪个联合体成员。 */
    union {
        odrive_can_command_t command; /**< 原始写命令及参数。 */
        odrive_can_query_t query; /**< 原始查询类型。 */
    } value; /**< 操作内容。 */
    uint64_t deadline; /**< 微秒绝对期限，合法时间早于该值。 */
    uint64_t prepared_at; /**< 操作准备时刻，单位为微秒。 */
    odrive_can_driver_optional_time_t dispatching_at; /**< 最近一次开始发送尝试的处理时刻。 */
    odrive_can_driver_optional_time_t submitted_at; /**< 本地底层接受帧的事件时刻。 */
    odrive_can_driver_optional_time_t tx_event_at; /**< 最近回填的发送结果时刻，含已证实未入队的结果。 */
    odrive_can_driver_optional_time_t cancel_requested_at; /**< 已记录的取消请求时刻。 */
    odrive_can_driver_optional_time_t terminal_at; /**< 终态事件时刻。进入 Unknown 时记录处理时刻。 */
    uint64_t processed_at; /**< 最近一次更新本报告的处理时刻，单位为微秒。 */
    odrive_can_driver_operation_state_t state; /**< 当前本地操作状态。 */
    bool response_present; /**< 是否已有候选回复被接纳或明确拒绝。 */
    odrive_can_cached_response_t response; /**< 标记存在时，被接纳或拒绝的候选回复。 */
} odrive_can_driver_operation_report_t;

/** 已结束的底层发送尝试给出的证据。 */
typedef enum {
    ODRIVE_CAN_DRIVER_TX_SUBMITTED, /**< 本地底层已接受帧，设备是否执行需另取反馈。 */
    ODRIVE_CAN_DRIVER_TX_WOULD_BLOCK, /**< 本次尝试已结束，证据表明没有入队且之后也不会入队。 */
    ODRIVE_CAN_DRIVER_TX_NOT_SUBMITTED, /**< 本次尝试已失败，证据表明没有入队且之后也不会入队。 */
} odrive_can_driver_tx_outcome_t;

/** 成功回填发送结果后的下一步。 */
typedef enum {
    ODRIVE_CAN_DRIVER_COMPLETION_SUBMITTED, /**< 已记录本地提交证据。 */
    ODRIVE_CAN_DRIVER_COMPLETION_RETRY, /**< 已证实未入队，签发了新的单次发送凭证。 */
    ODRIVE_CAN_DRIVER_COMPLETION_FAILED, /**< 没有重试凭证，具体终态请查阅报告。 */
} odrive_can_driver_tx_completion_kind_t;

/** 成功回填的输出。 */
typedef struct {
    odrive_can_driver_tx_completion_kind_t kind; /**< 指定下一步操作。 */
    odrive_can_driver_send_permit_t retry; /**< kind 为 RETRY 时签发的新凭证。 */
} odrive_can_driver_tx_completion_t;

/**
 * @brief 使用调用方固定存储的单节点、单操作槽 Driver。
 *
 * 应先初始化，之后保持存储地址稳定且不可复制实例。调用方负责串行化所有访问，
 * 包括授权和紧随其后的底层发送调用。接收、实际收发操作、外设配置和反馈来源的连续性
 * 均由调用方管理。
 *
 * 所有时刻使用同一个单调微秒时钟域。事件时刻可早于处理时刻。
 * 必需的对象和输出指针必须有效。除可选的编解码诊断外，输出只在成功时写入。
 * 某些调用返回时钟错误时仍会更新内部报告，具体见相应接口。
 */
typedef struct {
    /** @cond INTERNAL */
    uint64_t _incarnation;
    uint64_t _next_operation;
    uint32_t _node;
    odrive_can_driver_optional_time_t _last;
    odrive_can_response_cache_t _cache;
    bool _active;
    odrive_can_driver_operation_report_t _report;
    odrive_can_frame_t _frame;
    uint64_t _attempt_sequence;
    bool _attempt_outstanding;
    bool _unknown_acknowledged;
    uint64_t _candidate_sequence;
    bool _candidate_present;
    odrive_can_cached_response_t _candidate;
    /** @endcond */
} odrive_can_driver_t;

/**
 * @brief 初始化指定节点的 Driver 存储。
 *
 * 所有仍可能收到旧凭证、尝试或操作标识的实例生命周期必须使用不同的实例编号。
 * 重新初始化前，调用方应结束旧实例的发送操作、处置可能继续发帧的控制器队列，
 * 并取走旧操作报告。
 *
 * @param[out] driver 由调用方提供的 Driver 存储。
 * @param[in] node CANSimple 节点号，范围为 0 到 63。
 * @param[in] incarnation 由调用方分配的非零实例编号。重新初始化时分配新值。
 * @return 成功返回 OK。参数无效时返回 INVALID_ARGUMENT，存储保持原值。
 */
odrive_can_driver_result_t odrive_can_driver_init(
    odrive_can_driver_t *driver, uint32_t node, uint64_t incarnation);

/**
 * @brief 读取配置的节点号。
 * @param[in] driver 已初始化的 Driver，或 NULL。
 * @return 节点号。driver 为 NULL 时返回 UINT32_MAX。
 */
uint32_t odrive_can_driver_node(const odrive_can_driver_t *driver);

/**
 * @brief 借用 Driver 内的只读缓存。读取时须与 Driver 调用串行。
 * @param[in] driver 已初始化的 Driver，或 NULL。
 * @return 当前实例存续期间有效的缓存指针。driver 为 NULL 时返回 NULL。
 */
const odrive_can_response_cache_t *odrive_can_driver_cache(const odrive_can_driver_t *driver);

/**
 * @brief 编码写命令并准备操作，当前调用不执行底层发送。
 *
 * 实际发送到目标节点的命令可能会给受支持固件的看门狗喂狗。
 *
 * @param[in,out] driver 已初始化的 Driver。
 * @param[in] command 写命令及参数。
 * @param[in] now 本次准备的微秒时刻。
 * @param[in] deadline 微秒绝对期限，须严格晚于 now。
 * @param[out] permit 成功时签发的发送凭证。
 * @param[out] codec_error 可选的协议诊断，只在 ENCODE_ERROR 时写入。允许为 NULL，
 *                        非空时须与所有输入和其他输出独立。
 * @return 成功返回 OK。槽位被占用时返回 BUSY，处理时间回退时返回 CLOCK_ROLLBACK，
 *         到达或超过期限时返回 DEADLINE_ELAPSED，协议拒绝命令时返回 ENCODE_ERROR，
 *         操作序号耗尽时返回 ID_EXHAUSTED，必需指针为空时返回 INVALID_ARGUMENT。
 */
odrive_can_driver_result_t odrive_can_driver_prepare_command(
    odrive_can_driver_t *driver, const odrive_can_command_t *command, uint64_t now,
    uint64_t deadline, odrive_can_driver_send_permit_t *permit, odrive_can_codec_error_t *codec_error);

/**
 * @brief 编码 RTR 查询并准备操作，当前调用不执行底层发送。
 *
 * 实际发送到目标节点的 RTR 查询也可能会给受支持固件的看门狗喂狗。
 *
 * @param[in,out] driver 已初始化的 Driver。
 * @param[in] query 查询类型。
 * @param[in] now 本次准备的微秒时刻。
 * @param[in] deadline 微秒绝对期限，须严格晚于 now。
 * @param[out] permit 成功时签发的发送凭证。
 * @param[out] codec_error 可选的协议诊断，只在 ENCODE_ERROR 时写入。允许为 NULL，
 *                        非空时须与所有输入和其他输出独立。
 * @return 成功返回 OK。槽位被占用时返回 BUSY，处理时间回退时返回 CLOCK_ROLLBACK，
 *         到达或超过期限时返回 DEADLINE_ELAPSED，协议拒绝查询类型时返回 ENCODE_ERROR，
 *         操作序号耗尽时返回 ID_EXHAUSTED，必需指针为空时返回 INVALID_ARGUMENT。
 */
odrive_can_driver_result_t odrive_can_driver_prepare_query(
    odrive_can_driver_t *driver, odrive_can_query_t query, uint64_t now,
    uint64_t deadline, odrive_can_driver_send_permit_t *permit, odrive_can_codec_error_t *codec_error);

/**
 * @brief 消费发送凭证并开始一次尝试，当前调用不执行底层发送。
 *
 * 已匹配的准备操作遇到时钟回退或序号耗尽时进入 Failed，已经到期时进入 TimedOut。
 * 三种情况均未开始底层发送，可取走终态报告。
 *
 * @param[in,out] driver 已初始化的 Driver。
 * @param[in] permit 当前发送凭证。
 * @param[in] at 本次开始尝试的微秒处理时刻。
 * @param[out] attempt 成功时签发的发送尝试标识。
 * @return 成功返回 OK。必需指针为空时返回 INVALID_ARGUMENT，凭证不匹配或尝试序号
 *         耗尽时返回 STALE_PERMIT，操作已离开准备状态时返回 NOT_PREPARED，
 *         处理时钟回退时返回 CLOCK_ROLLBACK，已到期时返回 DEADLINE_ELAPSED。
 */
odrive_can_driver_result_t odrive_can_driver_begin_send(
    odrive_can_driver_t *driver, odrive_can_driver_send_permit_t permit, uint64_t at,
    odrive_can_driver_tx_attempt_t *attempt);

/**
 * @brief 为紧随其后的一次底层发送调用授予资格并输出帧。
 *
 * 每次调用底层发送前均须重新授权。授权和发送之间不可挂起或让其他调用推进本实例。
 * 返回错误后停止进一步的底层发送调用，由调用方处置仍在进行的发送工作和控制器队列。
 * 当前尝试首次在本入口遇到时钟回退或期限，只撤销后续发送资格并将操作置为 Unknown，
 * 仍可回填一次发送结果。
 *
 * @param[in,out] driver 已初始化的 Driver。
 * @param[in] attempt 本次申请授权的尝试标识。
 * @param[in] at 本次授权的微秒处理时刻。
 * @param[out] frame 成功时写入的帧，只用于紧随其后的一次底层发送调用。
 * @return 成功返回 OK。参数无效时返回 INVALID_ARGUMENT，标识不匹配或已回填时返回
 *         STALE_ATTEMPT，时钟回退时返回 CLOCK_ROLLBACK，发送资格已撤销时返回 REVOKED。
 */
odrive_can_driver_result_t odrive_can_driver_authorize_tx(
    odrive_can_driver_t *driver, odrive_can_driver_tx_attempt_t attempt, uint64_t at,
    odrive_can_frame_t *frame);

/**
 * @brief 在对应的底层尝试结束后回填已证实的发送结果。
 *
 * WouldBlock 和 NotSubmitted 均需底层证明本次未入队且之后也不会入队。
 * 当前仍可回填的尝试在本调用中只消费一次。即使时钟回退或事件时刻不合法，
 * 报告仍保留本次发送事件与已证实的本地提交。提交事件到期、时钟回退或时间矛盾时，
 * 已提交的结果记为 Unknown。只有成功回填后返回 RETRY 的情况才有新凭证。
 *
 * @param[in,out] driver 已初始化的 Driver。
 * @param[in] attempt 底层发送已经结束的尝试标识。
 * @param[in] outcome 该尝试的已证实结果。
 * @param[in] event_at 底层发送结果的微秒事件时刻。
 * @param[in] processed_at 处理本次回填的微秒时刻。
 * @param[out] completion 成功时给出的下一步，发生错误时保持原值。
 * @return 成功返回 OK。参数无效时返回 INVALID_ARGUMENT，尝试标识不匹配或已回填
 *         时返回 STALE_ATTEMPT，显式确认 Unknown 后返回 REVOKED。时间错误分别为
 *         CLOCK_ROLLBACK、INVALID_EVENT_TIME 或 EVENT_AFTER_DEADLINE。
 */
odrive_can_driver_result_t odrive_can_driver_finish_tx(
    odrive_can_driver_t *driver, odrive_can_driver_tx_attempt_t attempt,
    odrive_can_driver_tx_outcome_t outcome, uint64_t event_at, uint64_t processed_at,
    odrive_can_driver_tx_completion_t *completion);

/**
 * @brief 底层证明入队前已取消时，回填并消费该尝试。
 *
 * 调用方须证明原发送工作已结束且该帧未入队，普通取消、标识遗失和到期均无法提供此证据。
 * 有取消请求时记录 Cancelled。没有取消请求且处理时已到期时记录 TimedOut，
 * 其余情形记录 Cancelled。时钟或事件时刻错误仍消费匹配的回填资格并写入报告。
 *
 * @param[in,out] driver 已初始化的 Driver。
 * @param[in] attempt 已证实未提交的尝试标识。
 * @param[in] event_at 底层结果的微秒事件时刻。
 * @param[in] processed_at 处理本次回填的微秒时刻。
 * @return 成功返回 OK。driver 为空时返回 INVALID_ARGUMENT，尝试标识不匹配或已回填
 *         时返回 STALE_ATTEMPT，显式确认 Unknown 后返回 REVOKED，处理时钟回退时
 *         返回 CLOCK_ROLLBACK，事件时刻越界时返回 INVALID_EVENT_TIME。
 */
odrive_can_driver_result_t odrive_can_driver_cancel_unsubmitted(
    odrive_can_driver_t *driver, odrive_can_driver_tx_attempt_t attempt, uint64_t event_at,
    uint64_t processed_at);

/**
 * @brief 底层发送结束但提交结果不明时，消费尝试并记录 Unknown。
 * @param[in,out] driver 已初始化的 Driver。
 * @param[in] attempt 底层发送已结束但结果无法判定的尝试标识。
 * @param[in] at 本次处理的微秒时刻。
 * @return 成功返回 OK。driver 为空时返回 INVALID_ARGUMENT，尝试标识不匹配或已回填
 *         时返回 STALE_ATTEMPT，显式确认 Unknown 后返回 REVOKED。CLOCK_ROLLBACK
 *         仍消费匹配的尝试，并用上一次有效处理时刻记录 Unknown。
 */
odrive_can_driver_result_t odrive_can_driver_abandon_attempt(
    odrive_can_driver_t *driver, odrive_can_driver_tx_attempt_t attempt, uint64_t at);

/**
 * @brief 撤销后续发送资格，或取消已提交查询的等待。
 *
 * 仍待回填的发送尝试进入 Unknown，保留回填资格和已知的提交证据。
 * 调用方继续负责处置控制器队列和设备侧状态。对已暂定 TimedOut 的查询调用本函数
 * 仍可成功，操作状态及报告保持原值。需要关闭迟到反馈的接纳窗口时，应取走报告。
 *
 * @param[in,out] driver 已初始化的 Driver。
 * @param[in] id 待取消操作的标识。
 * @param[in] at 本次请求的微秒处理时刻。
 * @return 操作为当前项且时钟有效时返回 OK。driver 为空时返回 INVALID_ARGUMENT，
 *         操作标识不匹配时返回 STALE_OPERATION，处理时钟回退时返回 CLOCK_ROLLBACK。
 */
odrive_can_driver_result_t odrive_can_driver_cancel(
    odrive_can_driver_t *driver, odrive_can_driver_operation_id_t id, uint64_t at);

/**
 * @brief 推进处理时钟并检查期限。
 *
 * 到期时，待准备发送的操作或已提交查询进入 TimedOut，待回填的底层尝试进入 Unknown。
 * 已提交查询的超时在取走报告前为暂定状态，仍可处理符合接收时间窗口的迟到回复。
 *
 * @param[in,out] driver 已初始化的 Driver。
 * @param[in] at 本次推进的微秒处理时刻。
 * @return 成功返回 OK。driver 为空时返回 INVALID_ARGUMENT，处理时钟回退时
 *         返回 CLOCK_ROLLBACK。
 */
odrive_can_driver_result_t odrive_can_driver_tick(odrive_can_driver_t *driver, uint64_t at);

/**
 * @brief 解码接收帧，更新原始回复缓存，并记录查询的候选回复。
 *
 * 调用方持有原始帧，并负责把接收事件映射到共用时钟。接收时刻不会推进处理时钟。
 * 成功解码的回复仍需由调用方接纳，才能完成查询。
 *
 * @param[in,out] driver 已初始化的 Driver。
 * @param[in] frame 帧视图。解码器会读取匹配的非空经典 CAN 数据帧有效载荷。
 * @param[in] received_at 微秒接收事件时刻。
 * @param[out] message 成功时解码出的消息。
 * @param[out] codec_error 可选的协议诊断。允许为 NULL，非空时须与所有输入和其他
 *                        输出独立。参数检查通过后写入解码器给出的类别及详情。
 * @return 成功返回 OK。无关帧返回 UNRELATED，解码失败返回 DECODE_ERROR，
 *         必需指针为空时返回 INVALID_ARGUMENT。可通过协议诊断区分具体解码错误。
 */
odrive_can_driver_result_t odrive_can_driver_ingest(
    odrive_can_driver_t *driver, const odrive_can_frame_view_t *frame, uint64_t received_at,
    odrive_can_message_t *message, odrive_can_codec_error_t *codec_error);

/**
 * @brief 读取已知提交后的当前候选回复标识和快照。
 *
 * 仅当回复的接收事件严格晚于提交且严格早于期限时才可成为可接纳的候选。
 * 在超时报告取走前，处于该时间窗口的迟到回复仍可成为候选。
 *
 * @param[in] driver 已初始化的 Driver。
 * @param[in] id 操作标识。
 * @param[out] candidate 成功时输出当前候选回复标识。
 * @param[out] response 成功时输出候选回复快照。
 * @return 成功返回 OK。没有符合条件的候选回复时返回 NO_RESPONSE，操作标识
 *         不匹配时返回 STALE_OPERATION，必需指针为空时返回 INVALID_ARGUMENT。
 */
odrive_can_driver_result_t odrive_can_driver_pending_response(
    const odrive_can_driver_t *driver, odrive_can_driver_operation_id_t id,
    odrive_can_driver_response_candidate_t *candidate, odrive_can_cached_response_t *response);

/**
 * @brief 接纳符合条件的当前候选回复，并记录 Observed。
 *
 * Observed 表示调用方认可此查询和回复的关联，CANSimple 回复仍没有用于证明因果关系的标识。
 *
 * @param[in,out] driver 已初始化的 Driver。
 * @param[in] candidate 当前候选回复的标识。
 * @param[in] at 本次接纳的微秒处理时刻。
 * @return 成功返回 OK。driver 为空时返回 INVALID_ARGUMENT，候选不匹配或已失去
 *         接纳资格时返回 STALE_OPERATION。接收事件晚于 at 时返回 FUTURE_RESPONSE，
 *         处理时钟回退时返回 CLOCK_ROLLBACK。
 */
odrive_can_driver_result_t odrive_can_driver_accept_response(
    odrive_can_driver_t *driver, odrive_can_driver_response_candidate_t candidate, uint64_t at);

/**
 * @brief 丢弃当前候选回复，保留原始缓存及提交证据，不结束查询。
 *
 * @param[in,out] driver 已初始化的 Driver。
 * @param[in] candidate 当前候选回复的标识。
 * @param[in] at 本次忽略的微秒处理时刻。
 * @return 成功返回 OK。driver 为空时返回 INVALID_ARGUMENT，候选不匹配或已失去
 *         接纳资格时返回 STALE_OPERATION。接收事件晚于 at 时返回 FUTURE_RESPONSE，
 *         处理时钟回退时返回 CLOCK_ROLLBACK。
 */
odrive_can_driver_result_t odrive_can_driver_ignore_response(
    odrive_can_driver_t *driver, odrive_can_driver_response_candidate_t candidate, uint64_t at);

/**
 * @brief 拒绝当前候选回复并结束查询，保留被拒绝的回复和提交证据。
 *
 * @param[in,out] driver 已初始化的 Driver。
 * @param[in] candidate 当前候选回复的标识。
 * @param[in] at 本次拒绝的微秒处理时刻。
 * @return 成功返回 OK。driver 为空时返回 INVALID_ARGUMENT，候选不匹配或已失去
 *         接纳资格时返回 STALE_OPERATION。接收事件晚于 at 时返回 FUTURE_RESPONSE，
 *         处理时钟回退时返回 CLOCK_ROLLBACK。
 */
odrive_can_driver_result_t odrive_can_driver_reject_response(
    odrive_can_driver_t *driver, odrive_can_driver_response_candidate_t candidate, uint64_t at);

/**
 * @brief 读取当前报告的快照，槽位继续被占用。
 * @param[in] driver 已初始化的 Driver。
 * @param[in] id 操作标识。
 * @param[out] report 成功时给出的报告快照。
 * @return 成功返回 OK。操作标识不匹配时返回 STALE_OPERATION，必需指针为空时
 *         返回 INVALID_ARGUMENT。
 */
odrive_can_driver_result_t odrive_can_driver_report(
    const odrive_can_driver_t *driver, odrive_can_driver_operation_id_t id,
    odrive_can_driver_operation_report_t *report);

/**
 * @brief 取走终态报告并释放槽位。取走暂定超时报告会关闭迟到反馈的接纳窗口。
 * @param[in,out] driver 已初始化的 Driver。
 * @param[in] id 操作标识。
 * @param[out] report 成功时给出的终态报告。
 * @return 成功返回 OK。Prepared 或已提交查询仍在等待时返回 PENDING。
 *         Dispatching 或尚未确认处置的 Unknown 返回 UNKNOWN_PENDING。
 *         操作标识不匹配时返回 STALE_OPERATION，必需指针为空时返回 INVALID_ARGUMENT。
 */
odrive_can_driver_result_t odrive_can_driver_take_report(
    odrive_can_driver_t *driver, odrive_can_driver_operation_id_t id,
    odrive_can_driver_operation_report_t *report);

/**
 * @brief 确认 Unknown 已由调用方处置，使其报告可被取走。
 *
 * 调用方须确认原发送工作已结束，且已处置可能继续发帧的控制器队列。成功后关闭之前
 * 仍可用的发送与回填资格，报告保持 Unknown 并继续占用槽位，直至取走。确认处置
 * 不构成未提交证据，也不授予重试资格。
 *
 * @param[in,out] driver 已初始化的 Driver。
 * @param[in] id 已结束全部发送工作的操作标识。
 * @param[out] report 成功时给出的 Unknown 报告快照。
 * @return 成功返回 OK。当前操作的报告尚未进入 Unknown 时返回 PENDING，操作标识
 *         不匹配时返回 STALE_OPERATION，必需指针为空时返回 INVALID_ARGUMENT。
 */
odrive_can_driver_result_t odrive_can_driver_acknowledge_unknown(
    odrive_can_driver_t *driver, odrive_can_driver_operation_id_t id,
    odrive_can_driver_operation_report_t *report);

#ifdef __cplusplus
}
#endif

#endif
