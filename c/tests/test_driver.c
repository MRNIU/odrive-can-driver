/* Copyright The odrive-can-driver Contributors */

/**
 * @file
 * @brief 通过公开 API 验证 C Driver 的操作归属和结果处理。
 */

#include "odrive_can_driver/driver.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/**
 * @brief 检查测试条件，使用 NDEBUG 编译宿主测试时也执行。
 * @param[in] condition 预期成立的布尔表达式。
 */
#define CHECK(condition)                                                         \
    do {                                                                         \
        if (!(condition)) {                                                      \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition);       \
            exit(EXIT_FAILURE);                                                  \
        }                                                                        \
    } while (0)

/** @brief 固定测试帧使用的节点。 */
enum {
    TEST_NODE = 7, /**< 与测试中固定帧 ID 对应的 CANSimple 节点。 */
};

/**
 * @brief 准备写命令，全程无需访问 CAN 传输层。
 * @param[in,out] driver 已初始化且操作槽为空的 Driver。
 * @param[in] deadline_us 绝对截止时刻，单位为微秒，截止时刻本身不在有效范围内。
 * @return 此操作首次签发的发送凭证。
 */
static odrive_can_driver_send_permit_t prepare_clear_errors(odrive_can_driver_t *driver,
                                                           uint64_t deadline_us)
{
    const odrive_can_command_t command = {.kind = ODRIVE_CAN_COMMAND_CLEAR_ERRORS};
    odrive_can_driver_send_permit_t permit;

    CHECK(odrive_can_driver_prepare_command(driver, &command, 1, deadline_us, &permit, NULL) ==
          ODRIVE_CAN_DRIVER_OK);
    return permit;
}

/** @brief 验证并存的 Driver 实例和复用同一存储的实例之间的身份隔离。 */
static void rejects_foreign_and_reconstructed_driver_identities(void)
{
    odrive_can_driver_t first;
    odrive_can_driver_t second;
    odrive_can_driver_tx_attempt_t old_attempt;
    odrive_can_driver_tx_attempt_t other_attempt;
    odrive_can_driver_operation_report_t report;
    odrive_can_frame_t frame;

    CHECK(odrive_can_driver_init(&first, TEST_NODE, 10) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_init(&second, TEST_NODE, 11) == ODRIVE_CAN_DRIVER_OK);
    const odrive_can_driver_send_permit_t old_permit = prepare_clear_errors(&first, 100);
    const odrive_can_driver_send_permit_t other_permit = prepare_clear_errors(&second, 100);

    CHECK(odrive_can_driver_report(&second, old_permit.id, &report) ==
          ODRIVE_CAN_DRIVER_STALE_OPERATION);
    CHECK(odrive_can_driver_begin_send(&second, old_permit, 2, &other_attempt) ==
          ODRIVE_CAN_DRIVER_STALE_PERMIT);
    CHECK(odrive_can_driver_begin_send(&first, old_permit, 2, &old_attempt) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_begin_send(&second, other_permit, 2, &other_attempt) ==
          ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_authorize_tx(&second, old_attempt, 3, &frame) ==
          ODRIVE_CAN_DRIVER_STALE_ATTEMPT);

    /* 复用存储前结束原尝试。此处没有调用底层发送。 */
    CHECK(odrive_can_driver_cancel_unsubmitted(&first, old_attempt, 3, 3) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_take_report(&first, old_permit.id, &report) == ODRIVE_CAN_DRIVER_OK);
    CHECK(report.state == ODRIVE_CAN_DRIVER_STATE_CANCELLED);
    CHECK(odrive_can_driver_init(&first, TEST_NODE, 12) == ODRIVE_CAN_DRIVER_OK);

    const odrive_can_driver_send_permit_t new_permit = prepare_clear_errors(&first, 100);
    odrive_can_driver_tx_attempt_t new_attempt;
    CHECK(odrive_can_driver_begin_send(&first, old_permit, 2, &new_attempt) ==
          ODRIVE_CAN_DRIVER_STALE_PERMIT);
    CHECK(odrive_can_driver_begin_send(&first, new_permit, 2, &new_attempt) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_authorize_tx(&first, old_attempt, 3, &frame) ==
          ODRIVE_CAN_DRIVER_STALE_ATTEMPT);
    CHECK(odrive_can_driver_take_report(&first, old_permit.id, &report) ==
          ODRIVE_CAN_DRIVER_STALE_OPERATION);
    CHECK(odrive_can_driver_authorize_tx(&first, new_attempt, 3, &frame) == ODRIVE_CAN_DRIVER_OK);
}

/** @brief 验证底层返回 WouldBlock 后，只能凭新签发的凭证重试。 */
static void rejects_replay_of_permit_and_attempt_after_would_block(void)
{
    odrive_can_driver_t driver;
    odrive_can_driver_tx_attempt_t attempt;
    odrive_can_driver_tx_attempt_t next_attempt;
    odrive_can_driver_tx_completion_t completion;
    odrive_can_driver_operation_report_t report;
    odrive_can_frame_t frame;

    CHECK(odrive_can_driver_init(&driver, TEST_NODE, 20) == ODRIVE_CAN_DRIVER_OK);
    const odrive_can_driver_send_permit_t first = prepare_clear_errors(&driver, 100);
    CHECK(odrive_can_driver_begin_send(&driver, first, 2, &attempt) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_authorize_tx(&driver, attempt, 3, &frame) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_finish_tx(&driver, attempt, ODRIVE_CAN_DRIVER_TX_WOULD_BLOCK,
                                     4, 5, &completion) == ODRIVE_CAN_DRIVER_OK);
    CHECK(completion.kind == ODRIVE_CAN_DRIVER_COMPLETION_RETRY);
    const odrive_can_driver_send_permit_t retry = completion.retry;

    CHECK(odrive_can_driver_begin_send(&driver, first, 6, &next_attempt) ==
          ODRIVE_CAN_DRIVER_STALE_PERMIT);
    CHECK(odrive_can_driver_begin_send(&driver, retry, 6, &next_attempt) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_authorize_tx(&driver, attempt, 7, &frame) ==
          ODRIVE_CAN_DRIVER_STALE_ATTEMPT);
    CHECK(odrive_can_driver_finish_tx(&driver, attempt, ODRIVE_CAN_DRIVER_TX_SUBMITTED,
                                     7, 7, &completion) == ODRIVE_CAN_DRIVER_STALE_ATTEMPT);

    CHECK(odrive_can_driver_authorize_tx(&driver, next_attempt, 7, &frame) == ODRIVE_CAN_DRIVER_OK);
    CHECK(frame.id == 0x0f8);
    CHECK(frame.kind == ODRIVE_CAN_FRAME_DATA);
    CHECK(frame.dlc == 0);
    CHECK(odrive_can_driver_finish_tx(&driver, next_attempt, ODRIVE_CAN_DRIVER_TX_SUBMITTED,
                                     8, 8, &completion) == ODRIVE_CAN_DRIVER_OK);
    CHECK(completion.kind == ODRIVE_CAN_DRIVER_COMPLETION_SUBMITTED);
    CHECK(odrive_can_driver_take_report(&driver, first.id, &report) == ODRIVE_CAN_DRIVER_OK);
    CHECK(report.state == ODRIVE_CAN_DRIVER_STATE_SUBMITTED);
    CHECK(report.submitted_at.present);
    CHECK(report.submitted_at.us == 8);
}

/** @brief 验证发送尝试和操作的序号耗尽后拒绝签发。 */
static void sequence_exhaustion_does_not_wrap(void)
{
    odrive_can_driver_t driver;
    odrive_can_driver_tx_attempt_t attempt = {.id = {91, 92}, .sequence = 93};
    odrive_can_driver_operation_report_t report;

    CHECK(odrive_can_driver_init(&driver, TEST_NODE, 21) == ODRIVE_CAN_DRIVER_OK);
    odrive_can_driver_send_permit_t permit = prepare_clear_errors(&driver, 100);

    /* 仅在测试中构造无法通过实际循环到达的耗尽边界。 */
    driver._attempt_sequence = UINT64_MAX;
    permit.sequence = UINT64_MAX;
    CHECK(odrive_can_driver_begin_send(&driver, permit, 2, &attempt) == ODRIVE_CAN_DRIVER_STALE_PERMIT);
    CHECK(driver._attempt_sequence == UINT64_MAX);
    CHECK(attempt.id.incarnation == 91 && attempt.id.sequence == 92 && attempt.sequence == 93);
    CHECK(odrive_can_driver_take_report(&driver, permit.id, &report) == ODRIVE_CAN_DRIVER_OK);
    CHECK(report.state == ODRIVE_CAN_DRIVER_STATE_FAILED && !report.dispatching_at.present);
    CHECK(report.terminal_at.present && report.terminal_at.us == 2);

    driver._next_operation = UINT64_MAX;
    CHECK(odrive_can_driver_prepare_query(&driver, ODRIVE_CAN_QUERY_MOTOR_ERROR, 3, 100, &permit, NULL) ==
          ODRIVE_CAN_DRIVER_ID_EXHAUSTED);
    CHECK(driver._next_operation == UINT64_MAX);
    CHECK(permit.id.incarnation == 21 && permit.id.sequence == 0 && permit.sequence == UINT64_MAX);
    CHECK(odrive_can_driver_report(&driver, permit.id, &report) == ODRIVE_CAN_DRIVER_STALE_OPERATION);
}

/** @brief 验证迟到的结果回填即使返回错误，也会消费该发送尝试。 */
static void preserves_late_submission_when_consumed_attempt_is_replayed(void)
{
    odrive_can_driver_t driver;
    odrive_can_driver_tx_attempt_t attempt;
    odrive_can_driver_tx_completion_t completion;
    odrive_can_driver_operation_report_t report;
    odrive_can_frame_t frame;

    CHECK(odrive_can_driver_init(&driver, TEST_NODE, 30) == ODRIVE_CAN_DRIVER_OK);
    const odrive_can_driver_send_permit_t permit = prepare_clear_errors(&driver, 10);
    CHECK(odrive_can_driver_begin_send(&driver, permit, 2, &attempt) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_authorize_tx(&driver, attempt, 3, &frame) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_tick(&driver, 10) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_authorize_tx(&driver, attempt, 10, &frame) == ODRIVE_CAN_DRIVER_REVOKED);

    /* 到期后撤销后续发送授权，仍保留一次回填已观察到的发送结果的机会。 */
    CHECK(odrive_can_driver_finish_tx(&driver, attempt, ODRIVE_CAN_DRIVER_TX_SUBMITTED,
                                     10, 11, &completion) == ODRIVE_CAN_DRIVER_EVENT_AFTER_DEADLINE);
    CHECK(odrive_can_driver_finish_tx(&driver, attempt, ODRIVE_CAN_DRIVER_TX_SUBMITTED,
                                     9, 12, &completion) == ODRIVE_CAN_DRIVER_STALE_ATTEMPT);
    CHECK(odrive_can_driver_cancel_unsubmitted(&driver, attempt, 12, 12) ==
          ODRIVE_CAN_DRIVER_STALE_ATTEMPT);
    CHECK(odrive_can_driver_abandon_attempt(&driver, attempt, 12) == ODRIVE_CAN_DRIVER_STALE_ATTEMPT);
    CHECK(odrive_can_driver_take_report(&driver, permit.id, &report) ==
          ODRIVE_CAN_DRIVER_UNKNOWN_PENDING);
    CHECK(odrive_can_driver_acknowledge_unknown(&driver, permit.id, &report) ==
          ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_take_report(&driver, permit.id, &report) == ODRIVE_CAN_DRIVER_OK);
    CHECK(report.state == ODRIVE_CAN_DRIVER_STATE_UNKNOWN);
    CHECK(report.submitted_at.present);
    CHECK(report.submitted_at.us == 10);
    CHECK(report.tx_event_at.present);
    CHECK(report.tx_event_at.us == 10);
    CHECK(report.processed_at == 11);
}

/** @brief 验证发送授权撤销后，放弃尝试也会消费唯一的回填资格。 */
static void abandonment_consumes_attempt_while_report_remains_unknown(void)
{
    odrive_can_driver_t driver;
    odrive_can_driver_tx_attempt_t attempt;
    odrive_can_driver_tx_completion_t completion;
    odrive_can_driver_operation_report_t report;
    odrive_can_frame_t frame;

    CHECK(odrive_can_driver_init(&driver, TEST_NODE, 40) == ODRIVE_CAN_DRIVER_OK);
    const odrive_can_driver_send_permit_t permit = prepare_clear_errors(&driver, 100);
    CHECK(odrive_can_driver_begin_send(&driver, permit, 2, &attempt) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_authorize_tx(&driver, attempt, 2, &frame) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_cancel(&driver, permit.id, 3) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_authorize_tx(&driver, attempt, 4, &frame) == ODRIVE_CAN_DRIVER_REVOKED);
    CHECK(odrive_can_driver_abandon_attempt(&driver, attempt, 5) == ODRIVE_CAN_DRIVER_OK);

    CHECK(odrive_can_driver_abandon_attempt(&driver, attempt, 6) == ODRIVE_CAN_DRIVER_STALE_ATTEMPT);
    CHECK(odrive_can_driver_finish_tx(&driver, attempt, ODRIVE_CAN_DRIVER_TX_WOULD_BLOCK,
                                     6, 6, &completion) == ODRIVE_CAN_DRIVER_STALE_ATTEMPT);
    CHECK(odrive_can_driver_report(&driver, permit.id, &report) == ODRIVE_CAN_DRIVER_OK);
    CHECK(report.state == ODRIVE_CAN_DRIVER_STATE_UNKNOWN);
    CHECK(report.cancel_requested_at.present);
    CHECK(report.cancel_requested_at.us == 3);
    CHECK(!report.submitted_at.present);
    CHECK(!report.tx_event_at.present);
    CHECK(odrive_can_driver_acknowledge_unknown(&driver, permit.id, &report) ==
          ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_take_report(&driver, permit.id, &report) == ODRIVE_CAN_DRIVER_OK);
}

/** @brief 验证显式确认未知结果后，未解决的尝试副本也无法使用。 */
static void acknowledgement_closes_an_unresolved_attempt_before_report_is_taken(void)
{
    odrive_can_driver_t driver;
    odrive_can_driver_tx_attempt_t attempt;
    odrive_can_driver_tx_completion_t completion;
    odrive_can_driver_operation_report_t report;
    odrive_can_frame_t frame;

    CHECK(odrive_can_driver_init(&driver, TEST_NODE, 41) == ODRIVE_CAN_DRIVER_OK);
    const odrive_can_driver_send_permit_t permit = prepare_clear_errors(&driver, 100);
    CHECK(odrive_can_driver_begin_send(&driver, permit, 2, &attempt) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_authorize_tx(&driver, attempt, 2, &frame) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_cancel(&driver, permit.id, 3) == ODRIVE_CAN_DRIVER_OK);

    /* 模拟调用方已结束底层发送工作，但未查明发送结果。 */
    CHECK(odrive_can_driver_acknowledge_unknown(&driver, permit.id, &report) ==
          ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_finish_tx(&driver, attempt, ODRIVE_CAN_DRIVER_TX_SUBMITTED,
                                     3, 4, &completion) == ODRIVE_CAN_DRIVER_REVOKED);
    CHECK(odrive_can_driver_authorize_tx(&driver, attempt, 4, &frame) == ODRIVE_CAN_DRIVER_REVOKED);
    CHECK(odrive_can_driver_cancel_unsubmitted(&driver, attempt, 4, 4) == ODRIVE_CAN_DRIVER_REVOKED);
    CHECK(odrive_can_driver_abandon_attempt(&driver, attempt, 4) == ODRIVE_CAN_DRIVER_REVOKED);
    CHECK(odrive_can_driver_take_report(&driver, permit.id, &report) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_authorize_tx(&driver, attempt, 5, &frame) == ODRIVE_CAN_DRIVER_STALE_ATTEMPT);
    CHECK(report.state == ODRIVE_CAN_DRIVER_STATE_UNKNOWN);
    CHECK(!report.submitted_at.present);
    CHECK(report.processed_at == 3);
}

/** @brief 验证准备失败时保留编码诊断，且失败不占用操作槽或推进处理时钟。 */
static void prepare_reports_encoding_errors_without_consuming_operation_slot(void)
{
    odrive_can_driver_t driver;
    odrive_can_command_t command = {
        .kind = ODRIVE_CAN_COMMAND_SET_INPUT_VEL,
        .value.set_input_vel = {.velocity = INFINITY, .torque_ff = 0.0f},
    };
    odrive_can_driver_send_permit_t permit = {.id = {91, 92}, .sequence = 93};
    odrive_can_codec_error_t error = {0};

    CHECK(odrive_can_driver_init(&driver, TEST_NODE, 43) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_prepare_command(&driver, &command, 20, 100, &permit, &error) ==
          ODRIVE_CAN_DRIVER_ENCODE_ERROR);
    CHECK(error.status == ODRIVE_CAN_CODEC_NON_FINITE && error.field == ODRIVE_CAN_FLOAT_FIELD_VELOCITY);

    command.value.set_input_vel.velocity = 0.0f;
    command.value.set_input_vel.torque_ff = NAN;
    CHECK(odrive_can_driver_prepare_command(&driver, &command, 20, 100, &permit, &error) ==
          ODRIVE_CAN_DRIVER_ENCODE_ERROR);
    CHECK(error.status == ODRIVE_CAN_CODEC_NON_FINITE && error.field == ODRIVE_CAN_FLOAT_FIELD_TORQUE_FF);
    CHECK(odrive_can_driver_prepare_query(&driver, (odrive_can_query_t)0, 20, 100, &permit, &error) ==
          ODRIVE_CAN_DRIVER_ENCODE_ERROR);
    CHECK(error.status == ODRIVE_CAN_CODEC_INVALID_ARGUMENT);

    error = (odrive_can_codec_error_t){.status = ODRIVE_CAN_CODEC_UNRELATED, .value = 101};
    CHECK(odrive_can_driver_prepare_query(&driver, ODRIVE_CAN_QUERY_MOTOR_ERROR, 5, 5, &permit, &error) ==
          ODRIVE_CAN_DRIVER_DEADLINE_ELAPSED);
    CHECK(error.status == ODRIVE_CAN_CODEC_UNRELATED && error.value == 101);
    CHECK(permit.id.incarnation == 91 && permit.id.sequence == 92 && permit.sequence == 93);

    CHECK(odrive_can_driver_prepare_query(&driver, ODRIVE_CAN_QUERY_MOTOR_ERROR, 5, 100, &permit, &error) ==
          ODRIVE_CAN_DRIVER_OK);
    CHECK(permit.id.incarnation == 43 && permit.id.sequence == 0);
    CHECK(error.status == ODRIVE_CAN_CODEC_UNRELATED && error.value == 101);
}

/** @brief 验证收帧时可取得具体协议错误，已准备的操作仍有效。 */
static void decode_errors_report_the_reason_without_changing_the_operation(void)
{
    odrive_can_driver_t driver;
    odrive_can_driver_operation_report_t report;
    odrive_can_driver_tx_attempt_t attempt;
    odrive_can_codec_error_t error = {0};
    odrive_can_message_t message;
    const uint8_t data[8] = {0};
    odrive_can_frame_view_t frame = {
        .id_kind = ODRIVE_CAN_ID_STANDARD,
        .id = 0x0e3,
        .kind = ODRIVE_CAN_FRAME_FD,
        .data = data,
        .data_len = sizeof(data),
        .dlc = 8,
    };

    CHECK(odrive_can_driver_init(&driver, TEST_NODE, 42) == ODRIVE_CAN_DRIVER_OK);
    const odrive_can_driver_send_permit_t permit = prepare_clear_errors(&driver, 100);
    CHECK(odrive_can_driver_ingest(&driver, &frame, 2, &message, &error) ==
          ODRIVE_CAN_DRIVER_DECODE_ERROR);
    CHECK(error.status == ODRIVE_CAN_CODEC_UNSUPPORTED_CAN_FD);

    frame.id = 0x0e8;
    frame.kind = ODRIVE_CAN_FRAME_DATA;
    CHECK(odrive_can_driver_ingest(&driver, &frame, 3, &message, &error) ==
          ODRIVE_CAN_DRIVER_DECODE_ERROR);
    CHECK(error.status == ODRIVE_CAN_CODEC_UNSUPPORTED_COMMAND);
    CHECK(odrive_can_driver_ingest(&driver, &frame, 4, &message, NULL) ==
          ODRIVE_CAN_DRIVER_DECODE_ERROR);

    CHECK(odrive_can_driver_report(&driver, permit.id, &report) == ODRIVE_CAN_DRIVER_OK);
    CHECK(report.state == ODRIVE_CAN_DRIVER_STATE_PREPARED);
    CHECK(report.processed_at == 1);
    CHECK(!report.submitted_at.present);
    CHECK(odrive_can_driver_begin_send(&driver, permit, 5, &attempt) == ODRIVE_CAN_DRIVER_OK);
}

/** @brief 验证接纳回复时使用 Driver 内当前候选的原始值。 */
static void replacing_a_candidate_revokes_old_token_and_preserves_received_payload(void)
{
    odrive_can_driver_t driver;
    odrive_can_driver_send_permit_t permit;
    odrive_can_driver_tx_attempt_t attempt;
    odrive_can_driver_tx_completion_t completion;
    odrive_can_driver_response_candidate_t old_candidate;
    odrive_can_driver_response_candidate_t current_candidate;
    odrive_can_cached_response_t snapshot;
    odrive_can_driver_operation_report_t report;
    odrive_can_message_t message;
    odrive_can_frame_t tx_frame;
    uint8_t data[8] = {0x01};
    const odrive_can_frame_view_t rx_frame = {
        .id_kind = ODRIVE_CAN_ID_STANDARD,
        .id = 0x0e3,
        .kind = ODRIVE_CAN_FRAME_DATA,
        .data = data,
        .data_len = sizeof(data),
        .dlc = 8,
    };

    CHECK(odrive_can_driver_init(&driver, TEST_NODE, 50) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_prepare_query(&driver, ODRIVE_CAN_QUERY_MOTOR_ERROR, 1, 100, &permit, NULL) ==
          ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_begin_send(&driver, permit, 2, &attempt) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_authorize_tx(&driver, attempt, 3, &tx_frame) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_finish_tx(&driver, attempt, ODRIVE_CAN_DRIVER_TX_SUBMITTED,
                                     4, 4, &completion) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_ingest(&driver, &rx_frame, 5, &message, NULL) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_pending_response(&driver, permit.id, &old_candidate, &snapshot) ==
          ODRIVE_CAN_DRIVER_OK);
    CHECK(snapshot.response.value.motor_error == 1);

    data[0] = 2;
    CHECK(odrive_can_driver_ingest(&driver, &rx_frame, 5, &message, NULL) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_pending_response(&driver, permit.id, &current_candidate, &snapshot) ==
          ODRIVE_CAN_DRIVER_OK);
    CHECK(snapshot.response.value.motor_error == 2);
    CHECK(odrive_can_driver_accept_response(&driver, old_candidate, 6) ==
          ODRIVE_CAN_DRIVER_STALE_OPERATION);

    snapshot.response.value.motor_error = 99;
    snapshot.received_at_us = 77;
    CHECK(odrive_can_driver_accept_response(&driver, current_candidate, 6) == ODRIVE_CAN_DRIVER_OK);
    CHECK(odrive_can_driver_accept_response(&driver, current_candidate, 7) ==
          ODRIVE_CAN_DRIVER_STALE_OPERATION);
    CHECK(odrive_can_driver_take_report(&driver, permit.id, &report) == ODRIVE_CAN_DRIVER_OK);
    CHECK(report.state == ODRIVE_CAN_DRIVER_STATE_OBSERVED);
    CHECK(report.response_present);
    CHECK(report.response.received_at_us == 5);
    CHECK(report.response.response.kind == ODRIVE_CAN_RESPONSE_MOTOR_ERROR);
    CHECK(report.response.response.value.motor_error == 2);
}

/**
 * @brief 运行 C Driver 专属测试。
 * @return 全部断言通过时返回 EXIT_SUCCESS。
 */
int main(void)
{
    rejects_foreign_and_reconstructed_driver_identities();
    rejects_replay_of_permit_and_attempt_after_would_block();
    sequence_exhaustion_does_not_wrap();
    preserves_late_submission_when_consumed_attempt_is_replayed();
    abandonment_consumes_attempt_while_report_remains_unknown();
    acknowledgement_closes_an_unresolved_attempt_before_report_is_taken();
    prepare_reports_encoding_errors_without_consuming_operation_slot();
    decode_errors_report_the_reason_without_changing_the_operation();
    replacing_a_candidate_revokes_old_token_and_preserves_received_payload();
    return EXIT_SUCCESS;
}
