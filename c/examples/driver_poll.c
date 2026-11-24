/* Copyright The odrive-can-driver Contributors */

/**
 * @file
 * @brief 用结果可控的非阻塞模拟收发推进单槽查询。
 *
 * 应用持有收发端和单调微秒时钟。应用可以在两轮发送之间分发其他报文，
 * 并决定是否接纳查询的候选回复。
 */

#include <odrive_can_driver/driver.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

/** @brief 一次模拟发送调用结束后返回的证据。 */
typedef enum {
    MOCK_FIFO_FULL, /**< 调用已结束，帧未进入发送队列。 */
    MOCK_ACCEPTED,  /**< 本地发送端已接收该帧。 */
    MOCK_UNCERTAIN, /**< 调用已结束，是否提交仍未知。 */
} mock_tx_result_t;

/** @brief 代替应用 CAN HAL 的可控模拟发送端。 */
typedef struct {
    mock_tx_result_t next_result; /**< 下一次调用返回的证据。 */
    bool unresolved;              /**< 模拟队列中有需要核查的发送。 */
} mock_tx_t;

/**
 * @brief 执行一次非阻塞的模拟 CAN 发送调用。
 * @param[in,out] tx 由应用持有的模拟发送端。
 * @param[in] frame Driver 为本次调用授权发送的帧。
 * @return 模拟发送端给出的本次调用结果。
 */
static mock_tx_result_t mock_send(mock_tx_t *tx, const odrive_can_frame_t *frame)
{
    if (tx->next_result == MOCK_UNCERTAIN || frame->id != 0x029 ||
        frame->kind != ODRIVE_CAN_FRAME_REMOTE || frame->dlc != 8 || frame->data_len != 0) {
        tx->unresolved = true;
        return MOCK_UNCERTAIN;
    }
    return tx->next_result;
}

/**
 * @brief 结束未处理完的发送并核查模拟队列。
 * @param[in,out] tx 由应用持有的模拟发送端。
 *
 * 使用实际适配器时，调用方须取得该后端的相关证据才能确认完成。
 */
static void mock_resolve_transmitter(mock_tx_t *tx)
{
    tx->unresolved = false;
}

/**
 * @brief 使用凭证在本轮最多进行一次底层发送调用。
 * @param[in,out] driver 绑定到查询节点的 Driver。
 * @param[in,out] tx 由应用持有的模拟发送端。
 * @param[in] permit 本轮使用的一次性发送凭证。
 * @param[in] now_us 处理此轮调用的单调时刻，单位为微秒。
 * @param[out] has_completion 指示 completion 是否写入。
 * @param[out] completion 发送结果已知且成功回填时，写入回填结果。
 * @return Driver 的返回状态。发送结果未知时，调用 abandon_attempt 记录不确定性。
 */
static odrive_can_driver_result_t send_once(odrive_can_driver_t *driver, mock_tx_t *tx,
                                            odrive_can_driver_send_permit_t permit, uint64_t now_us,
                                            bool *has_completion,
                                            odrive_can_driver_tx_completion_t *completion)
{
    *has_completion = false;
    odrive_can_driver_tx_attempt_t attempt;
    odrive_can_driver_result_t result = odrive_can_driver_begin_send(driver, permit, now_us, &attempt);
    if (result != ODRIVE_CAN_DRIVER_OK) {
        return result;
    }

    odrive_can_frame_t frame;
    result = odrive_can_driver_authorize_tx(driver, attempt, now_us, &frame);
    if (result != ODRIVE_CAN_DRIVER_OK) {
        /* 本次尝试尚未调用底层发送，帧不可能入队。 */
        const odrive_can_driver_result_t resolved =
            odrive_can_driver_cancel_unsubmitted(driver, attempt, now_us, now_us);
        return resolved == ODRIVE_CAN_DRIVER_OK ? result : resolved;
    }
    const mock_tx_result_t native = mock_send(tx, &frame);

    /* 模拟发送在授权后一微秒同步完成。 */
    const uint64_t event_at_us = now_us + 1;
    if (native == MOCK_UNCERTAIN) {
        return odrive_can_driver_abandon_attempt(driver, attempt, event_at_us);
    }
    const odrive_can_driver_tx_outcome_t outcome = native == MOCK_FIFO_FULL
        ? ODRIVE_CAN_DRIVER_TX_WOULD_BLOCK : ODRIVE_CAN_DRIVER_TX_SUBMITTED;
    result = odrive_can_driver_finish_tx(driver, attempt, outcome, event_at_us, event_at_us,
                                        completion);
    *has_completion = result == ODRIVE_CAN_DRIVER_OK;
    return result;
}

/**
 * @brief 在有未发送证据时重试查询，并接纳候选回复。
 * @return 查询以符合预期的 Observed 报告结束时返回 true。
 */
static bool observe_reply(void)
{
    odrive_can_driver_t driver;
    odrive_can_driver_send_permit_t permit;
    if (odrive_can_driver_init(&driver, 1, 1) != ODRIVE_CAN_DRIVER_OK ||
        odrive_can_driver_prepare_query(&driver, ODRIVE_CAN_QUERY_ENCODER_ESTIMATES,
                                       100, 1000, &permit, NULL) != ODRIVE_CAN_DRIVER_OK) {
        return false;
    }
    const odrive_can_driver_operation_id_t id = permit.id;

    mock_tx_t tx = {.next_result = MOCK_FIFO_FULL};
    bool has_completion;
    odrive_can_driver_tx_completion_t completion;
    if (send_once(&driver, &tx, permit, 101, &has_completion, &completion) != ODRIVE_CAN_DRIVER_OK ||
        !has_completion ||
        completion.kind != ODRIVE_CAN_DRIVER_COMPLETION_RETRY) {
        return false;
    }

    /* 应用可在两次非阻塞发送调用之间处理接收帧并推进时钟。 */
    const uint8_t heartbeat[] = {0, 0, 0, 0, ODRIVE_CAN_AXIS_STATE_IDLE, 0, 0, 0};
    odrive_can_frame_view_t rx = {
        .id_kind = ODRIVE_CAN_ID_STANDARD,
        .id = 0x021,
        .kind = ODRIVE_CAN_FRAME_DATA,
        .data = heartbeat,
        .data_len = sizeof heartbeat,
        .dlc = sizeof heartbeat,
    };
    odrive_can_message_t message;
    if (odrive_can_driver_ingest(&driver, &rx, 150, &message, NULL) != ODRIVE_CAN_DRIVER_OK) {
        return false;
    }

    permit = completion.retry;
    tx.next_result = MOCK_ACCEPTED;
    if (odrive_can_driver_tick(&driver, 200) != ODRIVE_CAN_DRIVER_OK ||
        send_once(&driver, &tx, permit, 200, &has_completion, &completion) != ODRIVE_CAN_DRIVER_OK ||
        !has_completion ||
        completion.kind != ODRIVE_CAN_DRIVER_COMPLETION_SUBMITTED) {
        return false;
    }

    odrive_can_driver_response_candidate_t candidate;
    odrive_can_cached_response_t response;
    if (odrive_can_driver_pending_response(&driver, id, &candidate, &response) !=
        ODRIVE_CAN_DRIVER_NO_RESPONSE) {
        return false;
    }

    const uint8_t data[] = {0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0xc0};
    rx.id = 0x029;
    rx.data = data;
    rx.data_len = sizeof data;
    rx.dlc = sizeof data;
    if (odrive_can_driver_ingest(&driver, &rx, 230, &message, NULL) != ODRIVE_CAN_DRIVER_OK ||
        odrive_can_driver_pending_response(&driver, id, &candidate, &response) !=
            ODRIVE_CAN_DRIVER_OK) {
        return false;
    }

    /* 应用根据收到的候选快照作出接纳决定。 */
    odrive_can_driver_operation_report_t report;
    return response.response.kind == ODRIVE_CAN_RESPONSE_ENCODER_ESTIMATES &&
           response.response.value.encoder_estimates.velocity == -2.0f &&
           odrive_can_driver_accept_response(&driver, candidate, 231) == ODRIVE_CAN_DRIVER_OK &&
           odrive_can_driver_take_report(&driver, id, &report) == ODRIVE_CAN_DRIVER_OK &&
           report.state == ODRIVE_CAN_DRIVER_STATE_OBSERVED && report.submitted_at.present &&
           report.submitted_at.us == 201 && report.response_present &&
           report.response.received_at_us == 230;
}

/**
 * @brief 在应用处理完遗留的发送工作前，继续隔离未知结果。
 * @return 显式处理遗留发送后，取走 Unknown 报告并释放操作槽时返回 true。
 */
static bool preserve_unknown(void)
{
    odrive_can_driver_t driver;
    odrive_can_driver_send_permit_t permit;
    if (odrive_can_driver_init(&driver, 1, 2) != ODRIVE_CAN_DRIVER_OK ||
        odrive_can_driver_prepare_query(&driver, ODRIVE_CAN_QUERY_ENCODER_ESTIMATES,
                                       100, 1000, &permit, NULL) != ODRIVE_CAN_DRIVER_OK) {
        return false;
    }

    mock_tx_t tx = {.next_result = MOCK_UNCERTAIN};
    bool has_completion;
    odrive_can_driver_tx_completion_t completion;
    odrive_can_driver_operation_report_t report;
    if (send_once(&driver, &tx, permit, 101, &has_completion, &completion) != ODRIVE_CAN_DRIVER_OK ||
        has_completion ||
        !tx.unresolved ||
        odrive_can_driver_take_report(&driver, permit.id, &report) != ODRIVE_CAN_DRIVER_UNKNOWN_PENDING) {
        return false;
    }

    mock_resolve_transmitter(&tx);
    return !tx.unresolved &&
           odrive_can_driver_acknowledge_unknown(&driver, permit.id, &report) == ODRIVE_CAN_DRIVER_OK &&
           odrive_can_driver_take_report(&driver, permit.id, &report) == ODRIVE_CAN_DRIVER_OK &&
           report.state == ODRIVE_CAN_DRIVER_STATE_UNKNOWN;
}

/**
 * @brief 演示本地重试、查询回复接纳以及未知结果处置。
 * @return 两种模拟情形均产生预期报告时返回 0。
 */
int main(void)
{
    if (!observe_reply() || !preserve_unknown()) {
        return EXIT_FAILURE;
    }
    puts("Mock query observed a reply after a proved unsent retry.");
    puts("Uncertain submission remained Unknown after explicit queue reconciliation.");
    return EXIT_SUCCESS;
}
