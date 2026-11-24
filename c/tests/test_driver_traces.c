/*
 * Copyright The odrive-can-driver Contributors
 */

/**
 * @file
 * @brief 用共享操作时序测试向量验证 C Driver。
 */

#include "odrive_can_driver/driver.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** @brief 测试向量的一行和按制表符拆分的字段。 */
typedef struct {
    size_t line;             /**< 测试向量文件中的行号，从 1 开始。 */
    size_t count;            /**< 已解析的字段数。 */
    const char *fields[12];  /**< 引用当前行缓冲区的字段。 */
} trace_row_t;

/** @brief 单个用例中的当前操作和相关凭证。 */
typedef struct {
    odrive_can_driver_t driver;              /**< 当前用例的 Driver 存储。 */
    odrive_can_driver_send_permit_t permit;  /**< 最近签发的发送凭证。 */
    odrive_can_driver_tx_attempt_t attempt;  /**< 当前的发送尝试。 */
    uint64_t incarnation;                    /**< 本次测试运行中唯一的 Driver 实例编号。 */
    bool query;                              /**< 当前操作为 MotorError 查询时为 true。 */
} trace_case_t;

/**
 * @brief 断言失败时输出测试向量行号和失败条件，并退出程序。
 * @param[in] row 提供诊断行号的测试向量。
 * @param[in] condition 预期成立的布尔表达式。
 */
#define CHECK(row, condition)                                                              \
    do {                                                                                   \
        if (!(condition)) {                                                                \
            fprintf(stderr, "driver_traces.tsv:%zu: %s\n", (row)->line, #condition);        \
            exit(EXIT_FAILURE);                                                            \
        }                                                                                  \
    } while (0)

/**
 * @brief 解析测试向量字段中的十进制时间戳或错误位。
 * @param[in] row 提供诊断行号的测试向量。
 * @param[in] field 十进制数字字符串。
 * @return 解析得到的值。
 */
static uint64_t number(const trace_row_t *row, const char *field)
{
    char *end;
    errno = 0;
    const unsigned long long value = strtoull(field, &end, 10);
    CHECK(row, field[0] >= '0' && field[0] <= '9' && *end == '\0' && errno == 0);
    return value;
}

/**
 * @brief 检查可缺省的时间戳及其预期值。
 * @param[in] row 提供诊断行号的测试向量。
 * @param[in] actual 可缺省的时间戳，单位为微秒。
 * @param[in] field 十进制预期时刻，缺省时为短横线。
 */
static void check_time(const trace_row_t *row, odrive_can_driver_optional_time_t actual,
                       const char *field)
{
    if (strcmp(field, "-") == 0) {
        CHECK(row, !actual.present);
    } else {
        CHECK(row, actual.present && actual.us == number(row, field));
    }
}

/**
 * @brief 检查 MotorError 回复的类型、解码后的错误位和接收时间。
 * @param[in] row 提供诊断行号的测试向量。
 * @param[in] present 是否有回复。
 * @param[in] actual 有回复时指向实际观测值。
 * @param[in] time 预期接收时刻，没有回复时为短横线。
 * @param[in] bits 预期错误位，没有回复时为短横线。
 */
static void check_response(const trace_row_t *row, bool present,
                           const odrive_can_cached_response_t *actual,
                           const char *time, const char *bits)
{
    if (strcmp(time, "-") == 0) {
        CHECK(row, !present && strcmp(bits, "-") == 0);
        return;
    }

    CHECK(row, present);
    CHECK(row, actual->received_at_us == number(row, time));
    CHECK(row, actual->response.kind == ODRIVE_CAN_RESPONSE_MOTOR_ERROR);
    CHECK(row, actual->response.value.motor_error == number(row, bits));
}

/**
 * @brief 按独立预期检查操作报告的标识、内容、状态和时间。
 * @param[in] row 包含预期报告的测试向量。
 * @param[in] test 当前正在重放的用例。
 * @param[in] report Driver 返回的报告。
 */
static void check_report(const trace_row_t *row, const trace_case_t *test,
                         const odrive_can_driver_operation_report_t *report)
{
    CHECK(row, row->count == 12);
    const char *const *field = row->fields;
    if (strcmp(field[1], "Prepared") == 0) {
        CHECK(row, report->state == ODRIVE_CAN_DRIVER_STATE_PREPARED);
    } else if (strcmp(field[1], "Cancelled") == 0) {
        CHECK(row, report->state == ODRIVE_CAN_DRIVER_STATE_CANCELLED);
    } else if (strcmp(field[1], "Unknown") == 0) {
        CHECK(row, report->state == ODRIVE_CAN_DRIVER_STATE_UNKNOWN);
    } else if (strcmp(field[1], "Submitted") == 0) {
        CHECK(row, report->state == ODRIVE_CAN_DRIVER_STATE_SUBMITTED);
    } else if (strcmp(field[1], "Observed") == 0) {
        CHECK(row, report->state == ODRIVE_CAN_DRIVER_STATE_OBSERVED);
    } else {
        CHECK(row, strcmp(field[1], "TimedOut") == 0);
        CHECK(row, report->state == ODRIVE_CAN_DRIVER_STATE_TIMED_OUT);
    }

    CHECK(row, report->id.incarnation == test->permit.id.incarnation);
    CHECK(row, report->id.sequence == test->permit.id.sequence);
    if (test->query) {
        CHECK(row, report->kind == ODRIVE_CAN_DRIVER_OPERATION_QUERY);
        CHECK(row, report->value.query == ODRIVE_CAN_QUERY_MOTOR_ERROR);
    } else {
        CHECK(row, report->kind == ODRIVE_CAN_DRIVER_OPERATION_COMMAND);
        CHECK(row, report->value.command.kind == ODRIVE_CAN_COMMAND_CLEAR_ERRORS);
    }

    CHECK(row, report->prepared_at == number(row, field[2]));
    CHECK(row, report->deadline == number(row, field[3]));
    check_time(row, report->dispatching_at, field[4]);
    check_time(row, report->submitted_at, field[5]);
    check_time(row, report->tx_event_at, field[6]);
    check_time(row, report->cancel_requested_at, field[7]);
    check_time(row, report->terminal_at, field[8]);
    CHECK(row, report->processed_at == number(row, field[9]));
    check_response(row, report->response_present, &report->response, field[10], field[11]);
}

/**
 * @brief 直接构造节点 7 的 MotorError 回复帧作为输入，无需调用编解码器的编码接口。
 * @param[in] row 给出接收时间戳和错误位的测试向量。
 * @param[in,out] test 当前用例，收帧会更新其中的 Driver。
 */
static void receive_motor_error(const trace_row_t *row, trace_case_t *test)
{
    CHECK(row, row->count == 3);
    const uint64_t value = number(row, row->fields[2]);
    CHECK(row, value <= UINT32_MAX);
    const uint32_t bits = (uint32_t)value;
    const uint8_t data[8] = {
        (uint8_t)bits, (uint8_t)(bits >> 8), (uint8_t)(bits >> 16), (uint8_t)(bits >> 24)
    };
    const odrive_can_frame_view_t frame = {
        .id_kind = ODRIVE_CAN_ID_STANDARD,
        .id = 0x0e3,
        .kind = ODRIVE_CAN_FRAME_DATA,
        .data = data,
        .data_len = sizeof(data),
        .dlc = 8
    };
    odrive_can_message_t message;
    CHECK(row, odrive_can_driver_ingest(&test->driver, &frame, number(row, row->fields[1]),
                                       &message, NULL) == ODRIVE_CAN_DRIVER_OK);
    CHECK(row, message.kind == ODRIVE_CAN_MESSAGE_RESPONSE);
    CHECK(row, message.value.response.kind == ODRIVE_CAN_RESPONSE_MOTOR_ERROR);
    CHECK(row, message.value.response.value.motor_error == bits);
}

/**
 * @brief 重放一行测试向量并验证预期。
 * @param[in] row 解析后的事件或预期报告。
 * @param[in,out] test 当前用例及其操作状态。
 */
static void replay(const trace_row_t *row, trace_case_t *test)
{
    odrive_can_driver_t *driver = &test->driver;
    const char *const *field = row->fields;
    if (strcmp(field[0], "case") == 0) {
        CHECK(row, row->count == 2);
        test->incarnation++;
        CHECK(row, odrive_can_driver_init(driver, 7, test->incarnation) == ODRIVE_CAN_DRIVER_OK);
    } else if (strcmp(field[0], "prepare") == 0) {
        CHECK(row, row->count == 4);
        const uint64_t now = number(row, field[2]);
        const uint64_t deadline = number(row, field[3]);
        test->query = strcmp(field[1], "query") == 0;
        if (test->query) {
            CHECK(row, odrive_can_driver_prepare_query(driver, ODRIVE_CAN_QUERY_MOTOR_ERROR,
                                                       now, deadline, &test->permit, NULL) ==
                       ODRIVE_CAN_DRIVER_OK);
        } else {
            CHECK(row, strcmp(field[1], "command") == 0);
            const odrive_can_command_t command = { .kind = ODRIVE_CAN_COMMAND_CLEAR_ERRORS };
            CHECK(row, odrive_can_driver_prepare_command(driver, &command, now, deadline,
                                                         &test->permit, NULL) == ODRIVE_CAN_DRIVER_OK);
        }
    } else if (strcmp(field[0], "begin") == 0) {
        CHECK(row, row->count == 2);
        CHECK(row, odrive_can_driver_begin_send(driver, test->permit, number(row, field[1]),
                                                &test->attempt) == ODRIVE_CAN_DRIVER_OK);
    } else if (strcmp(field[0], "gate") == 0) {
        CHECK(row, row->count == 3);
        odrive_can_frame_t frame;
        const odrive_can_driver_result_t result = odrive_can_driver_authorize_tx(
            driver, test->attempt, number(row, field[1]), &frame);
        if (strcmp(field[2], "ok") == 0) {
            CHECK(row, result == ODRIVE_CAN_DRIVER_OK);
        } else {
            CHECK(row, strcmp(field[2], "revoked") == 0);
            CHECK(row, result == ODRIVE_CAN_DRIVER_REVOKED);
        }
    } else if (strcmp(field[0], "submit") == 0 || strcmp(field[0], "block") == 0) {
        CHECK(row, row->count == 4);
        const odrive_can_driver_tx_outcome_t outcome = strcmp(field[0], "submit") == 0
            ? ODRIVE_CAN_DRIVER_TX_SUBMITTED
            : ODRIVE_CAN_DRIVER_TX_WOULD_BLOCK;
        odrive_can_driver_tx_completion_t completion;
        const odrive_can_driver_result_t result = odrive_can_driver_finish_tx(
            driver, test->attempt, outcome, number(row, field[1]), number(row, field[2]),
            &completion);
        if (strcmp(field[3], "ok") == 0) {
            CHECK(row, result == ODRIVE_CAN_DRIVER_OK);
            CHECK(row, completion.kind == ODRIVE_CAN_DRIVER_COMPLETION_SUBMITTED);
        } else if (strcmp(field[3], "rollback") == 0) {
            CHECK(row, result == ODRIVE_CAN_DRIVER_CLOCK_ROLLBACK);
        } else if (strcmp(field[3], "late") == 0) {
            CHECK(row, result == ODRIVE_CAN_DRIVER_EVENT_AFTER_DEADLINE);
        } else if (strcmp(field[3], "retry") == 0) {
            CHECK(row, result == ODRIVE_CAN_DRIVER_OK);
            CHECK(row, completion.kind == ODRIVE_CAN_DRIVER_COMPLETION_RETRY);
            CHECK(row, completion.retry.id.incarnation == test->permit.id.incarnation);
            CHECK(row, completion.retry.id.sequence == test->permit.id.sequence);
            test->permit = completion.retry;
        } else {
            CHECK(row, strcmp(field[3], "failed") == 0);
            CHECK(row, result == ODRIVE_CAN_DRIVER_OK);
            CHECK(row, completion.kind == ODRIVE_CAN_DRIVER_COMPLETION_FAILED);
        }
    } else if (strcmp(field[0], "rx") == 0) {
        receive_motor_error(row, test);
    } else if (strcmp(field[0], "tick") == 0) {
        CHECK(row, row->count == 2);
        CHECK(row, odrive_can_driver_tick(driver, number(row, field[1])) == ODRIVE_CAN_DRIVER_OK);
    } else if (strcmp(field[0], "cancel") == 0) {
        CHECK(row, row->count == 2);
        CHECK(row, odrive_can_driver_cancel(driver, test->permit.id, number(row, field[1])) ==
                   ODRIVE_CAN_DRIVER_OK);
    } else if (strcmp(field[0], "peek") == 0 || strcmp(field[0], "accept") == 0) {
        odrive_can_driver_response_candidate_t candidate;
        odrive_can_cached_response_t response;
        const odrive_can_driver_result_t result = odrive_can_driver_pending_response(
            driver, test->permit.id, &candidate, &response);
        if (strcmp(field[0], "accept") == 0) {
            CHECK(row, row->count == 2 && result == ODRIVE_CAN_DRIVER_OK);
            CHECK(row, odrive_can_driver_accept_response(driver, candidate, number(row, field[1])) ==
                       ODRIVE_CAN_DRIVER_OK);
        } else {
            CHECK(row, row->count == 3);
            const bool present = strcmp(field[1], "-") != 0;
            CHECK(row, result == (present ? ODRIVE_CAN_DRIVER_OK : ODRIVE_CAN_DRIVER_NO_RESPONSE));
            check_response(row, present, &response, field[1], field[2]);
        }
    } else if (strcmp(field[0], "report") == 0 || strcmp(field[0], "take") == 0) {
        CHECK(row, row->count >= 2);
        odrive_can_driver_operation_report_t report;
        const odrive_can_driver_result_t result = strcmp(field[0], "take") == 0
            ? odrive_can_driver_take_report(driver, test->permit.id, &report)
            : odrive_can_driver_report(driver, test->permit.id, &report);
        if (strcmp(field[1], "unknown_pending") == 0) {
            CHECK(row, result == ODRIVE_CAN_DRIVER_UNKNOWN_PENDING);
        } else {
            CHECK(row, result == ODRIVE_CAN_DRIVER_OK);
            check_report(row, test, &report);
        }
    } else if (strcmp(field[0], "missing") == 0) {
        odrive_can_driver_operation_report_t report;
        odrive_can_driver_response_candidate_t candidate;
        odrive_can_cached_response_t response;
        CHECK(row, odrive_can_driver_report(driver, test->permit.id, &report) ==
                   ODRIVE_CAN_DRIVER_STALE_OPERATION);
        CHECK(row, odrive_can_driver_pending_response(driver, test->permit.id, &candidate, &response) ==
                   ODRIVE_CAN_DRIVER_STALE_OPERATION);
    } else if (strcmp(field[0], "ack") == 0) {
        odrive_can_driver_operation_report_t report;
        CHECK(row, odrive_can_driver_acknowledge_unknown(driver, test->permit.id, &report) ==
                   ODRIVE_CAN_DRIVER_OK);
        CHECK(row, report.state == ODRIVE_CAN_DRIVER_STATE_UNKNOWN);
    } else {
        CHECK(row, !"unknown fixture event");
    }
}

/**
 * @brief 运行共享测试向量中的全部操作时序用例。
 * @param[in] argc 参数个数，包含程序名。
 * @param[in] argv 参数数组，程序名后的第一项是测试向量文件路径。
 * @return 所有用例均通过且文件关闭成功时返回 EXIT_SUCCESS。
 */
int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: %s driver_traces.tsv\n", argv[0]);
        return EXIT_FAILURE;
    }
    FILE *fixture = fopen(argv[1], "r");
    if (fixture == NULL) {
        perror(argv[1]);
        return EXIT_FAILURE;
    }

    trace_case_t test = {0};
    trace_row_t row = {0};
    char line[256];
    while (fgets(line, sizeof(line), fixture) != NULL) {
        row.line++;
        line[strcspn(line, "\r\n")] = '\0';
        if (line[0] == '\0' || line[0] == '#') {
            continue;
        }

        row.count = 0;
        for (char *field = strtok(line, "\t"); field != NULL; field = strtok(NULL, "\t")) {
            CHECK(&row, row.count < sizeof(row.fields) / sizeof(row.fields[0]));
            row.fields[row.count++] = field;
        }
        CHECK(&row, row.count > 0);
        replay(&row, &test);
    }
    CHECK(&row, !ferror(fixture));
    CHECK(&row, test.incarnation != 0);
    return fclose(fixture) == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
