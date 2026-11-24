/* Copyright The odrive-can-driver Contributors */

/**
 * @file
 * @brief CANSimple 操作状态、发送授权和查询回复接纳。
 */

#include <odrive_can_driver/driver.h>

#include <stddef.h>

static odrive_can_driver_optional_time_t time_present(uint64_t us)
{
    return (odrive_can_driver_optional_time_t){.present = true, .us = us};
}

static bool advance_time(odrive_can_driver_t *driver, uint64_t at)
{
    if (driver->_last.present && at < driver->_last.us) {
        return false;
    }
    driver->_last = time_present(at);
    return true;
}

static bool owns_operation(const odrive_can_driver_t *driver, odrive_can_driver_operation_id_t id)
{
    return driver->_active && driver->_incarnation == id.incarnation &&
           driver->_report.id.sequence == id.sequence;
}

static odrive_can_driver_result_t check_attempt(const odrive_can_driver_t *driver,
                                                odrive_can_driver_tx_attempt_t attempt)
{
    if (!owns_operation(driver, attempt.id) || !driver->_attempt_outstanding ||
        driver->_attempt_sequence != attempt.sequence) {
        return ODRIVE_CAN_DRIVER_STALE_ATTEMPT;
    }
    return driver->_unknown_acknowledged ? ODRIVE_CAN_DRIVER_REVOKED : ODRIVE_CAN_DRIVER_OK;
}

static void finish(odrive_can_driver_operation_report_t *report,
                   odrive_can_driver_operation_state_t state, uint64_t event, uint64_t processed)
{
    report->state = state;
    report->terminal_at = time_present(event);
    report->processed_at = processed;
    report->response_present = false;
}

odrive_can_driver_result_t odrive_can_driver_init(odrive_can_driver_t *driver, uint32_t node,
                                                  uint64_t incarnation)
{
    if (driver == NULL || node > 63 || incarnation == 0) {
        return ODRIVE_CAN_DRIVER_INVALID_ARGUMENT;
    }
    *driver = (odrive_can_driver_t){._incarnation = incarnation, ._node = node};
    return ODRIVE_CAN_DRIVER_OK;
}

uint32_t odrive_can_driver_node(const odrive_can_driver_t *driver)
{
    return driver == NULL ? UINT32_MAX : driver->_node;
}

const odrive_can_response_cache_t *odrive_can_driver_cache(const odrive_can_driver_t *driver)
{
    return driver == NULL ? NULL : &driver->_cache;
}

static odrive_can_driver_result_t prepare(odrive_can_driver_t *driver,
                                         const odrive_can_message_t *message, uint64_t now,
                                         uint64_t deadline, odrive_can_driver_send_permit_t *permit,
                                         odrive_can_codec_error_t *codec_error)
{
    if (driver == NULL || permit == NULL) {
        return ODRIVE_CAN_DRIVER_INVALID_ARGUMENT;
    }
    if (driver->_active) {
        return ODRIVE_CAN_DRIVER_BUSY;
    }
    if (driver->_last.present && now < driver->_last.us) {
        return ODRIVE_CAN_DRIVER_CLOCK_ROLLBACK;
    }
    if (now >= deadline) {
        return ODRIVE_CAN_DRIVER_DEADLINE_ELAPSED;
    }
    odrive_can_frame_t frame;
    odrive_can_codec_error_t error;
    if (odrive_can_v051_encode(driver->_node, message, &frame, &error) != ODRIVE_CAN_CODEC_OK) {
        if (codec_error != NULL) {
            *codec_error = error;
        }
        return ODRIVE_CAN_DRIVER_ENCODE_ERROR;
    }
    if (driver->_next_operation == UINT64_MAX) {
        return ODRIVE_CAN_DRIVER_ID_EXHAUSTED;
    }

    const odrive_can_driver_operation_id_t id = {driver->_incarnation, driver->_next_operation++};
    driver->_last = time_present(now);
    driver->_active = true;
    driver->_frame = frame;
    driver->_attempt_sequence = 0;
    driver->_attempt_outstanding = false;
    driver->_unknown_acknowledged = false;
    driver->_candidate_sequence = 0;
    driver->_candidate_present = false;
    driver->_report = (odrive_can_driver_operation_report_t){
        .id = id,
        .deadline = deadline,
        .prepared_at = now,
        .processed_at = now,
        .state = ODRIVE_CAN_DRIVER_STATE_PREPARED,
    };
    if (message->kind == ODRIVE_CAN_MESSAGE_COMMAND) {
        driver->_report.kind = ODRIVE_CAN_DRIVER_OPERATION_COMMAND;
        driver->_report.value.command = message->value.command;
    } else {
        driver->_report.kind = ODRIVE_CAN_DRIVER_OPERATION_QUERY;
        driver->_report.value.query = message->value.query;
    }
    *permit = (odrive_can_driver_send_permit_t){.id = id, .sequence = 0};
    return ODRIVE_CAN_DRIVER_OK;
}

odrive_can_driver_result_t odrive_can_driver_prepare_command(
    odrive_can_driver_t *driver, const odrive_can_command_t *command, uint64_t now, uint64_t deadline,
    odrive_can_driver_send_permit_t *permit, odrive_can_codec_error_t *codec_error)
{
    if (command == NULL) {
        return ODRIVE_CAN_DRIVER_INVALID_ARGUMENT;
    }
    const odrive_can_message_t message = {.kind = ODRIVE_CAN_MESSAGE_COMMAND, .value.command = *command};
    return prepare(driver, &message, now, deadline, permit, codec_error);
}

odrive_can_driver_result_t odrive_can_driver_prepare_query(
    odrive_can_driver_t *driver, odrive_can_query_t query, uint64_t now, uint64_t deadline,
    odrive_can_driver_send_permit_t *permit, odrive_can_codec_error_t *codec_error)
{
    const odrive_can_message_t message = {.kind = ODRIVE_CAN_MESSAGE_REQUEST, .value.query = query};
    return prepare(driver, &message, now, deadline, permit, codec_error);
}

odrive_can_driver_result_t odrive_can_driver_begin_send(
    odrive_can_driver_t *driver, odrive_can_driver_send_permit_t permit, uint64_t at,
    odrive_can_driver_tx_attempt_t *attempt)
{
    if (driver == NULL || attempt == NULL) {
        return ODRIVE_CAN_DRIVER_INVALID_ARGUMENT;
    }
    if (!owns_operation(driver, permit.id) || driver->_attempt_sequence != permit.sequence) {
        return ODRIVE_CAN_DRIVER_STALE_PERMIT;
    }
    odrive_can_driver_operation_report_t *const report = &driver->_report;
    if (report->state != ODRIVE_CAN_DRIVER_STATE_PREPARED) {
        return ODRIVE_CAN_DRIVER_NOT_PREPARED;
    }
    if (!advance_time(driver, at)) {
        finish(report, ODRIVE_CAN_DRIVER_STATE_FAILED, driver->_last.us, driver->_last.us);
        return ODRIVE_CAN_DRIVER_CLOCK_ROLLBACK;
    }
    if (at >= report->deadline) {
        finish(report, ODRIVE_CAN_DRIVER_STATE_TIMED_OUT, report->deadline, at);
        return ODRIVE_CAN_DRIVER_DEADLINE_ELAPSED;
    }
    if (driver->_attempt_sequence == UINT64_MAX) {
        finish(report, ODRIVE_CAN_DRIVER_STATE_FAILED, at, at);
        return ODRIVE_CAN_DRIVER_STALE_PERMIT;
    }

    /* 开始尝试后推进序号，旧凭证的所有副本随之失效。 */
    ++driver->_attempt_sequence;
    driver->_attempt_outstanding = true;
    driver->_candidate_present = false;
    report->state = ODRIVE_CAN_DRIVER_STATE_DISPATCHING;
    report->dispatching_at = time_present(at);
    report->processed_at = at;
    *attempt = (odrive_can_driver_tx_attempt_t){.id = permit.id, .sequence = driver->_attempt_sequence};
    return ODRIVE_CAN_DRIVER_OK;
}

odrive_can_driver_result_t odrive_can_driver_authorize_tx(
    odrive_can_driver_t *driver, odrive_can_driver_tx_attempt_t attempt, uint64_t at,
    odrive_can_frame_t *frame)
{
    if (driver == NULL || frame == NULL) {
        return ODRIVE_CAN_DRIVER_INVALID_ARGUMENT;
    }
    const odrive_can_driver_result_t status = check_attempt(driver, attempt);
    if (status != ODRIVE_CAN_DRIVER_OK) {
        return status;
    }
    odrive_can_driver_operation_report_t *const report = &driver->_report;
    if (!advance_time(driver, at)) {
        if (report->state == ODRIVE_CAN_DRIVER_STATE_DISPATCHING) {
            finish(report, ODRIVE_CAN_DRIVER_STATE_UNKNOWN, driver->_last.us, driver->_last.us);
        }
        return ODRIVE_CAN_DRIVER_CLOCK_ROLLBACK;
    }
    if (report->state == ODRIVE_CAN_DRIVER_STATE_DISPATCHING && at >= report->deadline) {
        finish(report, ODRIVE_CAN_DRIVER_STATE_UNKNOWN, at, at);
    }
    report->processed_at = at;
    if (report->state != ODRIVE_CAN_DRIVER_STATE_DISPATCHING) {
        return ODRIVE_CAN_DRIVER_REVOKED;
    }
    *frame = driver->_frame;
    return ODRIVE_CAN_DRIVER_OK;
}

odrive_can_driver_result_t odrive_can_driver_finish_tx(
    odrive_can_driver_t *driver, odrive_can_driver_tx_attempt_t attempt,
    odrive_can_driver_tx_outcome_t outcome, uint64_t event_at, uint64_t processed_at,
    odrive_can_driver_tx_completion_t *completion)
{
    if (driver == NULL || completion == NULL ||
        (outcome != ODRIVE_CAN_DRIVER_TX_SUBMITTED && outcome != ODRIVE_CAN_DRIVER_TX_WOULD_BLOCK &&
         outcome != ODRIVE_CAN_DRIVER_TX_NOT_SUBMITTED)) {
        return ODRIVE_CAN_DRIVER_INVALID_ARGUMENT;
    }
    const odrive_can_driver_result_t status = check_attempt(driver, attempt);
    if (status != ODRIVE_CAN_DRIVER_OK) {
        return status;
    }

    /* 撤销后续发送仍允许回填，回填时消费这次资格。 */
    driver->_attempt_outstanding = false;
    const bool rollback = !advance_time(driver, processed_at);
    const uint64_t processed = driver->_last.us;
    odrive_can_driver_operation_report_t *const report = &driver->_report;
    const bool invalid = event_at < report->dispatching_at.us || event_at > processed_at;
    const bool submitted = outcome == ODRIVE_CAN_DRIVER_TX_SUBMITTED;
    const bool after_deadline = event_at >= report->deadline;
    report->tx_event_at = time_present(event_at);
    report->processed_at = processed;
    if (submitted) {
        report->submitted_at = time_present(event_at);
    }

    if (submitted && (invalid || rollback || after_deadline)) {
        finish(report, ODRIVE_CAN_DRIVER_STATE_UNKNOWN, processed, processed);
    } else if (submitted && report->kind == ODRIVE_CAN_DRIVER_OPERATION_COMMAND) {
        finish(report, ODRIVE_CAN_DRIVER_STATE_SUBMITTED, event_at, processed);
    } else if (report->cancel_requested_at.present) {
        finish(report, ODRIVE_CAN_DRIVER_STATE_CANCELLED, report->cancel_requested_at.us, processed);
    } else if (processed >= report->deadline) {
        finish(report, ODRIVE_CAN_DRIVER_STATE_TIMED_OUT, report->deadline, processed);
    } else if (submitted) {
        report->state = ODRIVE_CAN_DRIVER_STATE_SUBMITTED;
        report->terminal_at = (odrive_can_driver_optional_time_t){0};
    } else if (invalid || rollback || outcome == ODRIVE_CAN_DRIVER_TX_NOT_SUBMITTED) {
        finish(report, ODRIVE_CAN_DRIVER_STATE_FAILED, event_at, processed);
    } else {
        report->state = ODRIVE_CAN_DRIVER_STATE_PREPARED;
        report->terminal_at = (odrive_can_driver_optional_time_t){0};
        driver->_candidate_present = false;
    }

    if (rollback) {
        return ODRIVE_CAN_DRIVER_CLOCK_ROLLBACK;
    }
    if (invalid) {
        return ODRIVE_CAN_DRIVER_INVALID_EVENT_TIME;
    }
    if (submitted && after_deadline) {
        return ODRIVE_CAN_DRIVER_EVENT_AFTER_DEADLINE;
    }

    odrive_can_driver_tx_completion_t result = {.kind = ODRIVE_CAN_DRIVER_COMPLETION_FAILED};
    if (submitted) {
        result.kind = ODRIVE_CAN_DRIVER_COMPLETION_SUBMITTED;
    } else if (report->state == ODRIVE_CAN_DRIVER_STATE_PREPARED) {
        result.kind = ODRIVE_CAN_DRIVER_COMPLETION_RETRY;
        result.retry = (odrive_can_driver_send_permit_t){.id = attempt.id, .sequence = attempt.sequence};
    }
    *completion = result;
    return ODRIVE_CAN_DRIVER_OK;
}

odrive_can_driver_result_t odrive_can_driver_cancel_unsubmitted(
    odrive_can_driver_t *driver, odrive_can_driver_tx_attempt_t attempt, uint64_t event_at,
    uint64_t processed_at)
{
    if (driver == NULL) {
        return ODRIVE_CAN_DRIVER_INVALID_ARGUMENT;
    }
    const odrive_can_driver_result_t status = check_attempt(driver, attempt);
    if (status != ODRIVE_CAN_DRIVER_OK) {
        return status;
    }

    driver->_attempt_outstanding = false;
    const bool rollback = !advance_time(driver, processed_at);
    const uint64_t processed = driver->_last.us;
    odrive_can_driver_operation_report_t *const report = &driver->_report;
    report->tx_event_at = time_present(event_at);
    if (report->cancel_requested_at.present) {
        finish(report, ODRIVE_CAN_DRIVER_STATE_CANCELLED, report->cancel_requested_at.us, processed);
    } else if (processed >= report->deadline) {
        finish(report, ODRIVE_CAN_DRIVER_STATE_TIMED_OUT, report->deadline, processed);
    } else {
        finish(report, ODRIVE_CAN_DRIVER_STATE_CANCELLED, event_at, processed);
    }

    if (rollback) {
        return ODRIVE_CAN_DRIVER_CLOCK_ROLLBACK;
    }
    if (event_at < report->dispatching_at.us || event_at > processed_at) {
        return ODRIVE_CAN_DRIVER_INVALID_EVENT_TIME;
    }
    return ODRIVE_CAN_DRIVER_OK;
}

odrive_can_driver_result_t odrive_can_driver_abandon_attempt(
    odrive_can_driver_t *driver, odrive_can_driver_tx_attempt_t attempt, uint64_t at)
{
    if (driver == NULL) {
        return ODRIVE_CAN_DRIVER_INVALID_ARGUMENT;
    }
    const odrive_can_driver_result_t status = check_attempt(driver, attempt);
    if (status != ODRIVE_CAN_DRIVER_OK) {
        return status;
    }
    driver->_attempt_outstanding = false;
    const bool rollback = !advance_time(driver, at);
    finish(&driver->_report, ODRIVE_CAN_DRIVER_STATE_UNKNOWN, driver->_last.us, driver->_last.us);
    return rollback ? ODRIVE_CAN_DRIVER_CLOCK_ROLLBACK : ODRIVE_CAN_DRIVER_OK;
}

odrive_can_driver_result_t odrive_can_driver_cancel(
    odrive_can_driver_t *driver, odrive_can_driver_operation_id_t id, uint64_t at)
{
    if (driver == NULL) {
        return ODRIVE_CAN_DRIVER_INVALID_ARGUMENT;
    }
    if (!owns_operation(driver, id)) {
        return ODRIVE_CAN_DRIVER_STALE_OPERATION;
    }
    if (!advance_time(driver, at)) {
        return ODRIVE_CAN_DRIVER_CLOCK_ROLLBACK;
    }

    odrive_can_driver_operation_report_t *const report = &driver->_report;
    if (report->state == ODRIVE_CAN_DRIVER_STATE_PREPARED ||
        (report->state == ODRIVE_CAN_DRIVER_STATE_SUBMITTED &&
         report->kind == ODRIVE_CAN_DRIVER_OPERATION_QUERY)) {
        report->cancel_requested_at = time_present(at);
        finish(report, ODRIVE_CAN_DRIVER_STATE_CANCELLED, at, at);
    } else if (report->state == ODRIVE_CAN_DRIVER_STATE_DISPATCHING ||
               report->state == ODRIVE_CAN_DRIVER_STATE_UNKNOWN) {
        if (!report->cancel_requested_at.present) {
            report->cancel_requested_at = time_present(at);
        }
        finish(report, ODRIVE_CAN_DRIVER_STATE_UNKNOWN, at, at);
    }
    return ODRIVE_CAN_DRIVER_OK;
}

odrive_can_driver_result_t odrive_can_driver_tick(odrive_can_driver_t *driver, uint64_t at)
{
    if (driver == NULL) {
        return ODRIVE_CAN_DRIVER_INVALID_ARGUMENT;
    }
    if (!advance_time(driver, at)) {
        return ODRIVE_CAN_DRIVER_CLOCK_ROLLBACK;
    }
    if (!driver->_active) {
        return ODRIVE_CAN_DRIVER_OK;
    }

    odrive_can_driver_operation_report_t *const report = &driver->_report;
    report->processed_at = at;
    if (at >= report->deadline) {
        if (report->state == ODRIVE_CAN_DRIVER_STATE_PREPARED ||
            (report->state == ODRIVE_CAN_DRIVER_STATE_SUBMITTED &&
             report->kind == ODRIVE_CAN_DRIVER_OPERATION_QUERY)) {
            finish(report, ODRIVE_CAN_DRIVER_STATE_TIMED_OUT, report->deadline, at);
        } else if (report->state == ODRIVE_CAN_DRIVER_STATE_DISPATCHING) {
            finish(report, ODRIVE_CAN_DRIVER_STATE_UNKNOWN, at, at);
        }
    }
    return ODRIVE_CAN_DRIVER_OK;
}

static bool response_matches(odrive_can_query_t query, odrive_can_response_kind_t response)
{
    switch (query) {
    case ODRIVE_CAN_QUERY_MOTOR_ERROR:
        return response == ODRIVE_CAN_RESPONSE_MOTOR_ERROR;
    case ODRIVE_CAN_QUERY_ENCODER_ERROR:
        return response == ODRIVE_CAN_RESPONSE_ENCODER_ERROR;
    case ODRIVE_CAN_QUERY_SENSORLESS_ERROR:
        return response == ODRIVE_CAN_RESPONSE_SENSORLESS_ERROR;
    case ODRIVE_CAN_QUERY_ENCODER_ESTIMATES:
        return response == ODRIVE_CAN_RESPONSE_ENCODER_ESTIMATES;
    case ODRIVE_CAN_QUERY_ENCODER_COUNT:
        return response == ODRIVE_CAN_RESPONSE_ENCODER_COUNT;
    case ODRIVE_CAN_QUERY_IQ:
        return response == ODRIVE_CAN_RESPONSE_IQ;
    case ODRIVE_CAN_QUERY_SENSORLESS_ESTIMATES:
        return response == ODRIVE_CAN_RESPONSE_SENSORLESS_ESTIMATES;
    case ODRIVE_CAN_QUERY_VBUS_VOLTAGE:
        return response == ODRIVE_CAN_RESPONSE_VBUS_VOLTAGE;
    }
    return false;
}

static void record_candidate(odrive_can_driver_t *driver, const odrive_can_response_t *response,
                             uint64_t received_at)
{
    const odrive_can_driver_operation_report_t *const report = &driver->_report;
    if (!driver->_active || report->kind != ODRIVE_CAN_DRIVER_OPERATION_QUERY ||
        !response_matches(report->value.query, response->kind) ||
        (report->state != ODRIVE_CAN_DRIVER_STATE_DISPATCHING &&
         report->state != ODRIVE_CAN_DRIVER_STATE_UNKNOWN &&
         report->state != ODRIVE_CAN_DRIVER_STATE_SUBMITTED &&
         report->state != ODRIVE_CAN_DRIVER_STATE_TIMED_OUT) ||
        !report->dispatching_at.present || received_at < report->dispatching_at.us ||
        received_at >= report->deadline ||
        (driver->_candidate_present && received_at < driver->_candidate.received_at_us) ||
        driver->_candidate_sequence == UINT64_MAX) {
        return;
    }

    ++driver->_candidate_sequence;
    driver->_candidate_present = true;
    driver->_candidate = (odrive_can_cached_response_t){.response = *response, .received_at_us = received_at};
}

odrive_can_driver_result_t odrive_can_driver_ingest(
    odrive_can_driver_t *driver, const odrive_can_frame_view_t *frame, uint64_t received_at,
    odrive_can_message_t *message, odrive_can_codec_error_t *codec_error)
{
    if (driver == NULL || frame == NULL || message == NULL) {
        return ODRIVE_CAN_DRIVER_INVALID_ARGUMENT;
    }
    const odrive_can_codec_status_t status = odrive_can_v051_decode(driver->_node, frame, message, codec_error);
    if (status == ODRIVE_CAN_CODEC_UNRELATED) {
        return ODRIVE_CAN_DRIVER_UNRELATED;
    }
    if (status != ODRIVE_CAN_CODEC_OK) {
        return ODRIVE_CAN_DRIVER_DECODE_ERROR;
    }
    if (message->kind == ODRIVE_CAN_MESSAGE_RESPONSE) {
        odrive_can_cache_record(&driver->_cache, &message->value.response, received_at);
        record_candidate(driver, &message->value.response, received_at);
    }
    return ODRIVE_CAN_DRIVER_OK;
}

static bool candidate_eligible(const odrive_can_driver_t *driver)
{
    const odrive_can_driver_operation_report_t *const report = &driver->_report;
    return driver->_candidate_present &&
           (report->state == ODRIVE_CAN_DRIVER_STATE_SUBMITTED ||
            report->state == ODRIVE_CAN_DRIVER_STATE_TIMED_OUT) &&
           !report->cancel_requested_at.present && report->submitted_at.present &&
           driver->_candidate.received_at_us > report->submitted_at.us &&
           driver->_candidate.received_at_us < report->deadline;
}

odrive_can_driver_result_t odrive_can_driver_pending_response(
    const odrive_can_driver_t *driver, odrive_can_driver_operation_id_t id,
    odrive_can_driver_response_candidate_t *candidate, odrive_can_cached_response_t *response)
{
    if (driver == NULL || candidate == NULL || response == NULL) {
        return ODRIVE_CAN_DRIVER_INVALID_ARGUMENT;
    }
    if (!owns_operation(driver, id)) {
        return ODRIVE_CAN_DRIVER_STALE_OPERATION;
    }
    if (!candidate_eligible(driver)) {
        return ODRIVE_CAN_DRIVER_NO_RESPONSE;
    }
    *candidate = (odrive_can_driver_response_candidate_t){.id = id, .sequence = driver->_candidate_sequence};
    *response = driver->_candidate;
    return ODRIVE_CAN_DRIVER_OK;
}

enum candidate_decision {
    ACCEPT_CANDIDATE,
    IGNORE_CANDIDATE,
    REJECT_CANDIDATE,
};

static odrive_can_driver_result_t decide(odrive_can_driver_t *driver,
                                        odrive_can_driver_response_candidate_t candidate,
                                        uint64_t at, enum candidate_decision decision)
{
    if (driver == NULL) {
        return ODRIVE_CAN_DRIVER_INVALID_ARGUMENT;
    }
    if (!owns_operation(driver, candidate.id) || candidate.sequence != driver->_candidate_sequence ||
        !candidate_eligible(driver)) {
        return ODRIVE_CAN_DRIVER_STALE_OPERATION;
    }
    if (driver->_candidate.received_at_us > at) {
        return ODRIVE_CAN_DRIVER_FUTURE_RESPONSE;
    }
    if (!advance_time(driver, at)) {
        return ODRIVE_CAN_DRIVER_CLOCK_ROLLBACK;
    }

    odrive_can_driver_operation_report_t *const report = &driver->_report;
    if (decision == IGNORE_CANDIDATE) {
        report->processed_at = at;
    } else {
        const odrive_can_driver_operation_state_t state = decision == ACCEPT_CANDIDATE
            ? ODRIVE_CAN_DRIVER_STATE_OBSERVED : ODRIVE_CAN_DRIVER_STATE_REJECTED;
        finish(report, state, driver->_candidate.received_at_us, at);
        report->response_present = true;
        report->response = driver->_candidate;
    }
    driver->_candidate_present = false;
    return ODRIVE_CAN_DRIVER_OK;
}

odrive_can_driver_result_t odrive_can_driver_accept_response(
    odrive_can_driver_t *driver, odrive_can_driver_response_candidate_t candidate, uint64_t at)
{
    return decide(driver, candidate, at, ACCEPT_CANDIDATE);
}

odrive_can_driver_result_t odrive_can_driver_ignore_response(
    odrive_can_driver_t *driver, odrive_can_driver_response_candidate_t candidate, uint64_t at)
{
    return decide(driver, candidate, at, IGNORE_CANDIDATE);
}

odrive_can_driver_result_t odrive_can_driver_reject_response(
    odrive_can_driver_t *driver, odrive_can_driver_response_candidate_t candidate, uint64_t at)
{
    return decide(driver, candidate, at, REJECT_CANDIDATE);
}

odrive_can_driver_result_t odrive_can_driver_report(
    const odrive_can_driver_t *driver, odrive_can_driver_operation_id_t id,
    odrive_can_driver_operation_report_t *report)
{
    if (driver == NULL || report == NULL) {
        return ODRIVE_CAN_DRIVER_INVALID_ARGUMENT;
    }
    if (!owns_operation(driver, id)) {
        return ODRIVE_CAN_DRIVER_STALE_OPERATION;
    }
    *report = driver->_report;
    return ODRIVE_CAN_DRIVER_OK;
}

odrive_can_driver_result_t odrive_can_driver_take_report(
    odrive_can_driver_t *driver, odrive_can_driver_operation_id_t id,
    odrive_can_driver_operation_report_t *report)
{
    if (driver == NULL || report == NULL) {
        return ODRIVE_CAN_DRIVER_INVALID_ARGUMENT;
    }
    if (!owns_operation(driver, id)) {
        return ODRIVE_CAN_DRIVER_STALE_OPERATION;
    }
    const odrive_can_driver_operation_state_t state = driver->_report.state;
    if (state == ODRIVE_CAN_DRIVER_STATE_DISPATCHING ||
        (state == ODRIVE_CAN_DRIVER_STATE_UNKNOWN && !driver->_unknown_acknowledged)) {
        return ODRIVE_CAN_DRIVER_UNKNOWN_PENDING;
    }
    if (state == ODRIVE_CAN_DRIVER_STATE_PREPARED ||
        (state == ODRIVE_CAN_DRIVER_STATE_SUBMITTED && driver->_report.kind == ODRIVE_CAN_DRIVER_OPERATION_QUERY)) {
        return ODRIVE_CAN_DRIVER_PENDING;
    }

    *report = driver->_report;
    driver->_active = false;
    return ODRIVE_CAN_DRIVER_OK;
}

odrive_can_driver_result_t odrive_can_driver_acknowledge_unknown(
    odrive_can_driver_t *driver, odrive_can_driver_operation_id_t id,
    odrive_can_driver_operation_report_t *report)
{
    if (driver == NULL || report == NULL) {
        return ODRIVE_CAN_DRIVER_INVALID_ARGUMENT;
    }
    if (!owns_operation(driver, id)) {
        return ODRIVE_CAN_DRIVER_STALE_OPERATION;
    }
    if (driver->_report.state != ODRIVE_CAN_DRIVER_STATE_UNKNOWN) {
        return ODRIVE_CAN_DRIVER_PENDING;
    }

    driver->_unknown_acknowledged = true;
    *report = driver->_report;
    return ODRIVE_CAN_DRIVER_OK;
}
