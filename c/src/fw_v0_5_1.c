/* Copyright The odrive-can-protocol Contributors */
/* Copyright The odrive-can-driver Contributors */

/**
 * @file
 * @brief ODrive fw-v0.5.1 经典 CANSimple 帧的编解码实现。
 */

#include "odrive_can_driver/fw_v0_5_1.h"

#include <float.h>
#include <limits.h>
#include <stdbool.h>
#include <string.h>

_Static_assert(CHAR_BIT == 8, "CANSimple requires 8-bit bytes");
_Static_assert(sizeof(float) == sizeof(uint32_t) && FLT_RADIX == 2 &&
               FLT_MANT_DIG == 24 && FLT_MIN_EXP == -125 && FLT_MAX_EXP == 128,
               "CANSimple requires IEEE 754 binary32 float");

enum {
    NODE_SHIFT = 5,
    COMMAND_MASK = 0x1f,
    MAX_NODE_ID = 63,
    MAX_STANDARD_ID = 0x07ff,
    CLASSIC_CAN_MAX_LENGTH = 8,
};

enum {
    WIRE_HEARTBEAT = 0x01,
    WIRE_ESTOP = 0x02,
    WIRE_MOTOR_ERROR = 0x03,
    WIRE_ENCODER_ERROR = 0x04,
    WIRE_SENSORLESS_ERROR = 0x05,
    WIRE_SET_AXIS_NODE_ID = 0x06,
    WIRE_SET_AXIS_REQUESTED_STATE = 0x07,
    WIRE_SET_AXIS_STARTUP_CONFIG = 0x08,
    WIRE_ENCODER_ESTIMATES = 0x09,
    WIRE_ENCODER_COUNT = 0x0a,
    WIRE_SET_CONTROLLER_MODES = 0x0b,
    WIRE_SET_INPUT_POS = 0x0c,
    WIRE_SET_INPUT_VEL = 0x0d,
    WIRE_SET_INPUT_TORQUE = 0x0e,
    WIRE_SET_VELOCITY_LIMIT = 0x0f,
    WIRE_START_ANTICOGGING = 0x10,
    WIRE_SET_TRAJ_VEL_LIMIT = 0x11,
    WIRE_SET_TRAJ_ACCEL_LIMITS = 0x12,
    WIRE_SET_TRAJ_INERTIA = 0x13,
    WIRE_IQ = 0x14,
    WIRE_SENSORLESS_ESTIMATES = 0x15,
    WIRE_REBOOT = 0x16,
    WIRE_VBUS_VOLTAGE = 0x17,
    WIRE_CLEAR_ERRORS = 0x18,
};

static const uint8_t wire_data_length[WIRE_CLEAR_ERRORS + 1] = {
    [WIRE_HEARTBEAT] = 8,
    [WIRE_MOTOR_ERROR] = 8,
    [WIRE_ENCODER_ERROR] = 8,
    [WIRE_SENSORLESS_ERROR] = 8,
    [WIRE_SET_AXIS_NODE_ID] = 4,
    [WIRE_SET_AXIS_REQUESTED_STATE] = 4,
    [WIRE_ENCODER_ESTIMATES] = 8,
    [WIRE_ENCODER_COUNT] = 8,
    [WIRE_SET_CONTROLLER_MODES] = 8,
    [WIRE_SET_INPUT_POS] = 8,
    [WIRE_SET_INPUT_VEL] = 8,
    [WIRE_SET_INPUT_TORQUE] = 4,
    [WIRE_SET_VELOCITY_LIMIT] = 4,
    [WIRE_SET_TRAJ_VEL_LIMIT] = 4,
    [WIRE_SET_TRAJ_ACCEL_LIMITS] = 8,
    [WIRE_SET_TRAJ_INERTIA] = 4,
    [WIRE_IQ] = 8,
    [WIRE_SENSORLESS_ESTIMATES] = 8,
    [WIRE_VBUS_VOLTAGE] = 8,
};

static const uint8_t query_wire_id[] = {
    [ODRIVE_CAN_QUERY_MOTOR_ERROR] = WIRE_MOTOR_ERROR,
    [ODRIVE_CAN_QUERY_ENCODER_ERROR] = WIRE_ENCODER_ERROR,
    [ODRIVE_CAN_QUERY_SENSORLESS_ERROR] = WIRE_SENSORLESS_ERROR,
    [ODRIVE_CAN_QUERY_ENCODER_ESTIMATES] = WIRE_ENCODER_ESTIMATES,
    [ODRIVE_CAN_QUERY_ENCODER_COUNT] = WIRE_ENCODER_COUNT,
    [ODRIVE_CAN_QUERY_IQ] = WIRE_IQ,
    [ODRIVE_CAN_QUERY_SENSORLESS_ESTIMATES] = WIRE_SENSORLESS_ESTIMATES,
    [ODRIVE_CAN_QUERY_VBUS_VOLTAGE] = WIRE_VBUS_VOLTAGE,
};

static void put_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

static void put_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static uint32_t get_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static int32_t get_i32(const uint8_t *data)
{
    uint32_t bits = get_u32(data);
    if (bits <= INT32_MAX) {
        return (int32_t)bits;
    }
    return -1 - (int32_t)(UINT32_MAX - bits);
}

static int16_t get_i16(const uint8_t *data)
{
    uint16_t bits = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    if (bits <= INT16_MAX) {
        return (int16_t)bits;
    }
    return (int16_t)(-1 - (int16_t)(UINT16_MAX - bits));
}

static void get_f32(float *value, const uint8_t *data)
{
    uint32_t bits = get_u32(data);
    memcpy(value, &bits, sizeof bits);
}

static uint32_t f32_bits(const float *value)
{
    uint32_t bits;
    memcpy(&bits, value, sizeof bits);
    return bits;
}

static void put_f32(uint8_t *data, const float *value)
{
    put_u32(data, f32_bits(value));
}

static odrive_can_codec_status_t put_command_f32(
    uint8_t *data,
    const float *value,
    odrive_can_float_field_t field,
    odrive_can_codec_error_t *error)
{
    uint32_t bits = f32_bits(value);
    if ((bits & UINT32_C(0x7f800000)) == UINT32_C(0x7f800000)) {
        if (error != NULL) {
            error->field = field;
        }
        return ODRIVE_CAN_CODEC_NON_FINITE;
    }
    put_u32(data, bits);
    return ODRIVE_CAN_CODEC_OK;
}

static odrive_can_codec_status_t invalid_node_id(uint32_t node_id, odrive_can_codec_error_t *error)
{
    if (error != NULL) {
        error->value = node_id;
    }
    return ODRIVE_CAN_CODEC_INVALID_NODE_ID;
}

static odrive_can_codec_status_t encode_command(
    const odrive_can_command_t *command,
    uint8_t *data,
    uint8_t *wire_id,
    odrive_can_codec_error_t *error)
{
    odrive_can_codec_status_t status;
    switch (command->kind) {
    case ODRIVE_CAN_COMMAND_ESTOP:
        *wire_id = WIRE_ESTOP;
        break;
    case ODRIVE_CAN_COMMAND_SET_AXIS_NODE_ID:
        if (command->value.set_axis_node_id.node_id > MAX_NODE_ID) {
            return invalid_node_id(command->value.set_axis_node_id.node_id, error);
        }
        *wire_id = WIRE_SET_AXIS_NODE_ID;
        put_u32(data, command->value.set_axis_node_id.node_id);
        break;
    case ODRIVE_CAN_COMMAND_SET_AXIS_REQUESTED_STATE:
        *wire_id = WIRE_SET_AXIS_REQUESTED_STATE;
        put_u32(data, command->value.set_axis_requested_state.state);
        break;
    case ODRIVE_CAN_COMMAND_SET_CONTROLLER_MODES:
        *wire_id = WIRE_SET_CONTROLLER_MODES;
        put_u32(data, (uint32_t)command->value.set_controller_modes.control_mode);
        put_u32(data + 4, (uint32_t)command->value.set_controller_modes.input_mode);
        break;
    case ODRIVE_CAN_COMMAND_SET_INPUT_POS:
        *wire_id = WIRE_SET_INPUT_POS;
        status = put_command_f32(data, &command->value.set_input_pos.position,
                                 ODRIVE_CAN_FLOAT_FIELD_POSITION, error);
        if (status != ODRIVE_CAN_CODEC_OK) {
            return status;
        }
        put_u16(data + 4, (uint16_t)command->value.set_input_pos.velocity_ff);
        put_u16(data + 6, (uint16_t)command->value.set_input_pos.torque_ff);
        break;
    case ODRIVE_CAN_COMMAND_SET_INPUT_VEL:
        *wire_id = WIRE_SET_INPUT_VEL;
        status = put_command_f32(data, &command->value.set_input_vel.velocity,
                                 ODRIVE_CAN_FLOAT_FIELD_VELOCITY, error);
        if (status != ODRIVE_CAN_CODEC_OK) {
            return status;
        }
        return put_command_f32(data + 4, &command->value.set_input_vel.torque_ff,
                               ODRIVE_CAN_FLOAT_FIELD_TORQUE_FF, error);
    case ODRIVE_CAN_COMMAND_SET_INPUT_TORQUE:
        *wire_id = WIRE_SET_INPUT_TORQUE;
        return put_command_f32(data, &command->value.set_input_torque.torque,
                               ODRIVE_CAN_FLOAT_FIELD_TORQUE, error);
    case ODRIVE_CAN_COMMAND_SET_VELOCITY_LIMIT:
        *wire_id = WIRE_SET_VELOCITY_LIMIT;
        return put_command_f32(data, &command->value.set_velocity_limit.velocity,
                               ODRIVE_CAN_FLOAT_FIELD_VELOCITY, error);
    case ODRIVE_CAN_COMMAND_START_ANTICOGGING:
        *wire_id = WIRE_START_ANTICOGGING;
        break;
    case ODRIVE_CAN_COMMAND_SET_TRAJ_VEL_LIMIT:
        *wire_id = WIRE_SET_TRAJ_VEL_LIMIT;
        return put_command_f32(data, &command->value.set_traj_vel_limit.velocity,
                               ODRIVE_CAN_FLOAT_FIELD_VELOCITY, error);
    case ODRIVE_CAN_COMMAND_SET_TRAJ_ACCEL_LIMITS:
        *wire_id = WIRE_SET_TRAJ_ACCEL_LIMITS;
        status = put_command_f32(data, &command->value.set_traj_accel_limits.acceleration,
                                 ODRIVE_CAN_FLOAT_FIELD_ACCELERATION, error);
        if (status != ODRIVE_CAN_CODEC_OK) {
            return status;
        }
        return put_command_f32(data + 4, &command->value.set_traj_accel_limits.deceleration,
                               ODRIVE_CAN_FLOAT_FIELD_DECELERATION, error);
    case ODRIVE_CAN_COMMAND_SET_TRAJ_INERTIA:
        *wire_id = WIRE_SET_TRAJ_INERTIA;
        return put_command_f32(data, &command->value.set_traj_inertia.inertia,
                               ODRIVE_CAN_FLOAT_FIELD_INERTIA, error);
    case ODRIVE_CAN_COMMAND_REBOOT:
        *wire_id = WIRE_REBOOT;
        break;
    case ODRIVE_CAN_COMMAND_CLEAR_ERRORS:
        *wire_id = WIRE_CLEAR_ERRORS;
        break;
    default:
        return ODRIVE_CAN_CODEC_INVALID_ARGUMENT;
    }
    return ODRIVE_CAN_CODEC_OK;
}

static uint8_t encode_response(const odrive_can_response_t *response, uint8_t *data)
{
    switch (response->kind) {
    case ODRIVE_CAN_RESPONSE_HEARTBEAT:
        put_u32(data, response->value.heartbeat.axis_error);
        put_u32(data + 4, response->value.heartbeat.axis_state);
        return WIRE_HEARTBEAT;
    case ODRIVE_CAN_RESPONSE_MOTOR_ERROR:
        put_u32(data, response->value.motor_error);
        return WIRE_MOTOR_ERROR;
    case ODRIVE_CAN_RESPONSE_ENCODER_ERROR:
        put_u32(data, response->value.encoder_error);
        return WIRE_ENCODER_ERROR;
    case ODRIVE_CAN_RESPONSE_SENSORLESS_ERROR:
        put_u32(data, response->value.sensorless_error);
        return WIRE_SENSORLESS_ERROR;
    case ODRIVE_CAN_RESPONSE_ENCODER_ESTIMATES:
        put_f32(data, &response->value.encoder_estimates.position);
        put_f32(data + 4, &response->value.encoder_estimates.velocity);
        return WIRE_ENCODER_ESTIMATES;
    case ODRIVE_CAN_RESPONSE_ENCODER_COUNT:
        put_u32(data, (uint32_t)response->value.encoder_count.shadow_count);
        put_u32(data + 4, (uint32_t)response->value.encoder_count.count_in_cpr);
        return WIRE_ENCODER_COUNT;
    case ODRIVE_CAN_RESPONSE_IQ:
        put_f32(data, &response->value.iq.setpoint);
        put_f32(data + 4, &response->value.iq.measured);
        return WIRE_IQ;
    case ODRIVE_CAN_RESPONSE_SENSORLESS_ESTIMATES:
        put_f32(data, &response->value.sensorless_estimates.position);
        put_f32(data + 4, &response->value.sensorless_estimates.velocity);
        return WIRE_SENSORLESS_ESTIMATES;
    case ODRIVE_CAN_RESPONSE_VBUS_VOLTAGE:
        put_f32(data, &response->value.vbus_voltage);
        return WIRE_VBUS_VOLTAGE;
    default:
        return 0;
    }
}

static odrive_can_codec_status_t encode_message(
    uint32_t node_id,
    const odrive_can_message_t *message,
    odrive_can_frame_t *frame,
    odrive_can_codec_error_t *error)
{
    if (message == NULL || frame == NULL) {
        return ODRIVE_CAN_CODEC_INVALID_ARGUMENT;
    }
    if (node_id > MAX_NODE_ID) {
        return invalid_node_id(node_id, error);
    }

    odrive_can_frame_t encoded = {.kind = ODRIVE_CAN_FRAME_DATA};
    uint8_t wire_id = 0;
    switch (message->kind) {
    case ODRIVE_CAN_MESSAGE_COMMAND: {
        odrive_can_codec_status_t status =
            encode_command(&message->value.command, encoded.data, &wire_id, error);
        if (status != ODRIVE_CAN_CODEC_OK) {
            return status;
        }
        break;
    }
    case ODRIVE_CAN_MESSAGE_REQUEST: {
        size_t query = (size_t)message->value.query;
        if (query == 0 || query >= sizeof query_wire_id) {
            return ODRIVE_CAN_CODEC_INVALID_ARGUMENT;
        }
        wire_id = query_wire_id[query];
        encoded.kind = ODRIVE_CAN_FRAME_REMOTE;
        break;
    }
    case ODRIVE_CAN_MESSAGE_RESPONSE:
        wire_id = encode_response(&message->value.response, encoded.data);
        if (wire_id == 0) {
            return ODRIVE_CAN_CODEC_INVALID_ARGUMENT;
        }
        break;
    default:
        return ODRIVE_CAN_CODEC_INVALID_ARGUMENT;
    }
    encoded.id = (uint16_t)((node_id << NODE_SHIFT) | wire_id);
    if (encoded.kind == ODRIVE_CAN_FRAME_REMOTE) {
        encoded.dlc = CLASSIC_CAN_MAX_LENGTH;
    } else {
        encoded.data_len = wire_data_length[wire_id];
        encoded.dlc = encoded.data_len;
    }
    *frame = encoded;
    return ODRIVE_CAN_CODEC_OK;
}

odrive_can_codec_status_t odrive_can_v051_encode(
    uint32_t node_id,
    const odrive_can_message_t *message,
    odrive_can_frame_t *frame,
    odrive_can_codec_error_t *error)
{
    if (error != NULL) {
        *error = (odrive_can_codec_error_t){0};
    }
    odrive_can_codec_status_t status = encode_message(node_id, message, frame, error);
    if (error != NULL) {
        error->status = status;
    }
    return status;
}

static bool decode_response(uint8_t wire_id, const uint8_t *data, odrive_can_response_t *response)
{
    switch (wire_id) {
    case WIRE_HEARTBEAT:
        response->kind = ODRIVE_CAN_RESPONSE_HEARTBEAT;
        response->value.heartbeat.axis_error = get_u32(data);
        response->value.heartbeat.axis_state = get_u32(data + 4);
        return true;
    case WIRE_MOTOR_ERROR:
        response->kind = ODRIVE_CAN_RESPONSE_MOTOR_ERROR;
        response->value.motor_error = get_u32(data);
        return true;
    case WIRE_ENCODER_ERROR:
        response->kind = ODRIVE_CAN_RESPONSE_ENCODER_ERROR;
        response->value.encoder_error = get_u32(data);
        return true;
    case WIRE_SENSORLESS_ERROR:
        response->kind = ODRIVE_CAN_RESPONSE_SENSORLESS_ERROR;
        response->value.sensorless_error = get_u32(data);
        return true;
    case WIRE_ENCODER_ESTIMATES:
        response->kind = ODRIVE_CAN_RESPONSE_ENCODER_ESTIMATES;
        get_f32(&response->value.encoder_estimates.position, data);
        get_f32(&response->value.encoder_estimates.velocity, data + 4);
        return true;
    case WIRE_ENCODER_COUNT:
        response->kind = ODRIVE_CAN_RESPONSE_ENCODER_COUNT;
        response->value.encoder_count.shadow_count = get_i32(data);
        response->value.encoder_count.count_in_cpr = get_i32(data + 4);
        return true;
    case WIRE_IQ:
        response->kind = ODRIVE_CAN_RESPONSE_IQ;
        get_f32(&response->value.iq.setpoint, data);
        get_f32(&response->value.iq.measured, data + 4);
        return true;
    case WIRE_SENSORLESS_ESTIMATES:
        response->kind = ODRIVE_CAN_RESPONSE_SENSORLESS_ESTIMATES;
        get_f32(&response->value.sensorless_estimates.position, data);
        get_f32(&response->value.sensorless_estimates.velocity, data + 4);
        return true;
    case WIRE_VBUS_VOLTAGE:
        response->kind = ODRIVE_CAN_RESPONSE_VBUS_VOLTAGE;
        get_f32(&response->value.vbus_voltage, data);
        return true;
    default:
        return false;
    }
}

static odrive_can_codec_status_t decode_command(
    uint8_t wire_id,
    const uint8_t *data,
    odrive_can_command_t *command,
    odrive_can_codec_error_t *error)
{
    switch (wire_id) {
    case WIRE_ESTOP:
        command->kind = ODRIVE_CAN_COMMAND_ESTOP;
        break;
    case WIRE_SET_AXIS_NODE_ID: {
        uint32_t node_id = get_u32(data);
        if (node_id > MAX_NODE_ID) {
            return invalid_node_id(node_id, error);
        }
        command->kind = ODRIVE_CAN_COMMAND_SET_AXIS_NODE_ID;
        command->value.set_axis_node_id.node_id = node_id;
        break;
    }
    case WIRE_SET_AXIS_REQUESTED_STATE:
        command->kind = ODRIVE_CAN_COMMAND_SET_AXIS_REQUESTED_STATE;
        command->value.set_axis_requested_state.state = get_u32(data);
        break;
    case WIRE_SET_CONTROLLER_MODES:
        command->kind = ODRIVE_CAN_COMMAND_SET_CONTROLLER_MODES;
        command->value.set_controller_modes.control_mode = get_i32(data);
        command->value.set_controller_modes.input_mode = get_i32(data + 4);
        break;
    case WIRE_SET_INPUT_POS:
        command->kind = ODRIVE_CAN_COMMAND_SET_INPUT_POS;
        get_f32(&command->value.set_input_pos.position, data);
        command->value.set_input_pos.velocity_ff = get_i16(data + 4);
        command->value.set_input_pos.torque_ff = get_i16(data + 6);
        break;
    case WIRE_SET_INPUT_VEL:
        command->kind = ODRIVE_CAN_COMMAND_SET_INPUT_VEL;
        get_f32(&command->value.set_input_vel.velocity, data);
        get_f32(&command->value.set_input_vel.torque_ff, data + 4);
        break;
    case WIRE_SET_INPUT_TORQUE:
        command->kind = ODRIVE_CAN_COMMAND_SET_INPUT_TORQUE;
        get_f32(&command->value.set_input_torque.torque, data);
        break;
    case WIRE_SET_VELOCITY_LIMIT:
        command->kind = ODRIVE_CAN_COMMAND_SET_VELOCITY_LIMIT;
        get_f32(&command->value.set_velocity_limit.velocity, data);
        break;
    case WIRE_START_ANTICOGGING:
        command->kind = ODRIVE_CAN_COMMAND_START_ANTICOGGING;
        break;
    case WIRE_SET_TRAJ_VEL_LIMIT:
        command->kind = ODRIVE_CAN_COMMAND_SET_TRAJ_VEL_LIMIT;
        get_f32(&command->value.set_traj_vel_limit.velocity, data);
        break;
    case WIRE_SET_TRAJ_ACCEL_LIMITS:
        command->kind = ODRIVE_CAN_COMMAND_SET_TRAJ_ACCEL_LIMITS;
        get_f32(&command->value.set_traj_accel_limits.acceleration, data);
        get_f32(&command->value.set_traj_accel_limits.deceleration, data + 4);
        break;
    case WIRE_SET_TRAJ_INERTIA:
        command->kind = ODRIVE_CAN_COMMAND_SET_TRAJ_INERTIA;
        get_f32(&command->value.set_traj_inertia.inertia, data);
        break;
    case WIRE_REBOOT:
        command->kind = ODRIVE_CAN_COMMAND_REBOOT;
        break;
    case WIRE_CLEAR_ERRORS:
        command->kind = ODRIVE_CAN_COMMAND_CLEAR_ERRORS;
        break;
    default:
        return ODRIVE_CAN_CODEC_UNRELATED;
    }
    return ODRIVE_CAN_CODEC_OK;
}

static odrive_can_codec_status_t decode_remote(
    uint8_t wire_id,
    uint8_t dlc,
    odrive_can_message_t *message,
    odrive_can_codec_error_t *error)
{
    if (dlc > CLASSIC_CAN_MAX_LENGTH) {
        if (error != NULL) {
            error->value = dlc;
        }
        return ODRIVE_CAN_CODEC_INVALID_REMOTE_DLC;
    }
    for (size_t query = 1; query < sizeof query_wire_id; ++query) {
        if (query_wire_id[query] == wire_id) {
            *message = (odrive_can_message_t){
                .kind = ODRIVE_CAN_MESSAGE_REQUEST,
                .value.query = (odrive_can_query_t)query,
            };
            return ODRIVE_CAN_CODEC_OK;
        }
    }
    if (error != NULL) {
        error->command_id = wire_id;
    }
    return ODRIVE_CAN_CODEC_INVALID_FRAME_KIND;
}

static odrive_can_codec_status_t decode_frame(
    uint32_t node_id,
    const odrive_can_frame_view_t *frame,
    odrive_can_message_t *message,
    odrive_can_codec_error_t *error)
{
    if (frame == NULL || message == NULL) {
        return ODRIVE_CAN_CODEC_INVALID_ARGUMENT;
    }
    if (node_id > MAX_NODE_ID) {
        return invalid_node_id(node_id, error);
    }
    if (frame->id_kind == ODRIVE_CAN_ID_EXTENDED) {
        return ODRIVE_CAN_CODEC_UNSUPPORTED_EXTENDED_ID;
    }
    if (frame->id_kind != ODRIVE_CAN_ID_STANDARD) {
        return ODRIVE_CAN_CODEC_INVALID_ARGUMENT;
    }
    if (frame->id > MAX_STANDARD_ID) {
        if (error != NULL) {
            error->value = frame->id;
        }
        return ODRIVE_CAN_CODEC_INVALID_STANDARD_ID;
    }
    if ((frame->id >> NODE_SHIFT) != node_id) {
        return ODRIVE_CAN_CODEC_UNRELATED;
    }
    uint8_t wire_id = (uint8_t)(frame->id & COMMAND_MASK);
    if (wire_id == WIRE_SET_AXIS_STARTUP_CONFIG) {
        if (error != NULL) {
            error->command_id = wire_id;
        }
        return ODRIVE_CAN_CODEC_UNSUPPORTED_COMMAND;
    }
    if (wire_id < WIRE_HEARTBEAT || wire_id > WIRE_CLEAR_ERRORS) {
        return ODRIVE_CAN_CODEC_UNRELATED;
    }
    switch (frame->kind) {
    case ODRIVE_CAN_FRAME_FD:
        return ODRIVE_CAN_CODEC_UNSUPPORTED_CAN_FD;
    case ODRIVE_CAN_FRAME_REMOTE:
        return decode_remote(wire_id, frame->dlc, message, error);
    case ODRIVE_CAN_FRAME_DATA:
        break;
    default:
        return ODRIVE_CAN_CODEC_INVALID_ARGUMENT;
    }

    size_t expected_length = wire_data_length[wire_id];
    if (frame->data_len != expected_length) {
        if (error != NULL) {
            error->command_id = wire_id;
            error->expected_length = expected_length;
            error->actual_length = frame->data_len;
        }
        return ODRIVE_CAN_CODEC_INVALID_LENGTH;
    }
    const uint8_t *data = expected_length == 0 ? NULL : frame->data;
    if (data == NULL && expected_length > 0) {
        return ODRIVE_CAN_CODEC_INVALID_ARGUMENT;
    }

    odrive_can_message_t decoded = {.kind = ODRIVE_CAN_MESSAGE_RESPONSE};
    if (!decode_response(wire_id, data, &decoded.value.response)) {
        decoded.kind = ODRIVE_CAN_MESSAGE_COMMAND;
        odrive_can_codec_status_t status =
            decode_command(wire_id, data, &decoded.value.command, error);
        if (status != ODRIVE_CAN_CODEC_OK) {
            return status;
        }
    }
    *message = decoded;
    return ODRIVE_CAN_CODEC_OK;
}

odrive_can_codec_status_t odrive_can_v051_decode(
    uint32_t node_id,
    const odrive_can_frame_view_t *frame,
    odrive_can_message_t *message,
    odrive_can_codec_error_t *error)
{
    if (error != NULL) {
        *error = (odrive_can_codec_error_t){0};
    }
    odrive_can_codec_status_t status = decode_frame(node_id, frame, message, error);
    if (error != NULL) {
        error->status = status;
    }
    return status;
}

odrive_can_frame_view_t odrive_can_v051_frame_view(const odrive_can_frame_t *frame)
{
    if (frame == NULL) {
        return (odrive_can_frame_view_t){0};
    }
    return (odrive_can_frame_view_t){
        .id_kind = ODRIVE_CAN_ID_STANDARD,
        .id = frame->id,
        .kind = frame->kind,
        .data = frame->data,
        .data_len = frame->data_len,
        .dlc = frame->dlc,
    };
}
