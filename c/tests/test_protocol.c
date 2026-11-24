/* Copyright The odrive-can-driver Contributors */

/**
 * @file
 * @brief 用与 Rust 共用的独立向量验证 C 版 CANSimple 编解码器。
 */

#include "odrive_can_driver/fw_v0_5_1.h"

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** 单条协议测试向量。 */
typedef struct {
    size_t line;                  /**< 测试向量文件中的行号。 */
    char mode[32];                /**< 用例执行模式。 */
    uint32_t node;                /**< 传给编解码器的节点号。 */
    odrive_can_frame_view_t frame; /**< 接收输入帧或预期编码帧。 */
    uint8_t data[8];               /**< 帧数据缓冲区。 */
    char tag[32];                 /**< 消息名或预期错误名。 */
    uint32_t fields[3];           /**< 消息字段的原始位模式或预期错误详情。 */
    size_t field_count;           /**< 字段数量。 */
} vector_t;

/** 第一个断言失败时退出，Release 构建中也执行断言。 */
#define CHECK(vector, condition)                                                              \
    do {                                                                                      \
        if (!(condition)) {                                                                   \
            fprintf(stderr, "protocol vector line %zu: %s\n", (vector)->line, #condition);    \
            exit(EXIT_FAILURE);                                                               \
        }                                                                                     \
    } while (false)

/** 解析测试向量中的无符号值，拒绝超出 uint32_t 范围的输入。 */
static uint32_t unsigned_number(const vector_t *vector, const char *text, int base)
{
    char *end;
    CHECK(vector, text[0] != '-');
    if (base == 0) {
        base = strncmp(text, "0x", 2) == 0 ? 16 : 10;
    }
    errno = 0;
    uintmax_t value = strtoumax(text, &end, base);
    CHECK(vector, errno == 0 && end != text && *end == '\0' && value <= UINT32_MAX);
    return (uint32_t)value;
}

/** 解析消息字段，有符号负整数转换成相应的 32 位位模式。 */
static uint32_t field_bits(const vector_t *vector, const char *text)
{
    if (text[0] != '-') {
        return unsigned_number(vector, text, 0);
    }
    char *end;
    errno = 0;
    intmax_t value = strtoimax(text, &end, 10);
    CHECK(vector, errno == 0 && end != text && *end == '\0' && value >= INT32_MIN);
    return (uint32_t)value;
}

/** 分别解析测试模式、帧字段、线上有效载荷和预期结果。 */
static void read_vector(vector_t *vector, const char *line)
{
    char node[16];
    char id[16];
    char kind[16];
    char dlc[8];
    char data[17];
    char fields[64];
    char extra;
    int columns = sscanf(line, "%31s %15s %15s %15s %7s %16s %31s %63s %c",
                         vector->mode, node, id, kind, dlc, data, vector->tag, fields, &extra);
    CHECK(vector, columns == 8);
    vector->node = unsigned_number(vector, node, 0);

    if (strcmp(fields, "-") != 0) {
        char *field = fields;
        while (true) {
            char *next_field = strchr(field, ',');
            if (next_field != NULL) {
                *next_field = '\0';
            }
            CHECK(vector, vector->field_count < sizeof(vector->fields) / sizeof(vector->fields[0]));
            vector->fields[vector->field_count++] = field_bits(vector, field);
            if (next_field == NULL) {
                break;
            }
            field = next_field + 1;
        }
    }
    if (strncmp(vector->mode, "reject_", 7) == 0) {
        CHECK(vector, strcmp(id, "-") == 0 && strcmp(kind, "-") == 0);
        CHECK(vector, strcmp(dlc, "-") == 0 && strcmp(data, "-") == 0);
        return;
    }

    vector->frame.id_kind = ODRIVE_CAN_ID_STANDARD;
    if (strcmp(kind, "data") == 0 || strcmp(kind, "extended_data") == 0) {
        vector->frame.kind = ODRIVE_CAN_FRAME_DATA;
    } else if (strcmp(kind, "rtr") == 0) {
        vector->frame.kind = ODRIVE_CAN_FRAME_REMOTE;
    } else {
        CHECK(vector, strcmp(kind, "fd") == 0 || strcmp(kind, "extended_fd") == 0);
        vector->frame.kind = ODRIVE_CAN_FRAME_FD;
    }
    if (strcmp(kind, "extended_data") == 0 || strcmp(kind, "extended_fd") == 0) {
        vector->frame.id_kind = ODRIVE_CAN_ID_EXTENDED;
    }
    vector->frame.id = unsigned_number(vector, id, 0);
    CHECK(vector, vector->frame.id_kind == ODRIVE_CAN_ID_EXTENDED || vector->frame.id <= UINT16_MAX);
    uint32_t raw_dlc = unsigned_number(vector, dlc, 0);
    CHECK(vector, raw_dlc <= UINT8_MAX);
    vector->frame.dlc = (uint8_t)raw_dlc;

    if (strcmp(data, "-") != 0) {
        size_t hex_digits = strlen(data);
        CHECK(vector, hex_digits % 2 == 0 && strspn(data, "0123456789abcdefABCDEF") == hex_digits);
        vector->frame.data_len = hex_digits / 2;
        CHECK(vector, vector->frame.data_len <= sizeof(vector->data));
        for (size_t i = 0; i < vector->frame.data_len; ++i) {
            const char byte[] = {data[2 * i], data[2 * i + 1], '\0'};
            vector->data[i] = (uint8_t)unsigned_number(vector, byte, 16);
        }
        vector->frame.data = vector->data;
    }
    if (vector->frame.kind == ODRIVE_CAN_FRAME_DATA) {
        CHECK(vector, vector->frame.data_len == vector->frame.dlc);
    } else if (vector->frame.kind == ODRIVE_CAN_FRAME_REMOTE) {
        CHECK(vector, vector->frame.data_len == 0);
    }
}

/** 测试向量中的消息名与 API 标签之间的映射。 */
typedef struct {
    const char *tag;                     /**< 测试向量中的消息名。 */
    odrive_can_message_kind_t direction; /**< 区分命令、查询和回复。 */
    int kind;                            /**< 对应方向下的消息枚举值。 */
    size_t field_count;                  /**< 消息字段数量。 */
} tag_t;

/**
 * @def COMMAND(name, symbol, count)
 * @brief 构造写命令的消息标签映射。
 * @param[in] name 测试向量的写命令名称标记。
 * @param[in] symbol 命令枚举值的后缀。
 * @param[in] count 写命令的消息字段数量。
 */
/**
 * @def QUERY(name, symbol)
 * @brief 构造 RTR 查询的消息标签映射。查询不含有效载荷字段。
 * @param[in] name 测试向量去掉 query_ 前缀后的查询名称标记。
 * @param[in] symbol 查询枚举值的后缀。
 */
/**
 * @def RESPONSE(name, symbol, count)
 * @brief 构造设备回复的消息标签映射。
 * @param[in] name 测试向量的回复名称标记。
 * @param[in] symbol 回复枚举值的后缀。
 * @param[in] count 回复的消息字段数量。
 */

/** 按测试向量的消息名查找 API 对应的消息标签。 */
static tag_t message_tag(const vector_t *vector)
{
#define COMMAND(name, symbol, count) { #name, ODRIVE_CAN_MESSAGE_COMMAND, ODRIVE_CAN_COMMAND_##symbol, count }
#define QUERY(name, symbol) { "query_" #name, ODRIVE_CAN_MESSAGE_REQUEST, ODRIVE_CAN_QUERY_##symbol, 0 }
#define RESPONSE(name, symbol, count) { #name, ODRIVE_CAN_MESSAGE_RESPONSE, ODRIVE_CAN_RESPONSE_##symbol, count }
    static const tag_t tags[] = {
        COMMAND(estop, ESTOP, 0),
        COMMAND(set_axis_node_id, SET_AXIS_NODE_ID, 1),
        COMMAND(set_axis_requested_state, SET_AXIS_REQUESTED_STATE, 1),
        COMMAND(set_controller_modes, SET_CONTROLLER_MODES, 2),
        COMMAND(set_input_pos, SET_INPUT_POS, 3),
        COMMAND(set_input_vel, SET_INPUT_VEL, 2),
        COMMAND(set_input_torque, SET_INPUT_TORQUE, 1),
        COMMAND(set_velocity_limit, SET_VELOCITY_LIMIT, 1),
        COMMAND(start_anticogging, START_ANTICOGGING, 0),
        COMMAND(set_traj_vel_limit, SET_TRAJ_VEL_LIMIT, 1),
        COMMAND(set_traj_accel_limits, SET_TRAJ_ACCEL_LIMITS, 2),
        COMMAND(set_traj_inertia, SET_TRAJ_INERTIA, 1),
        COMMAND(reboot, REBOOT, 0),
        COMMAND(clear_errors, CLEAR_ERRORS, 0),
        QUERY(motor_error, MOTOR_ERROR),
        QUERY(encoder_error, ENCODER_ERROR),
        QUERY(sensorless_error, SENSORLESS_ERROR),
        QUERY(encoder_estimates, ENCODER_ESTIMATES),
        QUERY(encoder_count, ENCODER_COUNT),
        QUERY(iq, IQ),
        QUERY(sensorless_estimates, SENSORLESS_ESTIMATES),
        QUERY(vbus_voltage, VBUS_VOLTAGE),
        RESPONSE(heartbeat, HEARTBEAT, 2),
        RESPONSE(motor_error, MOTOR_ERROR, 1),
        RESPONSE(encoder_error, ENCODER_ERROR, 1),
        RESPONSE(sensorless_error, SENSORLESS_ERROR, 1),
        RESPONSE(encoder_estimates, ENCODER_ESTIMATES, 2),
        RESPONSE(encoder_count, ENCODER_COUNT, 2),
        RESPONSE(iq, IQ, 2),
        RESPONSE(sensorless_estimates, SENSORLESS_ESTIMATES, 2),
        RESPONSE(vbus_voltage, VBUS_VOLTAGE, 1),
    };
#undef COMMAND
#undef QUERY
#undef RESPONSE
    for (size_t i = 0; i < sizeof(tags) / sizeof(tags[0]); ++i) {
        if (strcmp(vector->tag, tags[i].tag) == 0) {
            CHECK(vector, vector->field_count == tags[i].field_count);
            return tags[i];
        }
    }
    fprintf(stderr, "protocol vector line %zu: unknown message %s\n", vector->line, vector->tag);
    exit(EXIT_FAILURE);
}

/** 按对象字节写入或比较标量字段，保留完整的 binary32 位模式。 */
static void scalar_field(const vector_t *vector, void *member, size_t size, size_t index, bool build)
{
    uint32_t value = vector->fields[index];
    if (size == sizeof(uint16_t)) {
        CHECK(vector, value <= INT16_MAX || value >= UINT32_MAX - INT16_MAX);
    }
    uint16_t short_value = (uint16_t)value;
    const void *expected = size == sizeof(short_value) ? (const void *)&short_value : &value;
    CHECK(vector, size == sizeof(short_value) || size == sizeof(value));
    if (build) {
        memcpy(member, expected, size);
    } else {
        CHECK(vector, memcmp(member, expected, size) == 0);
    }
}

/**
 * @def FIELD(member, index)
 * @brief 依据当前用例模式写入或比较对应字段。
 * @param[in,out] member API 消息字段。
 * @param[in] index 测试向量中的字段下标。
 */

/** 构造 API 输入消息，或逐字段核对实际解码结果。 */
static void message_fields(const vector_t *vector, odrive_can_message_t *message, bool build)
{
    tag_t tag = message_tag(vector);
    if (build) {
        message->kind = tag.direction;
    } else {
        CHECK(vector, message->kind == tag.direction);
    }

#define FIELD(member, index) scalar_field(vector, &(member), sizeof(member), index, build)
    if (tag.direction == ODRIVE_CAN_MESSAGE_COMMAND) {
        odrive_can_command_t *command = &message->value.command;
        if (build) {
            command->kind = (odrive_can_command_kind_t)tag.kind;
        } else {
            CHECK(vector, command->kind == (odrive_can_command_kind_t)tag.kind);
        }
        switch (command->kind) {
        case ODRIVE_CAN_COMMAND_SET_AXIS_NODE_ID:
            FIELD(command->value.set_axis_node_id.node_id, 0);
            break;
        case ODRIVE_CAN_COMMAND_SET_AXIS_REQUESTED_STATE:
            FIELD(command->value.set_axis_requested_state.state, 0);
            break;
        case ODRIVE_CAN_COMMAND_SET_CONTROLLER_MODES:
            FIELD(command->value.set_controller_modes.control_mode, 0);
            FIELD(command->value.set_controller_modes.input_mode, 1);
            break;
        case ODRIVE_CAN_COMMAND_SET_INPUT_POS:
            FIELD(command->value.set_input_pos.position, 0);
            FIELD(command->value.set_input_pos.velocity_ff, 1);
            FIELD(command->value.set_input_pos.torque_ff, 2);
            break;
        case ODRIVE_CAN_COMMAND_SET_INPUT_VEL:
            FIELD(command->value.set_input_vel.velocity, 0);
            FIELD(command->value.set_input_vel.torque_ff, 1);
            break;
        case ODRIVE_CAN_COMMAND_SET_INPUT_TORQUE:
            FIELD(command->value.set_input_torque.torque, 0);
            break;
        case ODRIVE_CAN_COMMAND_SET_VELOCITY_LIMIT:
            FIELD(command->value.set_velocity_limit.velocity, 0);
            break;
        case ODRIVE_CAN_COMMAND_SET_TRAJ_VEL_LIMIT:
            FIELD(command->value.set_traj_vel_limit.velocity, 0);
            break;
        case ODRIVE_CAN_COMMAND_SET_TRAJ_ACCEL_LIMITS:
            FIELD(command->value.set_traj_accel_limits.acceleration, 0);
            FIELD(command->value.set_traj_accel_limits.deceleration, 1);
            break;
        case ODRIVE_CAN_COMMAND_SET_TRAJ_INERTIA:
            FIELD(command->value.set_traj_inertia.inertia, 0);
            break;
        case ODRIVE_CAN_COMMAND_ESTOP:
        case ODRIVE_CAN_COMMAND_START_ANTICOGGING:
        case ODRIVE_CAN_COMMAND_REBOOT:
        case ODRIVE_CAN_COMMAND_CLEAR_ERRORS:
            break;
        }
    } else if (tag.direction == ODRIVE_CAN_MESSAGE_REQUEST) {
        if (build) {
            message->value.query = (odrive_can_query_t)tag.kind;
        } else {
            CHECK(vector, message->value.query == (odrive_can_query_t)tag.kind);
        }
    } else {
        odrive_can_response_t *response = &message->value.response;
        if (build) {
            response->kind = (odrive_can_response_kind_t)tag.kind;
        } else {
            CHECK(vector, response->kind == (odrive_can_response_kind_t)tag.kind);
        }
        switch (response->kind) {
        case ODRIVE_CAN_RESPONSE_HEARTBEAT:
            FIELD(response->value.heartbeat.axis_error, 0);
            FIELD(response->value.heartbeat.axis_state, 1);
            break;
        case ODRIVE_CAN_RESPONSE_MOTOR_ERROR:
            FIELD(response->value.motor_error, 0);
            break;
        case ODRIVE_CAN_RESPONSE_ENCODER_ERROR:
            FIELD(response->value.encoder_error, 0);
            break;
        case ODRIVE_CAN_RESPONSE_SENSORLESS_ERROR:
            FIELD(response->value.sensorless_error, 0);
            break;
        case ODRIVE_CAN_RESPONSE_ENCODER_ESTIMATES:
            FIELD(response->value.encoder_estimates.position, 0);
            FIELD(response->value.encoder_estimates.velocity, 1);
            break;
        case ODRIVE_CAN_RESPONSE_ENCODER_COUNT:
            FIELD(response->value.encoder_count.shadow_count, 0);
            FIELD(response->value.encoder_count.count_in_cpr, 1);
            break;
        case ODRIVE_CAN_RESPONSE_IQ:
            FIELD(response->value.iq.setpoint, 0);
            FIELD(response->value.iq.measured, 1);
            break;
        case ODRIVE_CAN_RESPONSE_SENSORLESS_ESTIMATES:
            FIELD(response->value.sensorless_estimates.position, 0);
            FIELD(response->value.sensorless_estimates.velocity, 1);
            break;
        case ODRIVE_CAN_RESPONSE_VBUS_VOLTAGE:
            FIELD(response->value.vbus_voltage, 0);
            break;
        }
    }
#undef FIELD
}

/** 查找预期的解码结果状态。成功解码消息的向量使用 OK。 */
static odrive_can_codec_status_t expected_decode_status(const char *tag)
{
    static const struct {
        const char *tag;
        odrive_can_codec_status_t status;
    } errors[] = {
        {"unrelated", ODRIVE_CAN_CODEC_UNRELATED},
        {"invalid_standard_id", ODRIVE_CAN_CODEC_INVALID_STANDARD_ID},
        {"unsupported_extended_id", ODRIVE_CAN_CODEC_UNSUPPORTED_EXTENDED_ID},
        {"unsupported_can_fd", ODRIVE_CAN_CODEC_UNSUPPORTED_CAN_FD},
        {"unsupported_command", ODRIVE_CAN_CODEC_UNSUPPORTED_COMMAND},
        {"invalid_frame_kind", ODRIVE_CAN_CODEC_INVALID_FRAME_KIND},
        {"invalid_length", ODRIVE_CAN_CODEC_INVALID_LENGTH},
        {"invalid_remote_dlc", ODRIVE_CAN_CODEC_INVALID_REMOTE_DLC},
        {"invalid_node_id", ODRIVE_CAN_CODEC_INVALID_NODE_ID},
    };
    for (size_t i = 0; i < sizeof(errors) / sizeof(errors[0]); ++i) {
        if (strcmp(tag, errors[i].tag) == 0) {
            return errors[i].status;
        }
    }
    return ODRIVE_CAN_CODEC_OK;
}

/** 检查被拒绝输入的错误详情，包含超出协议范围的原始数值。 */
static void check_decode_error(const vector_t *vector, odrive_can_codec_status_t status,
                               const odrive_can_codec_error_t *error)
{
    switch (status) {
    case ODRIVE_CAN_CODEC_INVALID_STANDARD_ID:
    case ODRIVE_CAN_CODEC_INVALID_REMOTE_DLC:
    case ODRIVE_CAN_CODEC_INVALID_NODE_ID:
        CHECK(vector, vector->field_count == 1);
        CHECK(vector, error->value == vector->fields[0]);
        break;
    case ODRIVE_CAN_CODEC_UNSUPPORTED_COMMAND:
    case ODRIVE_CAN_CODEC_INVALID_FRAME_KIND:
        CHECK(vector, vector->field_count == 1);
        CHECK(vector, error->command_id == vector->fields[0]);
        break;
    case ODRIVE_CAN_CODEC_INVALID_LENGTH:
        CHECK(vector, vector->field_count == 3);
        CHECK(vector, error->command_id == vector->fields[0]);
        CHECK(vector, error->expected_length == vector->fields[1]);
        CHECK(vector, error->actual_length == vector->fields[2]);
        break;
    default:
        CHECK(vector, vector->field_count == 0);
        break;
    }
}

/** 按测试模式查找编码应报错的浮点字段。 */
static odrive_can_float_field_t expected_non_finite_field(const vector_t *vector)
{
    static const struct {
        const char *mode;
        odrive_can_float_field_t field;
    } fields[] = {
        {"reject_position", ODRIVE_CAN_FLOAT_FIELD_POSITION},
        {"reject_velocity", ODRIVE_CAN_FLOAT_FIELD_VELOCITY},
        {"reject_torque_ff", ODRIVE_CAN_FLOAT_FIELD_TORQUE_FF},
        {"reject_torque", ODRIVE_CAN_FLOAT_FIELD_TORQUE},
        {"reject_acceleration", ODRIVE_CAN_FLOAT_FIELD_ACCELERATION},
        {"reject_deceleration", ODRIVE_CAN_FLOAT_FIELD_DECELERATION},
        {"reject_inertia", ODRIVE_CAN_FLOAT_FIELD_INERTIA},
    };
    for (size_t i = 0; i < sizeof(fields) / sizeof(fields[0]); ++i) {
        if (strcmp(vector->mode, fields[i].mode) == 0) {
            return fields[i].field;
        }
    }
    fprintf(stderr, "protocol vector line %zu: unknown mode %s\n", vector->line, vector->mode);
    exit(EXIT_FAILURE);
}

/** 检查编码拒绝的原因，或校验成功编码的帧及其完整数据缓冲区。 */
static void check_encode(const vector_t *vector)
{
    odrive_can_message_t message = {0};
    odrive_can_frame_t frame;
    odrive_can_codec_error_t error;
    memset(&frame, 0xa5, sizeof(frame));
    uint8_t before[sizeof(frame)];
    memcpy(before, &frame, sizeof(frame));
    message_fields(vector, &message, true);
    odrive_can_codec_status_t status = odrive_can_v051_encode(vector->node, &message, &frame, &error);
    CHECK(vector, error.status == status);
    if (status != ODRIVE_CAN_CODEC_OK) {
        CHECK(vector, memcmp(&frame, before, sizeof(frame)) == 0);
    }

    if (strcmp(vector->mode, "reject_invalid_node") == 0) {
        CHECK(vector, status == ODRIVE_CAN_CODEC_INVALID_NODE_ID);
        CHECK(vector, error.value == vector->node);
        return;
    }
    if (strncmp(vector->mode, "reject_", 7) == 0) {
        CHECK(vector, status == ODRIVE_CAN_CODEC_NON_FINITE);
        CHECK(vector, error.field == expected_non_finite_field(vector));
        return;
    }

    CHECK(vector, status == ODRIVE_CAN_CODEC_OK);
    CHECK(vector, frame.id == vector->frame.id);
    CHECK(vector, frame.kind == vector->frame.kind);
    CHECK(vector, frame.dlc == vector->frame.dlc);
    CHECK(vector, frame.data_len == vector->frame.data_len);
    CHECK(vector, memcmp(frame.data, vector->data, sizeof(frame.data)) == 0);
    odrive_can_frame_view_t view = odrive_can_v051_frame_view(&frame);
    CHECK(vector, view.id_kind == ODRIVE_CAN_ID_STANDARD);
    CHECK(vector, view.id == frame.id && view.kind == frame.kind);
    CHECK(vector, view.dlc == frame.dlc && view.data_len == frame.data_len);
    if (view.data_len != 0) {
        CHECK(vector, view.data == frame.data);
    }
}

/** 将固定线上字节交给解码器，校验消息内容或具体失败原因。 */
static void check_decode(const vector_t *vector)
{
    odrive_can_message_t message;
    odrive_can_codec_error_t error;
    memset(&message, 0xa5, sizeof(message));
    uint8_t before[sizeof(message)];
    memcpy(before, &message, sizeof(message));
    odrive_can_codec_status_t expected = expected_decode_status(vector->tag);
    CHECK(vector, expected == ODRIVE_CAN_CODEC_OK || strcmp(vector->mode, "decode") == 0);
    odrive_can_codec_status_t actual = odrive_can_v051_decode(vector->node, &vector->frame, &message, &error);
    CHECK(vector, actual == expected);
    CHECK(vector, error.status == expected);
    if (actual == ODRIVE_CAN_CODEC_OK) {
        message_fields(vector, &message, false);
        if (vector->frame.kind == ODRIVE_CAN_FRAME_REMOTE) {
            odrive_can_frame_view_t input = vector->frame;
            for (uint8_t dlc = 0; dlc <= 8; ++dlc) {
                input.dlc = dlc;
                CHECK(vector, odrive_can_v051_decode(vector->node, &input, &message, NULL) == ODRIVE_CAN_CODEC_OK);
                message_fields(vector, &message, false);
            }
        }
    } else {
        CHECK(vector, memcmp(&message, before, sizeof(message)) == 0);
        check_decode_error(vector, expected, &error);
    }
}

/**
 * @brief 读取 CTest 提供的测试向量文件并运行其中全部用例。
 * @param[in] argc 参数个数，包含程序名。
 * @param[in] argv 参数数组，程序名后的第一项是测试向量文件路径。
 * @return 全部用例通过且文件关闭成功时返回 EXIT_SUCCESS，否则返回 EXIT_FAILURE。
 */
int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: %s protocol-vector-file\n", argv[0]);
        return EXIT_FAILURE;
    }
    FILE *input = fopen(argv[1], "r");
    if (input == NULL) {
        perror(argv[1]);
        return EXIT_FAILURE;
    }
    char line[256];
    size_t line_number = 0;
    size_t case_count = 0;
    while (fgets(line, sizeof(line), input) != NULL) {
        vector_t vector = {.line = ++line_number};
        if (line[0] == '#' || line[0] == '\n') {
            continue;
        }
        ++case_count;
        read_vector(&vector, line);
        if (strncmp(vector.mode, "reject_", 7) == 0) {
            check_encode(&vector);
        } else {
            bool both = strcmp(vector.mode, "both") == 0;
            CHECK(&vector, both || strcmp(vector.mode, "decode") == 0);
            check_decode(&vector);
            if (both) {
                check_encode(&vector);
            }
        }
    }
    if (ferror(input)) {
        perror(argv[1]);
        fclose(input);
        return EXIT_FAILURE;
    }
    if (fclose(input) != 0) {
        perror(argv[1]);
        return EXIT_FAILURE;
    }
    if (case_count == 0) {
        fputs("protocol fixture contains no vectors\n", stderr);
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
