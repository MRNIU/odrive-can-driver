/* Copyright The odrive-can-driver Contributors */

/**
 * @file
 * @brief 在模拟 CAN 总线上调度周期命令并处理反馈。
 *
 * 应用独占发送端，按节点分发收到的帧，并决定何时发送零速指令和 Idle 状态请求。
 * 进入下一个控制周期时，编码器查询仍在等待回复。
 * @note 在支持的固件上，发给目标节点的 RTR 帧也会喂设备看门狗。
 */

#include <odrive_can_driver/response_cache.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

/** @brief 模拟收发使用的节点号和 CAN 标识符。 */
enum {
    NODE_ID = 1,                  /**< 应用选定的轴节点。 */
    ENCODER_ESTIMATES_ID = 0x029, /**< 编码器估算值查询与回复的标识符。 */
    SET_INPUT_VEL_ID = 0x02d,     /**< 设置输入速度的标识符。 */
    SET_AXIS_STATE_ID = 0x027,    /**< 请求切换轴状态的标识符。 */
};

/** @brief 应用持有的模拟发送端所记录的帧。 */
typedef struct {
    odrive_can_frame_t sent[3]; /**< 本地模拟发送端已接收的帧。 */
    size_t count;               /**< 已记录的帧数。 */
} mock_bus_t;

/**
 * @brief 编码消息并提交至应用唯一的发送端。
 * @param[in,out] bus 由应用持有的模拟 CAN 发送端。
 * @param[in] message 发给所配置节点的命令或查询。
 * @return 编码成功且模拟发送端已在本地接收该帧时返回 true。
 */
static bool send_message(mock_bus_t *bus, const odrive_can_message_t *message)
{
    if (bus->count == sizeof bus->sent / sizeof bus->sent[0]) {
        return false;
    }

    odrive_can_frame_t frame;
    if (odrive_can_v051_encode(NODE_ID, message, &frame, NULL) != ODRIVE_CAN_CODEC_OK) {
        return false;
    }

    bus->sent[bus->count++] = frame;
    return true;
}

/**
 * @brief 解码接收帧，并将目标节点的回复记入反馈缓存。
 * @param[in,out] cache 应用为该节点配置的缓存。
 * @param[in] frame 收到的帧视图。原始帧仍由调用方持有。
 * @param[in] received_at_us 接收事件时间，使用应用的单调微秒时钟。
 * @return 解码成功或编解码器忽略该帧时返回 true，协议解码出错时返回 false。
 */
static bool receive_frame(odrive_can_response_cache_t *cache, const odrive_can_frame_view_t *frame,
                          uint64_t received_at_us)
{
    odrive_can_message_t message;
    const odrive_can_codec_status_t result =
        odrive_can_v051_decode(NODE_ID, frame, &message, NULL);
    if (result == ODRIVE_CAN_CODEC_UNRELATED) {
        /* 共享总线的持有方可将原始帧继续分发给其他接收方。 */
        return true;
    }
    if (result != ODRIVE_CAN_CODEC_OK) {
        return false;
    }
    if (message.kind == ODRIVE_CAN_MESSAGE_RESPONSE) {
        odrive_can_cache_record(cache, &message.value.response, received_at_us);
    }
    return true;
}

/**
 * @brief 在上一条编码器查询等待回复时运行周期命令。
 * @return 模拟收发产生预期的帧和缓存条目时返回 0。
 */
int main(void)
{
    mock_bus_t bus = {0};
    odrive_can_response_cache_t cache;
    odrive_can_cache_init(&cache);

    const odrive_can_message_t query = {
        .kind = ODRIVE_CAN_MESSAGE_REQUEST,
        .value.query = ODRIVE_CAN_QUERY_ENCODER_ESTIMATES,
    };
    const odrive_can_message_t zero_velocity = {
        .kind = ODRIVE_CAN_MESSAGE_COMMAND,
        .value.command = {
            .kind = ODRIVE_CAN_COMMAND_SET_INPUT_VEL,
            .value.set_input_vel = {.velocity = 0.0f, .torque_ff = 0.0f},
        },
    };
    const odrive_can_message_t idle = {
        .kind = ODRIVE_CAN_MESSAGE_COMMAND,
        .value.command = {
            .kind = ODRIVE_CAN_COMMAND_SET_AXIS_REQUESTED_STATE,
            .value.set_axis_requested_state = {.state = ODRIVE_CAN_AXIS_STATE_IDLE},
        },
    };

    if (!send_message(&bus, &query)) {
        return EXIT_FAILURE;
    }

    /* 在下一轮 1 kHz 控制周期模拟停机决策，此时还没有收到查询回复。 */
    odrive_can_cached_response_t sample;
    if (odrive_can_cache_get(&cache, ODRIVE_CAN_RESPONSE_ENCODER_ESTIMATES, &sample) ||
        !send_message(&bus, &zero_velocity) || !send_message(&bus, &idle)) {
        return EXIT_FAILURE;
    }

    if (bus.sent[0].id != ENCODER_ESTIMATES_ID || bus.sent[0].kind != ODRIVE_CAN_FRAME_REMOTE ||
        bus.sent[0].dlc != 8 || bus.sent[0].data_len != 0 ||
        bus.sent[1].id != SET_INPUT_VEL_ID || bus.sent[1].kind != ODRIVE_CAN_FRAME_DATA ||
        bus.sent[1].dlc != 8 ||
        bus.sent[2].id != SET_AXIS_STATE_ID || bus.sent[2].kind != ODRIVE_CAN_FRAME_DATA ||
        bus.sent[2].dlc != 4 || bus.sent[2].data[0] != ODRIVE_CAN_AXIS_STATE_IDLE) {
        return EXIT_FAILURE;
    }

    const uint8_t data[] = {0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0xc0};
    odrive_can_frame_view_t rx = {
        .id_kind = ODRIVE_CAN_ID_STANDARD,
        .id = ENCODER_ESTIMATES_ID + 0x20, /* 来自节点 2 的编码器估算值。 */
        .kind = ODRIVE_CAN_FRAME_DATA,
        .data = data,
        .data_len = sizeof data,
        .dlc = sizeof data,
    };
    if (!receive_frame(&cache, &rx, 1100) ||
        odrive_can_cache_get(&cache, ODRIVE_CAN_RESPONSE_ENCODER_ESTIMATES, &sample)) {
        return EXIT_FAILURE;
    }

    rx.id = ENCODER_ESTIMATES_ID;
    if (!receive_frame(&cache, &rx, 1500) ||
        !odrive_can_cache_get(&cache, ODRIVE_CAN_RESPONSE_ENCODER_ESTIMATES, &sample) ||
        sample.received_at_us != 1500 || sample.response.value.encoder_estimates.velocity != -2.0f) {
        return EXIT_FAILURE;
    }

    puts("Mock bus accepted zero velocity and Idle while encoder feedback was pending.");
    return EXIT_SUCCESS;
}
