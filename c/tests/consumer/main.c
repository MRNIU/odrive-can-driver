/* Copyright The odrive-can-driver Contributors */

/**
 * @file
 * @brief 在独立的 CMake 工程中编译并链接公开 C API。
 */

#include <odrive_can_driver/driver.h>
#include <odrive_can_driver/fw_v0_5_1.h>
#include <odrive_can_driver/response_cache.h>

/**
 * @brief 比较独立编解码器和 Driver 生成的发送帧。
 * @return 成功时返回 0。失败时返回对应的阶段编号（1 至 3）。
 */
int main(void)
{
    const odrive_can_message_t query = {
        .kind = ODRIVE_CAN_MESSAGE_REQUEST,
        .value.query = ODRIVE_CAN_QUERY_ENCODER_ESTIMATES,
    };
    odrive_can_frame_t protocol_frame;
    if (odrive_can_v051_encode(1, &query, &protocol_frame, NULL) != ODRIVE_CAN_CODEC_OK ||
        protocol_frame.id != 0x029 || protocol_frame.kind != ODRIVE_CAN_FRAME_REMOTE ||
        protocol_frame.dlc != 8 || protocol_frame.data_len != 0) {
        return 1;
    }

    odrive_can_driver_t driver;
    odrive_can_driver_send_permit_t permit;
    odrive_can_driver_tx_attempt_t attempt;
    odrive_can_frame_t driver_frame;
    if (odrive_can_driver_init(&driver, 1, 1) != ODRIVE_CAN_DRIVER_OK ||
        odrive_can_driver_prepare_query(&driver, ODRIVE_CAN_QUERY_ENCODER_ESTIMATES, 0, 10, &permit, NULL) !=
            ODRIVE_CAN_DRIVER_OK ||
        odrive_can_driver_begin_send(&driver, permit, 0, &attempt) != ODRIVE_CAN_DRIVER_OK ||
        odrive_can_driver_authorize_tx(&driver, attempt, 0, &driver_frame) != ODRIVE_CAN_DRIVER_OK) {
        return 2;
    }
    if (driver_frame.id != protocol_frame.id || driver_frame.kind != protocol_frame.kind ||
        driver_frame.dlc != protocol_frame.dlc || driver_frame.data_len != protocol_frame.data_len ||
        odrive_can_cache_heartbeat_is_fresh(odrive_can_driver_cache(&driver), 0, 10)) {
        return 3;
    }
    return 0;
}
