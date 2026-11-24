/**
 * @file
 * @brief 一个 CANSimple 节点的定长解码反馈缓存。
 * @copyright The odrive-can-driver Contributors
 */

#ifndef ODRIVE_CAN_DRIVER_RESPONSE_CACHE_H
#define ODRIVE_CAN_DRIVER_RESPONSE_CACHE_H

#include <stdbool.h>
#include <stdint.h>

#include <odrive_can_driver/fw_v0_5_1.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 已解码的设备回复及其接收事件时间。 */
typedef struct {
    odrive_can_response_t response; /**< 保留原始字段值的回复。 */
    uint64_t received_at_us; /**< 应用单调时钟域中的接收事件时间，单位为微秒。 */
} odrive_can_cached_response_t;

/**
 * @brief 由调用方持有，按回复类别存储最新观测。
 *
 * 每个节点各用一个缓存，并串行访问。使用 odrive_can_cache_init() 或零初始化，
 * 再通过下列缓存函数读写观测。
 */
typedef struct {
    /** 由库管理的回复条目存储。 */
    odrive_can_cached_response_t entries[ODRIVE_CAN_RESPONSE_VBUS_VOLTAGE - ODRIVE_CAN_RESPONSE_HEARTBEAT + 1];
    uint16_t present; /**< 由库管理，标记已有观测的回复类别。 */
} odrive_can_response_cache_t;

/**
 * @brief 初始化缓存并清除已有观测。
 * @param[out] cache 待初始化的缓存存储。传入 NULL 时不做处理。
 */
void odrive_can_cache_init(odrive_can_response_cache_t *cache);

/**
 * @brief 按回复类别记录解码结果，保留接收时间最新的观测。
 *
 * 接收时间相同时，以最后一次写入为准。保留未知状态、错误位以及非有限浮点数的位表示。
 * 应用自行判断该观测是否适用于控制。
 *
 * @param[in,out] cache 该回复所属节点已初始化的缓存。
 * @param[in] response 已经过节点过滤的解码回复。
 * @param[in] received_at_us 应用单调时钟域中的接收事件时间，单位为微秒。
 * @note 指针为 NULL 或回复类别无效时，缓存保持原值。
 */
void odrive_can_cache_record(
    odrive_can_response_cache_t *cache, const odrive_can_response_t *response, uint64_t received_at_us);

/**
 * @brief 复制指定回复类别的最新观测。
 * @param[in] cache 已初始化的缓存。
 * @param[in] kind 要查询的回复类别。
 * @param[out] out 观测结果的写入位置。
 * @return 成功写入观测时返回 true。缺少观测、类别无效或指针为 NULL 时返回 false，
 *         不写入输出。
 */
bool odrive_can_cache_get(
    const odrive_can_response_cache_t *cache, odrive_can_response_kind_t kind, odrive_can_cached_response_t *out);

/**
 * @brief 计算最新心跳观测距当前时刻的时间间隔。
 * @param[in] cache 已初始化的缓存。
 * @param[in] now_us 当前时刻，使用记录接收事件的同一单调时钟域，单位为微秒。
 * @param[out] out_age_us 时间间隔的写入位置，单位为微秒。
 * @return 成功写入时间间隔时返回 true。缺少心跳、接收时刻晚于当前时刻或指针为 NULL 时
 *         返回 false，不写入输出。
 */
bool odrive_can_cache_heartbeat_age(
    const odrive_can_response_cache_t *cache, uint64_t now_us, uint64_t *out_age_us);

/**
 * @brief 检查距最近心跳的时间是否在限值以内，包含限值本身。
 *
 * 此检查依据应用提供的时间戳。调用方负责判断硬件来源是否连续，以及观测是否适用于控制。
 *
 * @param[in] cache 已初始化的缓存。传入 NULL 时视为缺少观测。
 * @param[in] now_us 当前时刻，使用记录接收事件的同一单调时钟域，单位为微秒。
 * @param[in] max_age_us 允许的最大时间间隔，单位为微秒。
 * @return 可计算的时间间隔小于或等于 max_age_us 时返回 true，其余情况返回 false。
 */
bool odrive_can_cache_heartbeat_is_fresh(
    const odrive_can_response_cache_t *cache, uint64_t now_us, uint64_t max_age_us);

#ifdef __cplusplus
}
#endif

#endif
