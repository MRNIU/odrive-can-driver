/**
 * @file
 * @brief 验证独立缓存的时间戳和原始反馈语义。
 * @copyright The odrive-can-driver Contributors
 */

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include <odrive_can_driver/response_cache.h>

/** 验证九种回复类别均可独立读取。 */
static void records_each_response_kind(void)
{
    const odrive_can_response_t samples[] = {
        {.kind = ODRIVE_CAN_RESPONSE_HEARTBEAT,
         .value.heartbeat = {.axis_error = UINT32_C(0x80000001), .axis_state = UINT32_C(0x10008)}},
        {.kind = ODRIVE_CAN_RESPONSE_MOTOR_ERROR, .value.motor_error = 1},
        {.kind = ODRIVE_CAN_RESPONSE_ENCODER_ERROR, .value.encoder_error = 2},
        {.kind = ODRIVE_CAN_RESPONSE_SENSORLESS_ERROR, .value.sensorless_error = 3},
        {.kind = ODRIVE_CAN_RESPONSE_ENCODER_ESTIMATES,
         .value.encoder_estimates = {.position = 1.0f, .velocity = -2.0f}},
        {.kind = ODRIVE_CAN_RESPONSE_ENCODER_COUNT,
         .value.encoder_count = {.shadow_count = -3, .count_in_cpr = 4}},
        {.kind = ODRIVE_CAN_RESPONSE_IQ, .value.iq = {.setpoint = 5.0f, .measured = 6.0f}},
        {.kind = ODRIVE_CAN_RESPONSE_SENSORLESS_ESTIMATES,
         .value.sensorless_estimates = {.position = 7.0f, .velocity = 8.0f}},
        {.kind = ODRIVE_CAN_RESPONSE_VBUS_VOLTAGE, .value.vbus_voltage = 48.0f},
    };
    odrive_can_response_cache_t cache;
    odrive_can_cache_init(&cache);
    for (size_t i = 0; i < sizeof(samples) / sizeof(samples[0]); ++i) {
        odrive_can_cache_record(&cache, &samples[i], 20);
    }

    odrive_can_cached_response_t cached;
    for (size_t i = 0; i < sizeof(samples) / sizeof(samples[0]); ++i) {
        assert(odrive_can_cache_get(&cache, samples[i].kind, &cached));
        assert(cached.response.kind == samples[i].kind);
        assert(cached.received_at_us == 20);
    }
    assert(odrive_can_cache_get(&cache, ODRIVE_CAN_RESPONSE_HEARTBEAT, &cached));
    assert(cached.response.value.heartbeat.axis_error == UINT32_C(0x80000001));
    assert(cached.response.value.heartbeat.axis_state == UINT32_C(0x10008));
}

/** 验证时间戳的覆盖规则和非有限数值的保留。 */
static void retains_equal_timestamp_and_discards_older_observation(void)
{
    odrive_can_response_cache_t cache = {0};
    odrive_can_response_t sample = {
        .kind = ODRIVE_CAN_RESPONSE_VBUS_VOLTAGE,
        .value.vbus_voltage = 48.0f,
    };
    odrive_can_cache_record(&cache, &sample, 20);

    const uint32_t nan_bits = UINT32_C(0x7fc050aa);
    memcpy(&sample.value.vbus_voltage, &nan_bits, sizeof(nan_bits));
    odrive_can_cache_record(&cache, &sample, 20);
    sample.value.vbus_voltage = 24.0f;
    odrive_can_cache_record(&cache, &sample, 19);

    odrive_can_cached_response_t cached;
    uint32_t cached_bits;
    assert(odrive_can_cache_get(&cache, ODRIVE_CAN_RESPONSE_VBUS_VOLTAGE, &cached));
    assert(cached.received_at_us == 20);
    memcpy(&cached_bits, &cached.response.value.vbus_voltage, sizeof(cached_bits));
    assert(cached_bits == nan_bits);
}

/** 验证仅在已记录的接收时刻不晚于当前时刻时才能计算时间间隔。 */
static void reports_available_heartbeat_age_with_inclusive_freshness(void)
{
    odrive_can_response_cache_t cache = {0};
    uint64_t age_us = UINT64_MAX;
    assert(!odrive_can_cache_heartbeat_age(&cache, 20, &age_us));
    assert(age_us == UINT64_MAX);

    const odrive_can_response_t heartbeat = {.kind = ODRIVE_CAN_RESPONSE_HEARTBEAT};
    odrive_can_cache_record(&cache, &heartbeat, 20);
    assert(!odrive_can_cache_heartbeat_age(&cache, 19, &age_us));
    assert(age_us == UINT64_MAX);
    assert(!odrive_can_cache_heartbeat_is_fresh(&cache, 19, 5));
    assert(odrive_can_cache_heartbeat_age(&cache, 25, &age_us));
    assert(age_us == 5);
    assert(odrive_can_cache_heartbeat_is_fresh(&cache, 25, 5));
    assert(!odrive_can_cache_heartbeat_is_fresh(&cache, 26, 5));

    odrive_can_cache_init(&cache);
    assert(!odrive_can_cache_heartbeat_age(&cache, 26, &age_us));
}

/** 验证回复类别无效或没有观测时不改写调用方输出。 */
static void unavailable_observations_do_not_overwrite_output(void)
{
    odrive_can_response_cache_t cache = {0};
    const odrive_can_response_t invalid_sample = {
        .kind = (odrive_can_response_kind_t)0,
    };
    odrive_can_cache_record(&cache, &invalid_sample, 20);
    odrive_can_cache_record(&cache, NULL, 20);

    odrive_can_cached_response_t cached = {
        .response.kind = ODRIVE_CAN_RESPONSE_VBUS_VOLTAGE,
        .response.value.vbus_voltage = 48.0f,
        .received_at_us = 15,
    };
    assert(!odrive_can_cache_get(&cache, ODRIVE_CAN_RESPONSE_HEARTBEAT, &cached));
    assert(!odrive_can_cache_get(&cache, invalid_sample.kind, &cached));
    assert(!odrive_can_cache_get(NULL, ODRIVE_CAN_RESPONSE_VBUS_VOLTAGE, &cached));
    assert(cached.response.kind == ODRIVE_CAN_RESPONSE_VBUS_VOLTAGE);
    assert(cached.response.value.vbus_voltage == 48.0f);
    assert(cached.received_at_us == 15);
}

/**
 * @brief 运行全部独立反馈缓存测试。
 * @return 全部检查通过时返回 0。
 */
int main(void)
{
    records_each_response_kind();
    retains_equal_timestamp_and_discards_older_observation();
    reports_available_heartbeat_age_with_inclusive_freshness();
    unavailable_observations_do_not_overwrite_output();
    return 0;
}
