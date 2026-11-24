/**
 * @file
 * @brief 按回复类别存储反馈，并根据接收时间计算心跳时效。
 * @copyright The odrive-can-driver Contributors
 */

#include <odrive_can_driver/response_cache.h>

#include <stddef.h>
#include <string.h>

/**
 * @brief 检查回复类别是否有对应的缓存槽。
 * @param[in] kind 要检查的回复类别。
 * @return 存在对应缓存槽时返回 true。
 */
static bool valid_kind(odrive_can_response_kind_t kind)
{
    return kind >= ODRIVE_CAN_RESPONSE_HEARTBEAT && kind <= ODRIVE_CAN_RESPONSE_VBUS_VOLTAGE;
}

/**
 * @brief 找到回复类别对应的槽。
 * @param[in] kind 已验证的回复类别。
 * @return 对应槽的索引。
 */
static size_t slot_index(odrive_can_response_kind_t kind)
{
    return (size_t)(kind - ODRIVE_CAN_RESPONSE_HEARTBEAT);
}

/**
 * @brief 查询标记为已记录的回复条目。
 * @param[in] cache 已初始化的缓存，或 NULL。
 * @param[in] kind 要查询的回复类别。
 * @return 缓存内的条目指针。没有可用条目时返回 NULL。
 */
static const odrive_can_cached_response_t *lookup(
    const odrive_can_response_cache_t *cache, odrive_can_response_kind_t kind)
{
    if (cache == NULL || !valid_kind(kind)) {
        return NULL;
    }
    const size_t index = slot_index(kind);
    return (cache->present & (1u << index)) != 0u ? &cache->entries[index] : NULL;
}

void odrive_can_cache_init(odrive_can_response_cache_t *cache)
{
    if (cache != NULL) {
        *cache = (odrive_can_response_cache_t){0};
    }
}

void odrive_can_cache_record(
    odrive_can_response_cache_t *cache, const odrive_can_response_t *response, uint64_t received_at_us)
{
    if (cache == NULL || response == NULL || !valid_kind(response->kind)) {
        return;
    }

    const size_t index = slot_index(response->kind);
    const uint16_t mask = (uint16_t)(1u << index);
    odrive_can_cached_response_t *entry = &cache->entries[index];
    if ((cache->present & mask) != 0u && received_at_us < entry->received_at_us) {
        return;
    }

    memmove(&entry->response, response, sizeof(entry->response));
    entry->received_at_us = received_at_us;
    cache->present |= mask;
}

bool odrive_can_cache_get(
    const odrive_can_response_cache_t *cache, odrive_can_response_kind_t kind, odrive_can_cached_response_t *out)
{
    const odrive_can_cached_response_t *entry = lookup(cache, kind);
    if (entry == NULL || out == NULL) {
        return false;
    }
    memmove(out, entry, sizeof(*out));
    return true;
}

bool odrive_can_cache_heartbeat_age(
    const odrive_can_response_cache_t *cache, uint64_t now_us, uint64_t *out_age_us)
{
    const odrive_can_cached_response_t *heartbeat = lookup(cache, ODRIVE_CAN_RESPONSE_HEARTBEAT);
    if (heartbeat == NULL || out_age_us == NULL || now_us < heartbeat->received_at_us) {
        return false;
    }
    *out_age_us = now_us - heartbeat->received_at_us;
    return true;
}

bool odrive_can_cache_heartbeat_is_fresh(
    const odrive_can_response_cache_t *cache, uint64_t now_us, uint64_t max_age_us)
{
    uint64_t age_us;
    return odrive_can_cache_heartbeat_age(cache, now_us, &age_us) && age_us <= max_age_us;
}
