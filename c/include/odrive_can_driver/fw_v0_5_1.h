/* Copyright The odrive-can-protocol Contributors */
/* Copyright The odrive-can-driver Contributors */

/**
 * @file
 * @brief ODrive fw-v0.5.1 和 MKS ODrive Mini fw-v0.5.1-20250326 的 CANSimple 双向编解码接口。
 *
 * 调用方负责 CAN 收发。目标平台须使用 8 位字节和 IEEE 754 binary32 浮点表示。
 * 线上多字节字段按小端字节序传输。
 */

#ifndef ODRIVE_CAN_DRIVER_FW_V0_5_1_H
#define ODRIVE_CAN_DRIVER_FW_V0_5_1_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** CAN 标识符格式。 */
typedef enum {
    ODRIVE_CAN_ID_STANDARD = 1, /**< 11 位标准标识符。 */
    ODRIVE_CAN_ID_EXTENDED,     /**< 扩展标识符，解码时拒绝。 */
} odrive_can_id_kind_t;

/** 调用方提供的 CAN 帧类型。 */
typedef enum {
    ODRIVE_CAN_FRAME_DATA = 1, /**< 经典 CAN 数据帧。 */
    ODRIVE_CAN_FRAME_REMOTE,   /**< 经典 CAN 远程帧（RTR）。 */
    ODRIVE_CAN_FRAME_FD,       /**< CAN FD 数据帧。 */
} odrive_can_frame_kind_t;

/** 供解码使用的 CAN 帧视图，数据缓冲区由调用方持有。 */
typedef struct {
    odrive_can_id_kind_t id_kind;   /**< 标识符格式。 */
    uint32_t id;                    /**< 标识符数值，去除底层传输标志位。 */
    odrive_can_frame_kind_t kind;   /**< 帧类型。 */
    const uint8_t *data;            /**< 非空经典 CAN 数据帧的载荷缓冲区，至少可读 data_len 字节。RTR 忽略此字段。 */
    size_t data_len;                /**< 经典 CAN 数据帧的实际有效载荷字节数。RTR 忽略此字段。 */
    uint8_t dlc;                   /**< 底层报告的 DLC，解码时仅用于 RTR。 */
} odrive_can_frame_view_t;

/** 编码得到的 11 位标准经典 CAN 帧。 */
typedef struct {
    uint16_t id;                    /**< 11 位标识符数值。 */
    odrive_can_frame_kind_t kind;   /**< 数据帧或远程帧。 */
    uint8_t dlc;                   /**< 线上 DLC，远程帧也保留该值。 */
    uint8_t data_len;              /**< 有效载荷字节数，远程帧为 0。 */
    uint8_t data[8];               /**< 固定长度的可读缓冲区，有效载荷之后的字节均补 0。 */
} odrive_can_frame_t;

/** 轴状态原始值，保留未知状态。 */
typedef uint32_t odrive_can_axis_state_t;

/** odrive_can_axis_state_t 的已知状态常量。 */
enum {
    ODRIVE_CAN_AXIS_STATE_UNDEFINED = 0,              /**< 未定义状态。 */
    ODRIVE_CAN_AXIS_STATE_IDLE = 1,                   /**< 空闲状态。 */
    ODRIVE_CAN_AXIS_STATE_STARTUP_SEQUENCE = 2,       /**< 启动序列。 */
    ODRIVE_CAN_AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3, /**< 完整校准序列。 */
    ODRIVE_CAN_AXIS_STATE_MOTOR_CALIBRATION = 4,      /**< 电机校准。 */
    ODRIVE_CAN_AXIS_STATE_SENSORLESS_CONTROL = 5,     /**< 无感控制。 */
    ODRIVE_CAN_AXIS_STATE_ENCODER_INDEX_SEARCH = 6,   /**< 编码器索引搜索。 */
    ODRIVE_CAN_AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7, /**< 编码器偏移校准。 */
    ODRIVE_CAN_AXIS_STATE_CLOSED_LOOP_CONTROL = 8,    /**< 闭环控制。 */
    ODRIVE_CAN_AXIS_STATE_LOCKIN_SPIN = 9,            /**< 锁相旋转。 */
    ODRIVE_CAN_AXIS_STATE_ENCODER_DIR_FIND = 10,      /**< 编码器方向查找。 */
    ODRIVE_CAN_AXIS_STATE_HOMING = 11,               /**< 回零。 */
};

/** 主机发往轴的写命令类型。 */
typedef enum {
    ODRIVE_CAN_COMMAND_ESTOP = 1,               /**< 请求轴进入急停故障状态。 */
    ODRIVE_CAN_COMMAND_SET_AXIS_NODE_ID,       /**< 设置 CAN 节点号。 */
    ODRIVE_CAN_COMMAND_SET_AXIS_REQUESTED_STATE, /**< 请求轴进入指定状态。 */
    ODRIVE_CAN_COMMAND_SET_CONTROLLER_MODES,   /**< 设置控制模式与输入模式。 */
    ODRIVE_CAN_COMMAND_SET_INPUT_POS,          /**< 设置位置与前馈量。 */
    ODRIVE_CAN_COMMAND_SET_INPUT_VEL,          /**< 设置速度与转矩前馈。 */
    ODRIVE_CAN_COMMAND_SET_INPUT_TORQUE,       /**< 设置转矩。 */
    ODRIVE_CAN_COMMAND_SET_VELOCITY_LIMIT,     /**< 设置速度上限（0x0f，4 字节）。 */
    ODRIVE_CAN_COMMAND_START_ANTICOGGING,      /**< 启动抗齿槽校准。 */
    ODRIVE_CAN_COMMAND_SET_TRAJ_VEL_LIMIT,     /**< 设置轨迹速度上限。 */
    ODRIVE_CAN_COMMAND_SET_TRAJ_ACCEL_LIMITS,  /**< 设置轨迹加速度与减速度上限。 */
    ODRIVE_CAN_COMMAND_SET_TRAJ_INERTIA,       /**< 设置轨迹惯量。 */
    ODRIVE_CAN_COMMAND_REBOOT,                 /**< 请求设备复位。 */
    ODRIVE_CAN_COMMAND_CLEAR_ERRORS,           /**< 请求清除设备错误。 */
} odrive_can_command_kind_t;

/** 主机发往轴的写命令，调用方按 kind 选择联合体成员。 */
typedef struct {
    odrive_can_command_kind_t kind; /**< 命令类型。 */
    union {
        struct {
            uint32_t node_id; /**< 新节点号，范围为 0 至 63，含两端。 */
        } set_axis_node_id; /**< 设置节点号命令。 */
        struct {
            odrive_can_axis_state_t state; /**< 编码完整的 32 位原始值，固件读取低 16 位。 */
        } set_axis_requested_state; /**< 请求轴状态命令。 */
        struct {
            int32_t control_mode; /**< 固件控制模式原始值。 */
            int32_t input_mode;   /**< 固件输入模式原始值。 */
        } set_controller_modes; /**< 设置控制模式与输入模式命令。 */
        struct {
            float position;      /**< 位置设定值，单位为转（turn）。 */
            int16_t velocity_ff; /**< 速度前馈原始值，每计数为 0.001 turn/s。 */
            int16_t torque_ff;   /**< 转矩前馈原始值，每计数为 0.001 N·m。 */
        } set_input_pos; /**< 位置命令。 */
        struct {
            float velocity;  /**< 速度设定值，单位为 turn/s。 */
            float torque_ff; /**< 转矩前馈值，单位为 N·m。 */
        } set_input_vel; /**< 速度命令。 */
        struct {
            float torque; /**< 转矩设定值，单位为 N·m。 */
        } set_input_torque; /**< 转矩命令。 */
        struct {
            float velocity; /**< 速度上限，单位为 turn/s。 */
        } set_velocity_limit; /**< 设置速度上限命令。 */
        struct {
            float velocity; /**< 轨迹速度上限，单位为 turn/s。 */
        } set_traj_vel_limit; /**< 设置轨迹速度上限命令。 */
        struct {
            float acceleration; /**< 轨迹加速度上限，单位为 turn/s²。 */
            float deceleration; /**< 轨迹减速度上限，单位为 turn/s²。 */
        } set_traj_accel_limits; /**< 设置轨迹加速度与减速度上限命令。 */
        struct {
            float inertia; /**< 轨迹惯量配置值，单位为 N·m/(turn/s²)。 */
        } set_traj_inertia; /**< 设置轨迹惯量命令。 */
    } value; /**< 命令载荷。急停、启动抗齿槽校准、复位和清错命令的载荷为空。 */
} odrive_can_command_t;

/** 主机发往轴的 RTR 查询类型。 */
typedef enum {
    ODRIVE_CAN_QUERY_MOTOR_ERROR = 1,          /**< 查询电机错误位图。 */
    ODRIVE_CAN_QUERY_ENCODER_ERROR,           /**< 查询编码器错误位图。 */
    ODRIVE_CAN_QUERY_SENSORLESS_ERROR,        /**< 查询无感估算器错误位图。 */
    ODRIVE_CAN_QUERY_ENCODER_ESTIMATES,       /**< 查询编码器位置与速度估算值。 */
    ODRIVE_CAN_QUERY_ENCODER_COUNT,           /**< 查询编码器累计计数与圈内计数。 */
    ODRIVE_CAN_QUERY_IQ,                      /**< 查询 q 轴电流设定值与测量值。 */
    ODRIVE_CAN_QUERY_SENSORLESS_ESTIMATES,    /**< 查询无感位置与速度估算值。 */
    ODRIVE_CAN_QUERY_VBUS_VOLTAGE,            /**< 查询直流母线电压。 */
} odrive_can_query_t;

/** 轴发往主机的遥测与查询回复类型。 */
typedef enum {
    ODRIVE_CAN_RESPONSE_HEARTBEAT = 1,           /**< 周期心跳。 */
    ODRIVE_CAN_RESPONSE_MOTOR_ERROR,            /**< 电机错误位图。 */
    ODRIVE_CAN_RESPONSE_ENCODER_ERROR,          /**< 编码器错误位图。 */
    ODRIVE_CAN_RESPONSE_SENSORLESS_ERROR,       /**< 无感估算器错误位图。 */
    ODRIVE_CAN_RESPONSE_ENCODER_ESTIMATES,      /**< 编码器位置与速度估算值。 */
    ODRIVE_CAN_RESPONSE_ENCODER_COUNT,          /**< 编码器累计计数与圈内计数。 */
    ODRIVE_CAN_RESPONSE_IQ,                     /**< q 轴电流设定值与测量值。 */
    ODRIVE_CAN_RESPONSE_SENSORLESS_ESTIMATES,   /**< 无感位置与速度估算值。 */
    ODRIVE_CAN_RESPONSE_VBUS_VOLTAGE,           /**< 直流母线电压。 */
} odrive_can_response_kind_t;

/** 轴发往主机的遥测或查询回复，kind 指定有效的联合体成员。 */
typedef struct {
    odrive_can_response_kind_t kind; /**< 回复类型。 */
    union {
        struct {
            uint32_t axis_error;                 /**< 轴错误位图，保留未知位。 */
            odrive_can_axis_state_t axis_state;  /**< 当前轴状态原始值。 */
        } heartbeat; /**< 周期心跳。 */
        uint32_t motor_error;      /**< 电机错误位图，保留未知位。 */
        uint32_t encoder_error;    /**< 编码器错误位图，保留未知位。 */
        uint32_t sensorless_error; /**< 无感估算器错误位图，保留未知位。 */
        struct {
            float position; /**< 编码器位置估算值，单位为转（turn）。 */
            float velocity; /**< 编码器速度估算值，单位为 turn/s。 */
        } encoder_estimates; /**< 编码器估算值。 */
        struct {
            int32_t shadow_count;  /**< 编码器累计计数。 */
            int32_t count_in_cpr;  /**< 编码器圈内计数。 */
        } encoder_count; /**< 编码器计数。 */
        struct {
            float setpoint; /**< q 轴电流设定值，单位为 A。 */
            float measured; /**< q 轴电流测量值，单位为 A。 */
        } iq; /**< q 轴电流。 */
        struct {
            float position; /**< 无感估算器 PLL 位置，单位为 rad。 */
            float velocity; /**< 无感速度估算值，单位为 turn/s。 */
        } sensorless_estimates; /**< 无感估算值。 */
        float vbus_voltage; /**< 直流母线电压，单位为 V。 */
    } value; /**< 回复载荷。 */
} odrive_can_response_t;

/** CANSimple 消息的方向与类型。 */
typedef enum {
    ODRIVE_CAN_MESSAGE_COMMAND = 1, /**< 主机发往轴的写命令。 */
    ODRIVE_CAN_MESSAGE_REQUEST,     /**< 主机发往轴的 RTR 查询。 */
    ODRIVE_CAN_MESSAGE_RESPONSE,    /**< 轴发往主机的遥测或查询回复。 */
} odrive_can_message_kind_t;

/** 双向 CANSimple 消息，kind 指定有效的联合体成员。 */
typedef struct {
    odrive_can_message_kind_t kind; /**< 消息类型。 */
    union {
        odrive_can_command_t command;   /**< 主机发往轴的写命令。 */
        odrive_can_query_t query;        /**< 主机发往轴的 RTR 查询。 */
        odrive_can_response_t response; /**< 轴发往主机的遥测或查询回复。 */
    } value; /**< 消息载荷。 */
} odrive_can_message_t;

/** 协议编解码结果。 */
typedef enum {
    ODRIVE_CAN_CODEC_OK = 0,               /**< 已输出帧或消息。 */
    ODRIVE_CAN_CODEC_UNRELATED,            /**< 标准帧属于其他节点，或命令号未知或保留。 */
    ODRIVE_CAN_CODEC_INVALID_ARGUMENT,     /**< 必需指针为空，或类型字段无效。 */
    ODRIVE_CAN_CODEC_INVALID_NODE_ID,      /**< 节点号大于 63。 */
    ODRIVE_CAN_CODEC_NON_FINITE,           /**< 写命令的浮点字段为 NaN 或无穷。 */
    ODRIVE_CAN_CODEC_INVALID_STANDARD_ID,  /**< 标准 ID 超过 0x07ff。 */
    ODRIVE_CAN_CODEC_UNSUPPORTED_EXTENDED_ID, /**< 扩展 CAN ID 不受支持。 */
    ODRIVE_CAN_CODEC_UNSUPPORTED_CAN_FD,   /**< 本节点的已知命令不接受 CAN FD 帧。 */
    ODRIVE_CAN_CODEC_UNSUPPORTED_COMMAND,  /**< 当前固件未实现本节点的命令 0x08。 */
    ODRIVE_CAN_CODEC_INVALID_FRAME_KIND,   /**< 该命令号不接受远程帧。 */
    ODRIVE_CAN_CODEC_INVALID_LENGTH,       /**< 数据帧的有效载荷长度与协议定义不符。 */
    ODRIVE_CAN_CODEC_INVALID_REMOTE_DLC,   /**< 远程帧的 DLC 大于 8。 */
} odrive_can_codec_status_t;

/** ODRIVE_CAN_CODEC_NON_FINITE 错误所指的字段。 */
typedef enum {
    ODRIVE_CAN_FLOAT_FIELD_NONE = 0, /**< 无浮点诊断字段。 */
    ODRIVE_CAN_FLOAT_FIELD_POSITION, /**< 位置设定值。 */
    ODRIVE_CAN_FLOAT_FIELD_VELOCITY, /**< 速度设定值或上限。 */
    ODRIVE_CAN_FLOAT_FIELD_TORQUE_FF, /**< 转矩前馈。 */
    ODRIVE_CAN_FLOAT_FIELD_TORQUE, /**< 转矩设定值。 */
    ODRIVE_CAN_FLOAT_FIELD_ACCELERATION, /**< 轨迹加速度。 */
    ODRIVE_CAN_FLOAT_FIELD_DECELERATION, /**< 轨迹减速度。 */
    ODRIVE_CAN_FLOAT_FIELD_INERTIA, /**< 轨迹惯量。 */
} odrive_can_float_field_t;

/** 可选的编解码诊断信息，除 status 外，不适用的字段均为 0。 */
typedef struct {
    odrive_can_codec_status_t status; /**< 与编解码函数返回值相同。 */
    uint32_t value;       /**< 出错的节点号、标准 ID 或远程帧 DLC。 */
    uint8_t command_id;  /**< 命令未实现、RTR 帧类型错误或长度不符时的 5 位线上命令号。 */
    size_t expected_length; /**< ODRIVE_CAN_CODEC_INVALID_LENGTH 所要求的载荷字节数。 */
    size_t actual_length;   /**< ODRIVE_CAN_CODEC_INVALID_LENGTH 所报告的实际载荷字节数。 */
    odrive_can_float_field_t field; /**< ODRIVE_CAN_CODEC_NON_FINITE 所指的写命令字段。 */
} odrive_can_codec_error_t;

/**
 * @brief 将 CANSimple 消息编码为 11 位标准经典 CAN 帧。
 *
 * 写命令的浮点字段必须为有限数，应用负责校验物理范围。设备回复保留浮点位模式，
 * 包括 NaN 和无穷。所有回复均为 DLC 8，错误和直流母线电压回复的后四字节补 0。
 * RTR 查询编码为 DLC 8、有效载荷长度为 0 的远程帧。
 *
 * @param[in] node_id 命令或查询的目标节点、回复的源节点，范围为 0 至 63。63 是普通节点。
 * @param[in] message 输入消息，须非空，并提供与类型匹配的载荷。
 * @param[out] frame 帧输出，须非空，仅在返回 ODRIVE_CAN_CODEC_OK 时写入。
 * @param[out] error 可选诊断信息，可为 NULL。提供时须与输入及帧输出使用不同存储。
 * @return ODRIVE_CAN_CODEC_OK、ODRIVE_CAN_CODEC_INVALID_ARGUMENT、
 *         ODRIVE_CAN_CODEC_INVALID_NODE_ID 或 ODRIVE_CAN_CODEC_NON_FINITE。
 *
 * 此函数只构造帧。实际发送该节点的帧（含 RTR 查询）可能喂设备看门狗。
 * 设备执行结果须依据设备反馈判断。
 */
odrive_can_codec_status_t odrive_can_v051_encode(
    uint32_t node_id,
    const odrive_can_message_t *message,
    odrive_can_frame_t *frame,
    odrive_can_codec_error_t *error);

/**
 * @brief 解码指定节点的 CANSimple 帧。
 *
 * 先检查标识符格式及宽度，再按节点过滤。其他节点以及未知或保留命令号返回
 * ODRIVE_CAN_CODEC_UNRELATED，无需检查帧类型或载荷。本节点的 0x08 返回
 * ODRIVE_CAN_CODEC_UNSUPPORTED_COMMAND。其他已知命令要求经典 CAN。
 * 数据帧须具有协议定义的准确载荷长度，按 data_len 解析。
 * RTR 仅用于查询，接受 DLC 0 至 8，忽略 data 和 data_len。
 * 需要校验或记录原始 DLC 和缓冲区边界时，由调用方在构造视图前处理。
 * 解码保留轴状态原始值、所有错误位和非有限浮点位模式。错误和直流母线电压回复只解释前四字节。
 *
 * @param[in] node_id 要匹配的源节点或目标节点，范围为 0 至 63。63 是普通节点。
 * @param[in] frame 输入帧，须非空。解码匹配的非空经典 CAN 数据帧时，载荷缓冲区须可读。
 *                  数据帧的载荷长度为 0 时，缓冲区可为 NULL。
 * @param[out] message 消息输出，须非空，仅在返回 ODRIVE_CAN_CODEC_OK 时写入。
 * @param[out] error 可选诊断信息，可为 NULL。提供时须与输入及消息输出使用不同存储。
 * @return 成功解码时返回 ODRIVE_CAN_CODEC_OK。忽略该帧时返回
 *         ODRIVE_CAN_CODEC_UNRELATED。其他结果为对应的协议错误。
 */
odrive_can_codec_status_t odrive_can_v051_decode(
    uint32_t node_id,
    const odrive_can_frame_view_t *frame,
    odrive_can_message_t *message,
    odrive_can_codec_error_t *error);

/**
 * @brief 借用 odrive_can_v051_encode 生成的帧及其载荷缓冲区。
 *
 * @param[in] frame 已编码的帧。使用视图期间须保持帧的存储有效。
 * @return 包含标准 ID 的帧视图。传入 NULL 时返回全零的无效视图。
 */
odrive_can_frame_view_t odrive_can_v051_frame_view(const odrive_can_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif
