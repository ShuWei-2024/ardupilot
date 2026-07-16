#include "Copter.h"
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "RC_Channel.h"

#if MODE_FOLLOW_EXT_ENABLED

namespace {

// 视觉速度控制功能开关。使用编译期常量，方便在测试时单独禁用某项耦合功能。
static constexpr bool FOLLOW_EXT_ENABLE_ERROR_SLOWDOWN = true;
static constexpr bool FOLLOW_EXT_ENABLE_TURN_ACCEL_LIMIT = true;
static constexpr bool FOLLOW_EXT_ENABLE_SPEED_INDEPENDENT_CLIMB = true;
static constexpr bool FOLLOW_EXT_ENABLE_TURN_ACCEL_FEEDFORWARD = true;
static constexpr bool FOLLOW_EXT_ENABLE_YAW_D_TERM = true;

// 前向速度调度参数。参见文档 docs/follow_ext/FOLLOW_EXT视觉误差与转向需求前速调度说明.md
// 当前协议没有定义 y_axis_err 和 z_axis_err 的单位，因此这些参数必须按照
// 导引实际发送的误差范围调整。建议先记录悬停、正常跟踪和目标接近画面
// 边缘时的误差，再根据日志调参，不要直接照搬到误差单位不同的视觉系统。

// 误差减速尺度：综合视觉误差等于该值时，误差减速系数为 0.5。
// 增大：相同误差下保留更高前速，跟随更积极，但目标偏离时更容易冲过头。
// 减小：更早、更强地减速，目标更容易重新居中，但平均跟随速度会降低。
// 建议先设为“目标偏到画面约一半位置时的综合误差”，再按 20% 幅度调整。
static constexpr float FOLLOW_EXT_ERROR_SLOWDOWN_SCALE = 350.0f;

// 垂向误差参与前向减速的权重，综合误差为 sqrt(y_err^2 + 权重*z_err^2)。
// 设为 0：高度误差不影响前速；设为 1：横向和垂向误差同等影响前速。
// 增大：高度偏差较大时会更明显减速，适合优先保证目标垂向居中。
// 减小：高度误差对前进影响较弱，适合垂向误差噪声较大或不希望频繁减速。
// 建议从 0.5 开始；若升降过程中前速下降过多则减小，反之增大。
static constexpr float FOLLOW_EXT_VERTICAL_ERROR_WEIGHT = 0.8f;

// 误差减速允许的最小前速倍率，最终误差减速倍率不会低于该值。
// 0.0 表示误差很大时允许接近停止；0.2 表示至少保留 FOLE_SPEED 的 20%。
// 增大：持续保持向前运动，但目标严重偏离时可能来不及转向。
// 减小：允许先减速对准目标，通常更安全。建议首次测试保持 0.0～0.1。
static constexpr float FOLLOW_EXT_MIN_SPEED_MULTIPLIER = 0.2f;

// 转弯可使用的水平加速度占位置控制器最大水平加速度的比例。
// 剩余裕度用于速度误差修正、抗风和避障；转弯前速满足 v*|yaw_rate| 不超过该预算。
// 增大：允许更高速度转弯，但倾角、推力占用和转弯外扩风险增加。
// 减小：转弯时减速更明显，轨迹更稳，但跟随速度降低。
// 建议从 0.6 开始；强风、重载或转弯掉高时降至 0.4～0.5，动力充足且
// 实测仍有较大余量时可逐步升至 0.7，但不建议接近 1.0。
static constexpr float FOLLOW_EXT_TURN_ACCEL_BUDGET_RATIO = 0.6f;

// 启用“按转向需求限制前速”的最小偏航角速度，单位 rad/s，此处为 2.0°/s。
// 小于该阈值时不使用 a/|yaw_rate| 计算转弯限速，避免除以很小的角速度。
// 增大：只有明显转向时才触发转弯限速，可减少小角速度噪声造成的速度变化。
// 减小：轻微转向也参与限速，控制更保守，但前速可能更容易波动。
// 一般保持 0.5°/s；若直线飞行时前速抖动可适当增大到 1～2°/s。
static constexpr float FOLLOW_EXT_MIN_YAW_RATE_RAD_S = radians(2.0f);

}

/*
 * ModeFollowExt State Machine
 *
 *        ┌────────┐
 *        │  IDLE  │
 *        └───┬────┘
 *            │ arm + switch to mode 4
 *            ▼
 *        ┌────────┐
 *        │ TAKEOFF│
 *        └───┬────┘
 *            │ reach alt
 *            ▼
 *        ┌────────┐     ┌────────┐
 *        │ POS CTL│◄────┤ VEL CTL│
 *        └────────┘     └────────┘
 */
/*
 * mode_follow.cpp  —— 串口目标版本
 * 数据源：ParsedPacket（lat/lon/alt/velocity/yaw）
 * 不再使用 g2.follow.get_target_dist_and_vel_ned()
 */
const AP_Param::GroupInfo ModeFollowExt::var_info[] = {
    // @Param: AUTO_ENABLE
    // @DisplayName: FollowExt auto enable/disable
    // @Description: Allows you to enable (1) or disable (0) FollowExt auto feature
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("AUTO_ENABLE", 1, ModeFollowExt, _followext_enabled, 1, AP_PARAM_FLAG_ENABLE),
    //  @Param: FOLE_KP_YAW
    //  @DisplayName: Follow mode P gain on yaw
    //  @Description: Proportional gain in follow mode
    //  @Range: -1000 1000
    //  @User: Advanced
    AP_GROUPINFO("KP_YAW", 2, ModeFollowExt, _kp_yaw, 1.5f),
    // @Param: FOLE_KP_THR
    // @DisplayName: Follow mode I gain on throttle
    // @Description: Proportional gain in follow mode
    // @Range: -1000 1000
    // @User: Advanced
    AP_GROUPINFO("KP_THR", 3, ModeFollowExt, _kp_thr, 0.3f),
    // @Param: FOLE_KD_YAW
    // @DisplayName: Follow mode D gain on yaw
    // @Description: Derivative gain in follow mode
    // @User: Advanced
    AP_GROUPINFO("KD_YAW", 4, ModeFollowExt, _kd_yaw, 0.0f),
    // @Param: FOLE_KD_THR
    // @DisplayName: Follow mode D gain on throttle
    // @Description: Derivative gain in follow mode
    // @User: Advanced
    AP_GROUPINFO("KD_THR", 5, ModeFollowExt, _kd_thr, 0.0f),
    //@Param: FOLE_SPEED
    //@DisplayName: Follow mode speed
    //@Description: Fixed speed(cm/s)
    //@Range: 0 10000
    //@Units: cm/s
    //@User: Advanced
    AP_GROUPINFO("SPEED", 6, ModeFollowExt, _speed, 1000.0f),
    //@Param: FOLE_ALPHA
    //@DisplayName: Follow mode alpha
    //@Description: Low pass filter alpha parameter in Follow mode
    //@Range: 0.0 1.0
    //@User: Advanced
    AP_GROUPINFO("ALPHA", 7, ModeFollowExt, _alpha, 1.0f),

    AP_GROUPEND};

ModeFollowExt::ModeFollowExt(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// 初始化
bool ModeFollowExt::init(const bool ignore_checks)
{
    if (!_followext_enabled) {
        return false;
    }

    gcs().send_text(MAV_SEVERITY_DEBUG, "entry FOLLOW_EXT");
    y_err = 0;
    z_err = 0;
    last_log_ms = 0;
    _control_packet_timed_out = false;
    _velocity_invalid_reported = false;
    _last_y_err = 0.0f;
    _last_vision_update_ms = 0;
    _yaw_derivative_valid = false;
    return ModeGuided::init(ignore_checks);
}

CompanionParamStatus ModeFollowExt::handle_external_param(CompanionParamOperation operation,
                                                          uint8_t param_id,
                                                          float request_value,
                                                          float &actual_value)
{
    actual_value = NAN;

    if (operation != CompanionParamOperation::SET_VOLATILE &&
        operation != CompanionParamOperation::SET_PERSISTENT &&
        operation != CompanionParamOperation::GET) {
        return CompanionParamStatus::BAD_OPERATION;
    }

    const bool persistent = operation == CompanionParamOperation::SET_PERSISTENT;
    const bool get_only = operation == CompanionParamOperation::GET;

    auto handle_float = [persistent, get_only, request_value, &actual_value](AP_Float &param,
                                                                            float minimum,
                                                                            float maximum) {
        if (!get_only) {
            if (!isfinite(request_value) || request_value < minimum || request_value > maximum) {
                actual_value = param.get();
                return CompanionParamStatus::INVALID_VALUE;
            }
            if (persistent) {
                param.set_and_save(request_value);
            } else {
                param.set(request_value);
            }
        }
        actual_value = param.get();
        return CompanionParamStatus::OK;
    };

    switch (param_id) {
    case 0x01: // FOLE_AUTO_ENABLE
        if (!get_only) {
            if (!isfinite(request_value) ||
                request_value < 0.0f ||
                request_value > 1.0f ||
                (request_value > 0.0f && request_value < 1.0f)) {
                actual_value = _followext_enabled.get();
                return CompanionParamStatus::INVALID_VALUE;
            }
            const int8_t enabled = static_cast<int8_t>(request_value);
            if (persistent) {
                _followext_enabled.set_and_save(enabled);
            } else {
                _followext_enabled.set(enabled);
            }
        }
        actual_value = _followext_enabled.get();
        return CompanionParamStatus::OK;

    case 0x02: // FOLE_KP_YAW
        return handle_float(_kp_yaw, -1000.0f, 1000.0f);

    case 0x03: // FOLE_KP_THR
        return handle_float(_kp_thr, -1000.0f, 1000.0f);

    case 0x04: // FOLE_KD_YAW
        return handle_float(_kd_yaw, -1000.0f, 1000.0f);

    case 0x05: // FOLE_KD_THR (not used by the current controller)
        actual_value = _kd_thr.get();
        return get_only ? CompanionParamStatus::OK : CompanionParamStatus::NOT_SUPPORTED;

    case 0x06: // FOLE_SPEED, cm/s
        return handle_float(_speed, 0.0f, 10000.0f);

    case 0x07: // FOLE_ALPHA
        return handle_float(_alpha, 0.0f, 1.0f);

    default:
        return CompanionParamStatus::UNKNOWN_PARAM;
    }
}

void ModeFollowExt::run()
{
    /* 2. 10 Hz 日志标记 */
    const uint32_t now = AP_HAL::millis();
    const bool ten_hz_flag = (now - last_log_ms >= 100) || (last_log_ms == 0);
    if (ten_hz_flag)
        last_log_ms = now;

    /* 3. 电机解锁状态 */
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    /* 4. 取 CompanionComputer 单例，并把最新包拷出来 */
    auto &cc = AP::companioncomputer();
    const CompanionReceivePacket pkt = cc.get_received_packet(); // 结构体拷贝，线程安全

    // Do not keep refreshing Guided with an old command when the companion
    // computer stops sending control packets.  Explicitly command a hover so
    // the last forward velocity and yaw-rate cannot remain active forever.
    const uint32_t last_control_packet_ms = cc.get_last_control_packet_ms();
    const bool control_packet_timed_out =
        last_control_packet_ms == 0 ||
        now - last_control_packet_ms > CONTROL_PACKET_TIMEOUT_MS;
    if (control_packet_timed_out) {
        const bool timeout_started = !_control_packet_timed_out;
        _control_packet_timed_out = true;
        y_err = 0;
        z_err = 0;
        _yaw_derivative_valid = false;
        _last_vision_update_ms = 0;

        if (timeout_started) {
            gcs().send_text(MAV_SEVERITY_WARNING, "FOLLOW_EXT: control timeout");
        }

        if (timeout_started || ten_hz_flag) {
            const Vector3f zero_velocity_neu_cms{0.0f, 0.0f, 0.0f};
            ModeGuided::set_velocity(zero_velocity_neu_cms,
                                     false, 0.0f,
                                     true, 0.0f,
                                     false,
                                     false);
        }
        ModeGuided::run();
        return;
    }

    if (_control_packet_timed_out) {
        _control_packet_timed_out = false;
        gcs().send_text(MAV_SEVERITY_INFO, "FOLLOW_EXT: control restored");
    }

    /* 5. 根据 ctrl_mode 决定控制方式 */
    switch (pkt.ctrl_mode) {
    case 1: { // 视觉导引模式
        /*  定姿
        // 1. 误差量（机体坐标，m）
        // float x_err = pkt.x_axis_err; // 前，用不到
        float y_err = pkt.y_axis_err; // 右
        float z_err = pkt.z_axis_err; // 下

        // 2. 控制参数
        const float pitch_fixed = 25.0f * DEG_TO_RAD; // 固定 25°

        // 3. 滚转=0，俯仰=固定，偏航=当前航向
        float roll_rad = 0.0f;
        float pitch_rad = pitch_fixed;
        float yaw_rad = copter.ahrs.get_yaw(); // 保持当前航向作为基准

        // 4. 根据 y_err 计算偏航角速率（rad/s，机体轴）
        float yaw_rate_cmd = -_kp_yaw * y_err; // 负号：y>0（目标在右侧）→ 需左转

        // 5. 根据 z_err 计算高度补偿
        float thrust_bas = 0.0f;                        // 悬停基准推力
        float thrust_cmd = thrust_bas - _kp_thr * z_err; // z>0（目标在下）→ 需提高高度

        // 6. 组装角速度（机体轴，rad/s）
        Vector3f ang_vel_body(0.0f, 0.0f, yaw_rate_cmd); // 滚转、俯仰速率=0

        // 7. 生成姿态四元数
        Quaternion att_quat;
        att_quat.from_euler(roll_rad, pitch_rad, yaw_rad);

        // 8. 调用 guided 角度接口
        ModeGuided::set_angle(att_quat, ang_vel_body, thrust_cmd, false);
        */

        _last_altitude = 0;
        _last_latitude = 0;
        _last_altitude = 0;
        /* 定速*/
        /* 1. 误差量（机体坐标，m） */
        if(ten_hz_flag == 0){   //10hz控制
            break;
        }
        // 添加低通滤波减少抖动
        float alpha = constrain_float(_alpha.get(), 0.0f, 1.0f);
        y_err = alpha * (float)pkt.y_axis_err + (1.0f - alpha) * y_err;// + 右
        z_err = alpha * (float)pkt.z_axis_err + (1.0f - alpha) * z_err;// + 下

        /* 2. 计算机体轴角速率 / 爬升速率 */
        float yaw_error_rate = 0.0f;
        if (FOLLOW_EXT_ENABLE_YAW_D_TERM && _yaw_derivative_valid && _last_vision_update_ms != 0) {
            const float vision_dt = (now - _last_vision_update_ms) * 0.001f;
            if (vision_dt > 0.0f && vision_dt < 0.5f) {
                yaw_error_rate = (y_err - _last_y_err) / vision_dt;
            }
        }
        _last_y_err = y_err;
        _last_vision_update_ms = now;
        _yaw_derivative_valid = true;

        float yaw_rate_cds = _kp_yaw * y_err; // centideg/s
        if (FOLLOW_EXT_ENABLE_YAW_D_TERM) {
            yaw_rate_cds += _kd_yaw * yaw_error_rate;
        }
        yaw_rate_cds = constrain_float(yaw_rate_cds, -2500.0f, 2500.0f);

        float forward_speed_cms = MAX(_speed.get(), 0.0f);
        const float yaw_rate_rad_s = radians(yaw_rate_cds * 0.01f);
        const float turn_accel_budget_cmss =
            FOLLOW_EXT_TURN_ACCEL_BUDGET_RATIO * pos_control->get_max_accel_xy_cmss();

        if (FOLLOW_EXT_ENABLE_ERROR_SLOWDOWN) {
            const float error_magnitude = safe_sqrt(sq(y_err) +
                                                    FOLLOW_EXT_VERTICAL_ERROR_WEIGHT * sq(z_err));
            const float error_ratio = error_magnitude / FOLLOW_EXT_ERROR_SLOWDOWN_SCALE;
            const float error_speed_multiplier = constrain_float(
                1.0f / (1.0f + sq(error_ratio)),
                FOLLOW_EXT_MIN_SPEED_MULTIPLIER,
                1.0f);
            forward_speed_cms *= error_speed_multiplier;
        }

        if (FOLLOW_EXT_ENABLE_TURN_ACCEL_LIMIT) {
            // 转弯向心加速度 a_turn = v*omega。限制前速，避免转弯耗尽
            // 速度控制环所需的水平加速度裕度。
            if (turn_accel_budget_cmss > 0.0f &&
                fabsf(yaw_rate_rad_s) > FOLLOW_EXT_MIN_YAW_RATE_RAD_S) {
                forward_speed_cms = MIN(forward_speed_cms,
                                        turn_accel_budget_cmss / fabsf(yaw_rate_rad_s));
            }
        }

        const float kp_thr = _kp_thr.get();
        float kp_thr_eff = kp_thr;
        if (!FOLLOW_EXT_ENABLE_SPEED_INDEPENDENT_CLIMB) {
            const float v_now = MAX(_speed.get(), 100.0f); // 旧版垂向增益调度
            const float speed_ratio = v_now / 500.0f;
            kp_thr_eff = kp_thr * sqrtf(0.5f + 0.5f * sq(speed_ratio));
            const float kp_thr_limit_a = kp_thr * 0.5f;
            const float kp_thr_limit_b = kp_thr * 2.0f;
            kp_thr_eff = constrain_float(kp_thr_eff,
                                         MIN(kp_thr_limit_a, kp_thr_limit_b),
                                         MAX(kp_thr_limit_a, kp_thr_limit_b));
        }
        float climb_rate_cms = kp_thr_eff * z_err;
        climb_rate_cms = constrain_float(climb_rate_cms, -200.0f, 200.0f);

        Vector3f vel_vector;
        vel_vector.x = forward_speed_cms;
        vel_vector.y = 0;
        vel_vector.z = climb_rate_cms;
        Vector3f accel_vector;
        if (FOLLOW_EXT_ENABLE_TURN_ACCEL_FEEDFORWARD) {
            // 机体前向速度以 yaw_rate 转动时，需要 v*omega 的机体 Y 轴
            // 向心加速度；机体 Y 轴正方向为右。
            accel_vector.y = constrain_float(forward_speed_cms * yaw_rate_rad_s,
                                             -turn_accel_budget_cmss,
                                             turn_accel_budget_cmss);
        }
        bool velocity_invalid = false;
        for (uint8_t i = 0; i < 3; i++) {
            // consider velocity invalid if any component is non-finite or exceeds 10000 cm/s
            if (!isfinite(vel_vector[i]) || fabsf(vel_vector[i]) > 10000.0f) {
                velocity_invalid = true;
                break;
            }
        }
        if (velocity_invalid) {
            if (!_velocity_invalid_reported) {
                _velocity_invalid_reported = true;
                gcs().send_text(MAV_SEVERITY_ERROR, "FOLLOW_EXT: invalid velocity");
            }
            const Vector3f zero_velocity_neu_cms{0.0f, 0.0f, 0.0f};
            ModeGuided::set_velocity(zero_velocity_neu_cms,
                                     false, 0.0f,
                                     true, 0.0f,
                                     false,
                                     false);
            break;
        }
        _velocity_invalid_reported = false;
        copter.rotate_body_frame_to_NE(vel_vector.x, vel_vector.y);
        copter.rotate_body_frame_to_NE(accel_vector.x, accel_vector.y);
        ModeGuided::set_velaccel(vel_vector,           // NEU cm/s，向上为正
                                 accel_vector,         // NEU cm/s/s
                                 false, 0,              // 不指定绝对 yaw
                                 true, yaw_rate_cds,    // 指定 yaw-rate
                                 false,                 // 绝对 yaw-rate
                                 ten_hz_flag);
        break;
    }
    case 3: { // 位置控制模式
        _yaw_derivative_valid = false;
        _last_vision_update_ms = 0;
        if(_last_lontitude != pkt.target_lon || _last_latitude != pkt.target_lat || _last_altitude != pkt.target_alt || _last_max_velocity != pkt.max_velocity){
            y_err = 0;
            z_err = 0;
            Location target_loc(pkt.target_lat, pkt.target_lon, pkt.target_alt, Location::AltFrame::ABOVE_HOME);
            _last_lontitude = pkt.target_lon;
            _last_latitude = pkt.target_lat;
            _last_altitude = pkt.target_alt;
            ModeGuided::set_destination(target_loc, false, 0.0f, false, 0.0f, false);

            if(pkt.max_velocity > 30){
                pos_control->set_max_speed_accel_xy(pkt.max_velocity, 250.0f);
                pos_control->set_correction_speed_accel_xy(pkt.max_velocity, 250.0f);
            } else {
                pos_control->set_max_speed_accel_xy(30, 30.0f);
                pos_control->set_correction_speed_accel_xy(30, 30.0f);
            }
            _last_max_velocity = pkt.max_velocity;
        }
        break;
    }
    case 4: { // 起飞模式
        _yaw_derivative_valid = false;
        _last_vision_update_ms = 0;
        //如果没有arm，先arm
        if(!ten_hz_flag){ 
            break;
        }
        if (!copter.arming.is_armed()) {
            if (!copter.arming.arm(AP_Arming::Method::MAVLINK)) {
                // gcs().send_text(MAV_SEVERITY_ERROR, "FOLLOW_EXT: Arm failed");
            }
        }else{
            if(!_takeoff_complete){
                do_user_takeoff(pkt.z_axis_err, false);
                // do_user_takeoff_start(_takeoff_target_alt_cm);
                _takeoff_complete = true;
            }
        }
        break;
    }
    case 5: { // 紧急停转模式
        _yaw_derivative_valid = false;
        _last_vision_update_ms = 0;
        /* 5.1 立即disarm */
        copter.arming.disarm(AP_Arming::Method::AUXSWITCH);

        /* 5.2 不再下发任何 thrust / vel / pos 指令，让控制器空跑 */
        Vector3f zero_vel_neu_cms{0, 0, 0};
        ModeGuided::set_velocity(zero_vel_neu_cms, // NED cm/s
                                 false, 0,         // 不控 yaw
                                 false, 0,         // 不控 yaw-rate
                                 false,
                                 ten_hz_flag);
        break;
    }
    default:
        _yaw_derivative_valid = false;
        _last_vision_update_ms = 0;
        /* 未定义模式，可以原地悬停或什么都不做 */
        if(ten_hz_flag){
            _last_altitude = 0;
            _last_latitude = 0;
            _last_altitude = 0;
            y_err = 0;
            z_err = 0;
            Vector3f desired_velocity_neu_cms(0.0f, 0.0f, 0.0f); // NED, cm/s
            ModeGuided::set_velocity(desired_velocity_neu_cms, false, 0.0, false, 0.0f, false, ten_hz_flag);
        }
        break;
    }

    /* 6. 让 guided 的姿态环继续跑 */
    ModeGuided::run();
}

#endif // MODE_FOLLOW_ENABLED
