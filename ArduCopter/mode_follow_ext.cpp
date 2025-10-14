#include "Copter.h"
#include <AP_Param/AP_Param.h>

#if MODE_FOLLOW_EXT_ENABLED

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
    //  @User: Advanced
    AP_GROUPINFO("KP_YAW", 2, ModeFollowExt, _kp_yaw, 2.0f),
    // @Param: FOLE_KP_THR
    // @DisplayName: Follow mode I gain on throttle
    // @Description: Proportional gain in follow mode
    // @User: Advanced
    AP_GROUPINFO("KP_THR", 3, ModeFollowExt, _kp_thr, 2.0f),
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
    //@Param: FOLE_Pitch
    //@DisplayName: Follow mode pitch angle
    //@Description: Fixed pitch angle in follow mode (degrees)
    //@User: Advanced
    AP_GROUPINFO("PITCH", 6, ModeFollowExt, _pitch_fixed, 25.0f),

    AP_GROUPEND};
// 工具：把经纬度差转换成 cm（水平面）
// static Vector2f diff_location_cm(const Location &loc1, const Location &loc2)
// {
//     float bearing = loc1.get_bearing_to(loc2) * 0.01f; // centideg -> deg
//     float dist = loc1.get_distance(loc2) * 100.0f;     // m -> cm
//     return Vector2f(dist * cosf(radians(bearing)), dist * sinf(radians(bearing)));
// }

// 初始化
bool ModeFollowExt::init(const bool ignore_checks)
{
    // return ModeGuided::init(ignore_checks);
    AP_Param::setup_object_defaults(this, var_info);
    gcs().send_text(MAV_SEVERITY_DEBUG, "entry FOLLOW_EXT");
    return true;
}

void ModeFollowExt::run()
{
    /* 1. 基础安全处理（可选，需要就打开） */
    // if (is_disarmed_or_landed()) {
    //     make_safe_ground_handling();
    //     return;
    // }

    /* 2. 10 Hz 日志标记 */
    const uint32_t now = AP_HAL::millis();
    const bool log_request = (now - last_log_ms >= 100) || (last_log_ms == 0);
    if (log_request)
        last_log_ms = now;

    /* 3. 电机解锁状态 */
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    /* 4. 取 CompanionComputer 单例，并把最新包拷出来 */
    const auto &cc = AP::companioncomputer();
    const CompanionReceivePacket pkt = cc.get_received_packet(); // 结构体拷贝，线程安全

    /* 5. 根据 ctrl_mode 决定控制方式 */
    switch (pkt.ctrl_mode) {
    case 1: { // 角度控制模式 TODO:根据要求的速度测个角度,set_velocity控制?
        /* 1. 误差量（机体坐标，m） */
        // float x_err = pkt.x_axis_err; // 前，用不到
        float y_err = pkt.y_axis_err; // 右
        float z_err = pkt.z_axis_err; // 下

        /* 2. 控制参数 */
        const float pitch_fixed = 25.0f * DEG_TO_RAD; // 固定 25°

        /* 3. 滚转=0，俯仰=固定，偏航=当前航向 */
        float roll_rad = 0.0f;
        float pitch_rad = pitch_fixed;
        float yaw_rad = copter.ahrs.get_yaw(); // 保持当前航向作为基准

        /* 4. 根据 y_err 计算偏航角速率（rad/s，机体轴）*/
        float yaw_rate_cmd = -_kp_yaw * y_err; // 负号：y>0（目标在右侧）→ 需左转

        /* 5. 根据 z_err 计算推力补偿（0~1）*/
        float thrust_bas = 0.5f;                        // 悬停基准推力
        float thrust_cmd = thrust_bas - _kp_thr * z_err; // z>0（目标在下）→ 需加大推力
        thrust_cmd = constrain_float(thrust_cmd, 0.2f, 0.8f);

        /* 6. 组装角速度（机体轴，rad/s）*/
        Vector3f ang_vel_body(0.0f, 0.0f, yaw_rate_cmd); // 滚转、俯仰速率=0

        /* 7. 生成姿态四元数 */
        Quaternion att_quat;
        att_quat.from_euler(roll_rad, pitch_rad, yaw_rad);

        /* 8. 调用 guided 角度接口 */
        ModeGuided::set_angle(att_quat, ang_vel_body, thrust_cmd, true); // use_thrust=true

        break;
    }
    case 3: { // 位置控制模式
        Location target_loc(pkt.target_lat, pkt.target_lon, pkt.target_alt, Location::AltFrame::ABOVE_HOME);
        if(_last_max_velocity != pkt.max_velocity){
            pos_control->set_max_speed_accel_xy(pkt.max_velocity, 200.0f); // cm/s, cm/s^2
            _last_max_velocity = pkt.max_velocity;
        }
        ModeGuided::set_destination(target_loc, false, 0.0f, false, 0.0f, false);

        break;
    }
    case 4: { // 起飞模式
        //如果没有arm，先arm
        if (!copter.arming.is_armed()) {
            if (!copter.arming.arm(AP_Arming::Method::MAVLINK)) {
                gcs().send_text(MAV_SEVERITY_ERROR, "FOLLOW_EXT: Arm failed");
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
    default:
        /* 未定义模式，可以原地悬停或什么都不做 */
        Vector3f desired_velocity_neu_cms(1500.0f, 0.0f, 0.0f); // NEU, cm/s
        ModeGuided::set_velocity(desired_velocity_neu_cms, false, 0.0, false, 0.0f, false, log_request);
        break;
    }

    /* 6. 让 guided 的姿态环继续跑 */
    ModeGuided::run();
}

#endif // MODE_FOLLOW_ENABLED