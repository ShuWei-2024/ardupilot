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
    //  @Param: FOLLOW_KP
    //  @DisplayName: Follow mode P gain
    //  @Description: Proportional gain for position error in follow mode
    //  @User: Advanced
    AP_GROUPINFO("FOLLOW_KP", 2, ModeFollowExt, _kp_param, 5.0),
    // @Param: FOLLOW_KI
    // @DisplayName: Follow mode I gain
    // @Description: Integral gain for position error in follow mode
    // @User: Advanced
    AP_GROUPINFO("FOLLOW_KI", 3, ModeFollowExt, _ki_param, 0.0f),
    // @Param: FOLLOW_KD
    // @DisplayName: Follow mode D gain
    // @Description: Derivative gain for position error in follow mode
    // @User: Advanced
    AP_GROUPINFO("FOLLOW_KD", 4, ModeFollowExt, _kd_param, 0.0f),
    // @Param: FOLLOW_IMAX
    // @DisplayName: Follow mode I max
    // @Description: Maximum integral term for position error in follow mode
    // @User: Advanced
    AP_GROUPINFO("FOLLOW_IMAX", 5, ModeFollowExt, _imax_param, 0.0f),

    AP_GROUPEND
};
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

// 主循环
void ModeFollowExt::run()
{
    // // 基础安全处理
    // if (is_disarmed_or_landed()) {
    //     make_safe_ground_handling();
    //     return;
    // }
    // log output at 10hz
    uint32_t now = AP_HAL::millis();
    bool log_request = false;
    if ((now - last_log_ms >= 100) || (last_log_ms == 0)) {
        log_request = true;
        last_log_ms = now;
    }
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    // const auto &cc = AP::companioncomputer();
    // int a = cc.get_received_packet().ctrl_mode;
    Vector3f desired_velocity_neu_cms(100.0f, 0.0f, 0.0f); // NEU, cm/s
    // re-use guided mode's velocity controller (takes NEU)
    ModeGuided::set_velocity(desired_velocity_neu_cms, false, 0.0, false, 0.0f, false, log_request);
    ModeGuided::run();
    gcs().send_text(MAV_SEVERITY_DEBUG, "run FOLLOW_EXT");
    // const CompanionReceivePacket &pkt = copter.companion_computer.get_received_packet();
    // // 2. 构造目标 Location
    // Location target_loc;
    // target_loc.lat = pkt.target_lat * 1.0e7; // 度 -> 1e7
    // target_loc.lng = pkt.target_lon * 1.0e7;
    // target_loc.alt = pkt.target_alt * 100.0f; // m -> cm（相对原点）

    // // 3. 计算相对向量（NEU，cm）
    // Vector2f offset_ne_cm = diff_location_cm(copter.current_loc, target_loc);
    // float offset_u_cm = (target_loc.alt - copter.current_loc.alt); // cm

    // Vector3f dist_vec_offs_neu(offset_ne_cm.x, offset_ne_cm.y, offset_u_cm);

    // // 4. 目标速度（NED -> NEU，cm/s）
    // Vector3f vel_target_neu_cms(pkt.target_velocity * cosf(radians(pkt.target_yaw)) * 100.0f,
    //                             pkt.target_velocity * sinf(radians(pkt.target_yaw)) * 100.0f,
    //                             0.0f); // 若以后有 vz 再补充

    // // 5. P 控制器：位置误差 -> 期望速度
    // const float kp = 1.0f; // 可调，也可放参数 FOLL_POS_P
    // Vector3f desired_velocity_neu_cms = dist_vec_offs_neu * kp;

    // // 6. 叠加目标速度
    // desired_velocity_neu_cms += vel_target_neu_cms;

    // // 7. 水平/垂直限速（复用原逻辑）
    // Vector2f des_xy(desired_velocity_neu_cms.x, desired_velocity_neu_cms.y);
    // des_xy.limit_length(pos_control->get_max_speed_xy_cms());
    // desired_velocity_neu_cms.x = des_xy.x;
    // desired_velocity_neu_cms.y = des_xy.y;

    // desired_velocity_neu_cms.z = constrain_float(
    //     desired_velocity_neu_cms.z, -fabsf(pos_control->get_max_speed_down_cms()), pos_control->get_max_speed_up_cms());

    // // 8. 避障限速（可选，保留）
    // copter.avoid.adjust_velocity(desired_velocity_neu_cms, pos_control->get_pos_xy_p().kP().get(),
    //                              pos_control->get_max_accel_xy_cmss(), pos_control->get_pos_z_p().kP().get(),
    //                              pos_control->get_max_accel_z_cmss(), G_Dt);

    // // 9. 航向：直接采用包里的 desired_yaw
    // bool use_yaw = true;
    // float yaw_cd = pkt.desired_yaw * 100.0f; // deg -> centideg

    // // 10. 日志输出 10 Hz
    // uint32_t now = AP_HAL::millis();
    // bool log_request = (now - last_log_ms >= 100) || (last_log_ms == 0);
    // last_log_ms = now;

    // // 11. 交给 Guided 底层
    // ModeGuided::set_velocity(desired_velocity_neu_cms, use_yaw, yaw_cd, false, 0.0f, false, log_request);
    // ModeGuided::run();
}

#endif // MODE_FOLLOW_ENABLED