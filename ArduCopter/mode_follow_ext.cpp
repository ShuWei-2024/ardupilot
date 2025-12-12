#include "Copter.h"
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "RC_Channel.h"

#if MODE_FOLLOW_EXT_ENABLED
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
    //  @User: Advanced
    AP_GROUPINFO("KP_YAW", 2, ModeFollowExt, _kp_yaw, 0.05f),
    // @Param: FOLE_KP_THR
    // @DisplayName: Follow mode I gain on throttle
    // @Description: Proportional gain in follow mode
    // @User: Advanced
    AP_GROUPINFO("KP_THR", 3, ModeFollowExt, _kp_thr, 0.1f),
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
    gcs().send_text(MAV_SEVERITY_DEBUG, "entry FOLLOW_EXT");
    y_err = 0;
    z_err = 0;
    last_log_ms = 0;
    return ModeGuided::init(ignore_checks);
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

    /* 5. 根据 ctrl_mode 决定控制方式 */
    switch (pkt.ctrl_mode) {
    case 1: { // 视觉导引模式
        // if (cc.is_new_param()) {
        //     cc.clear_new_param_flag();
        //     Mode1Param param = cc.get_mode1_param();
        //     _kd_thr.set_and_save(param._kd_thr);
        //     _kp_thr.set_and_save(param._kp_thr);
        //     _kd_yaw.set_and_save(param._kd_yaw);
        //     _kp_yaw.set_and_save(param._kp_yaw);
        //     _speed.set_and_save(param._speed);
        //     _alpha.set_and_save(param._alpha);
        // }
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
        float yaw_rate_cds = _kp_yaw * y_err; // centideg/s
        yaw_rate_cds = constrain_float(yaw_rate_cds, -2500.0f, 2500.0f);
        float v_now = MAX(_speed, 100.0f); // 避免除0
        float v_ref = 500.0f;              // 参考速度
        float speed_ratio = v_now / v_ref; // >1 = 高速，<1 = 低速
        float k = 0.5f;
        float kp_thr_eff = _kp_thr * sqrtf(0.5f + k * speed_ratio * speed_ratio);
        kp_thr_eff = constrain_float(kp_thr_eff, _kp_thr * 0.5f, _kp_thr * 2.0f);
        float climb_rate_cms = kp_thr_eff * z_err;
        climb_rate_cms = constrain_float(climb_rate_cms, -200.0f, 200.0f);

        Vector3f vel_vector;
        vel_vector.x = _speed;
        vel_vector.y = 0;
        vel_vector.z = climb_rate_cms;
        for (uint8_t i = 0; i < 3; i++) {
            // consider velocity invalid if any component nan or >1000(m/s or m/s/s)
            if (isnan(vel_vector[i]) || fabsf(vel_vector[i]) > 3000) {
               copter.mode_guided.init(true); 
            }
        }
        copter.rotate_body_frame_to_NE(vel_vector.x, vel_vector.y);
        ModeGuided::set_velocity(vel_vector,         // NED cm/s 向下是正
                                 false, 0,            // 不指定绝对 yaw
                                 true, yaw_rate_cds,  // 指定 yaw-rate
                                 false,               // 绝对 yaw-rate
                                 ten_hz_flag);
        break;
    }
    case 3: { // 位置控制模式
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