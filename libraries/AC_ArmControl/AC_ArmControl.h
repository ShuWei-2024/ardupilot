#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

class AC_ArmControl{
public:   
    void pwm_output();
    void set_servo_smooth(SRV_Channel* ch, uint16_t target_angle);
    void pwm_init();
    void output1();
    void output2();
    void output3();   
    void output4(); 
    void output5(); 

    enum  Pose{
        POSE0 = 0, //initial
        POSE1 = 1, //grasp_open
        POSE2 = 2, //throw
        POSE3 = 3, 
        POSE4 = 4,
        POSE5 = 5,//define angle
        
        
    };

    uint16_t angel_to_pwm(float angle);
    float pwm_to_angle(uint16_t pwm);
    static const struct AP_Param::GroupInfo var_info[];

protected:
    uint16_t get_speed() const{ return _speed; }
    uint8_t get_pose() const{ return _pose; }

    float get_angle(SRV_Channel *ch) { return pwm_to_angle(ch->get_output_pwm()); }

private:  
    SRV_Channel *ch6 = SRV_Channels::srv_channel(6); //servo7~10
    SRV_Channel *ch7 = SRV_Channels::srv_channel(7);
    SRV_Channel *ch8 = SRV_Channels::srv_channel(8);
    SRV_Channel *ch9 = SRV_Channels::srv_channel(9);

    const uint16_t delay_ms = 20;  // 每次更新的时间间隔

    AP_Int16 _angle6;
    AP_Int16 _angle7;
    AP_Int16 _angle8;
    AP_Int16 _angle9;
    AP_Int16 _pose;
    AP_Int16 _speed;
};