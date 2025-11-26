#include "AC_ArmControl.h"
#include "AC_ArmControl_Serial.h"

const AP_Param::GroupInfo AC_ArmControl::var_info[] = {

    AP_GROUPINFO("_6", 1, AC_ArmControl, _angle6, 90),

    AP_GROUPINFO("_7", 2, AC_ArmControl, _angle7, 90),

    AP_GROUPINFO("_8", 3, AC_ArmControl, _angle8, 90),

    AP_GROUPINFO("_9", 4, AC_ArmControl, _angle9, 90),

    AP_GROUPINFO("_Pose", 5, AC_ArmControl, _pose, 0),

    AP_GROUPINFO("_Speed", 6, AC_ArmControl, _speed, 1000),

    AP_GROUPEND
};

void AC_ArmControl::pwm_output()
{
    for(uint8_t i = 6; i < 10; i++)
    {
        SRV_Channel *ch = SRV_Channels::srv_channel(i);

        if (ch == nullptr) return;

        if(get_pose() == Pose::POSE0){
            pwm_init();
        }else if(get_pose() == Pose::POSE1){
            output1();
        }
        else if(get_pose() == Pose::POSE2){
            output2();
        }
        else if(get_pose() == Pose::POSE3){
            output3();}
        else if(get_pose() == Pose::POSE4){
            output4();}
        else if(get_pose() == Pose::POSE5){
            output5();}    
        else{
            set_servo_smooth(ch, pwm_to_angle(ch->get_output_pwm())); //Keep the original Angle unchanged             
        }  

        // gcs().send_text(MAV_SEVERITY_INFO, "angle%d: %d", i, ch->get_output_pwm());   
        // gcs().send_text(MAV_SEVERITY_INFO, "angle%d: %.2f",  i, get_angle(ch));

        
    }
   
}

void AC_ArmControl::set_servo_smooth(SRV_Channel* ch, uint16_t target_angle)
{
    if (ch == nullptr) return;

    if (ch->get_output_pwm() < 500) {  
        ch->set_output_pwm(500);      
    }
     if (ch->get_output_pwm() > 2500) {  
         ch->set_output_pwm(2500);      
    }

    uint16_t target_pwm = angel_to_pwm(target_angle);
    uint16_t current_pwm = ch->get_output_pwm();

    int16_t pwm_diff = target_pwm - current_pwm;  

    if (pwm_diff == 0) return; 

    int16_t step_size = (get_speed() * delay_ms / 1000) * (pwm_diff > 0 ? 1 : -1);
    
    current_pwm += step_size;

    if ((pwm_diff > 0 && current_pwm > target_pwm) || 
        (pwm_diff < 0 && current_pwm < target_pwm)) {
        current_pwm = target_pwm;
    }

    ch->set_output_pwm(current_pwm);
    hal.scheduler->delay(delay_ms);  
}

void AC_ArmControl::pwm_init()
{ 
    set_servo_smooth(ch6, 150);
    set_servo_smooth(ch7, 150);
    set_servo_smooth(ch8, 0);
    set_servo_smooth(ch9, 90);
}

void AC_ArmControl::output1() //grasp-open
{
    set_servo_smooth(ch6, 60);
    set_servo_smooth(ch7, 60);
    set_servo_smooth(ch8, 45);
    set_servo_smooth(ch9, 120);
}

void AC_ArmControl::output2() //grasp-close
{
    set_servo_smooth(ch6, 105);
    set_servo_smooth(ch7, 105);
    set_servo_smooth(ch8, 45);
    set_servo_smooth(ch9, 120);
}

void AC_ArmControl::output3() //fly
{
    set_servo_smooth(ch6, 105);
    set_servo_smooth(ch7, 105);
    set_servo_smooth(ch8, 0);
    set_servo_smooth(ch9, 90);
}

void AC_ArmControl::output4() //throw
{
    set_servo_smooth(ch6, 60);
    set_servo_smooth(ch7, 60);
    set_servo_smooth(ch8, 0);
    set_servo_smooth(ch9, 90);
}

void AC_ArmControl::output5()
{
    set_servo_smooth(ch6, _angle6);
    set_servo_smooth(ch7, _angle7);
    set_servo_smooth(ch8, _angle8);
    set_servo_smooth(ch9, _angle9);
}

uint16_t AC_ArmControl::angel_to_pwm(float angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    uint16_t pwm_value = 500 + static_cast<int>((angle * (2500 - 500)) / 180.0f);

    return pwm_value;
}

float AC_ArmControl::pwm_to_angle(uint16_t pwm)
{
    if (pwm < 500) pwm = 500;
    if (pwm > 2500) pwm = 2500;

    float angle = (pwm - 500) * 180.0f / (2500 - 500);

    return angle;
}