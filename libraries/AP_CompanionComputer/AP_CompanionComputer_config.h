#include <AP_HAL/AP_HAL.h>
#pragma once

/************接收通信协议*********************************
// format of serial packets received from companion computer
//
// Data Bit             Definition          Description
// ------------------------------------------------
// byte 0               FRAME_HEADER1        0xA5
// byte 1               FRAME_HEADER2        0x5A
// byte 2               command source      0xAA
// byte 3               command content     0x01
// byte 4               DATA_LENGTH         frome byte5 to byte below all these data totle length of byte, default is 0x1B
// byte 5               ctrl mode           0x01:distance mode;  0x02:hover mode;  0x03:guide mode
// byte 6               X_axis_err_L        X axis error low 8 bits
// byte 7               X_axis_err_H        X axis error high 8 bits
// bute 8               Y_axis_err_L        Y axis error low 8 bits
// byte 9               Y_axis_err_H        Y axis error high 8 bits
// bute 10              Z_axis_err_L        Z axis error low 8 bits
// byte 11              Z_axis_err_H        Z axis error high 8 bits
// byte 12              max_velocity_L     feihu quadrotor could reach the fastest velocity low 8 bits
// byte 13              max_velocity_H     feihu quadrotor could reach the fastest velocity high 8 bits
// byte 14              YAW_L               desired yaw data low 8 bits
// byte 15              YAW_H               desired yaw data high 8 bits
// byte 16              Target_lon8         target uav's longitude low 8 bits
// byte 17              Target_lon16        target uav's longitude low 8-16 bits
// byte 18              Target_lon24        target uav's longitude high  16-24 bits
// byte 19              Target_lon32        target uav's longitude high 8 bits
// byte 20              Target_lat8         target uav's latitude low 8 bits
// byte 21              Target_lat16        target uav's latitude low 8-16 bits
// byte 22              Target_lat24        target uav's latitude high  16-24 bits
// byte 23              Target_lat32        target uav's latitude high 8 bits
// byte 24              Target_alt_L        target uav's altitude low 8 bits
// byte 25              Target_alt_H        target uav's altitude high 8 bits
// byte 26              Target_yaw_L        target uav's yaw low 8 bits
// byte 27              Target_yaw_H        target uav's yaw high 8 bits
// byte 28              Target_Vel_L        target quadrotor could reach the fastest velocity low 8 bits
// byte 29              Target_Vel_H        target quadrotor could reach the fastest velocity high 8 bits
// byte 30              YAW_Max_L           target quadrotor could reach the fastest yaw rate low 8 bits
// byte 31              YAW_Max_H           target quadrotor could reach the fastest yaw rate high 8 bits
// byte 32              Checksum            low 8 bits of Checksum byte, sum of bytes 0 to bytes 8
// byte 33              end sign            the end of the packets, default is 0xFF

*********************************************/

/************发送通信协议*********************************    
// format of serial packets send to companion computer
//
// Data Bit             Definition          Description
// ------------------------------------------------
// byte 0               FRAME_HEADER1       0xA5
// byte 1               FRAME_HEADER2       0x5A
// byte 2               command source      0xBB: flight controller
// byte 3               command content     0x01
// byte 4               DATA_LENGTH         frome byte5 to byte below all these data totle length of byte, default is 0x14
// byte 5               ctrl mode           0x01:distance mode;  0x02:hover mode;  0x03:guide mode
// byte 6               Battery_Percent     1~100  %
// byte 7               My_lon8             my uav's longitude low 8 bits
// byte 8               My_lon16            my uav's longitude low 8-16 bits
// byte 9               My_lon24            my uav's longitude high  16-24 bits
// byte 10              My_lon32            my uav's longitude high 8 bits
// byte 11              My_lat8             my uav's latitude low 8 bits
// byte 12              My_lat16            my uav's latitude low 8-16 bits
// byte 13              My_lat24            my uav's latitude high  16-24 bits
// byte 14              My_lat32            my uav's latitude high 8 bits
// byte 15              My_alt_L            my uav's altitude low 8 bits
// byte 16              My_alt_H            my uav's altitude high 8 bits
// byte 17              My_Vel_L            my uav's current velocity low 8 bits
// byte 18              My_Vel_H            my uav's current velocity high 8 bits
// byte 19              My_Roll_L           my uav's current roll angle low 8 bits
// byte 20              My_Roll_H           my uav's current roll angle high 8 bits
// byte 21              My_Pitch_L          my uav's current Pitch angle low 8 bits
// byte 22              My_Pitch_H          my uav's current Pitch angle high 8 bits
// byte 23              My_Yaw_L            my uav's current Yaw angle low 8 bits
// byte 24              My_Yaw_H            my uav's current Yaw angle high 8 bits// byte 6               
// byte 25              Checksum            low 8 bits of Checksum byte, sum of bytes 0 to bytes 8
// byte 26              end sign            the end of the packets, default is 0xFF

*********************************************/


// 协议常量定义
#define COMPANION_FRAME_HEADER1         0xA5
#define COMPANION_FRAME_HEADER2         0x5A
#define COMPANION_CMD_SOURCE_RCVR       0xAA
#define COMPANION_CMD_SOURCE_FC         0xBB
#define COMPANION_END_SIGN              0xFF
#define COMPANION_CMD_CONTENT           0x01
#define COMPANION_RECV_TOTAL_LENGTH     36
#define COMPANION_RECV_DATA_LENGTH      0x1D // length of receive Data for Byte of companion_computer is 27
#define COMPANION_SEND_TOTAL_LENGTH     29
#define COMPANION_SEND_DATA_LENGTH      0x16 // length of send Data for Byte of companion_computer is 20

// 控制模式枚举
enum class CompanionCtrlMode : uint8_t {
    CTRL_MODE_DISTANCE = 0x01,
    CTRL_MODE_HOVER    = 0x02,
    CTRL_MODE_GUIDE    = 0x03
};

// 接收包结构
#pragma pack(push, 1)
typedef struct {
    uint8_t header1;
    uint8_t header2;
    uint8_t cmd_source;
    uint8_t cmd_content;
    uint8_t data_length;
    uint8_t ctrl_mode;
    int16_t x_axis_err;
    int16_t y_axis_err;
    int16_t z_axis_err;
    uint16_t max_velocity;
    int16_t desired_yaw;
    int32_t target_lon;
    int32_t target_lat;
    int32_t target_alt;
    int16_t target_yaw;
    uint16_t target_velocity;
    uint16_t yaw_max_rate;
    uint8_t checksum;
    uint8_t end_sign;
} CompanionReceivePacket;
#pragma pack(pop)

// 发送包结构
#pragma pack(push, 1)
typedef struct {
    uint8_t header1;
    uint8_t header2;
    uint8_t cmd_source;
    uint8_t cmd_content;    //指令类型
    uint8_t data_length;
    uint8_t ctrl_mode;
    uint8_t battery_percent;
    int32_t my_lon;
    int32_t my_lat;
    int32_t my_alt;
    uint16_t my_velocity;
    int16_t my_yaw;
    int16_t my_roll;
    int16_t my_pitch;
    uint8_t checksum;
    uint8_t end_sign;
} CompanionSendPacket;
#pragma pack(pop)

struct ParsedPacket
{
    uint8_t ctrl_mode;
    int16_t x_axis_err;
    int16_t y_axis_err;
    int16_t z_axis_err;
    float max_velocity;
    float desired_yaw;
    double_t target_lon;
    double_t target_lat;
    float target_alt;
    float target_yaw;
    float target_velocity;
    float yaw_max_rate;
};
