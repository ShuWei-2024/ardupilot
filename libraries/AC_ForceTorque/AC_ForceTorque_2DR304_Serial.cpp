#include "AC_ForceTorque_2DR304_Serial.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;

#define DR304_FRAME_HEADER1 0x01    // Header1 Byte from DR304_Serial
#define DR304_FRAME_HEADER2 0x03    // Header2 Byte from DR304_Serial
#define DR304_FRAME_LENGTH 29
#define DR304_DATA_LENGTH 0x30     // length of Data for Byte of DR304_Serial

#define FORCETORQUE_FORCE_MAX_N 50000
#define FORCETORQUE_TORQUE_MAX_NM 20000 
//参数列表
// format of serial packets received from forceTorque sensor D.R304
//
// Data Bit             Definition      Description
// ------------------------------------------------
// byte 0               Frame header    0x01
// byte 1               Frame header    0x03
// byte 2               DATA_LENGTH     force and torque totle length of byte, default is 0x18
// byte 3               Fx_d3           force in x axis raw data 3 high 8 bits
// byte 4               Fx_d2           force in x axis raw data 2 high 8 bits
// byte 5               Fx_d1           force in x axis raw data 1 low 8 bits
// bute 6               Fx_d0           force in x axis raw data 0 low 8 bits
// byte 7               Fy_d3           force in y axis raw data 3 high 8 bits
// bute 8               Fy_d2           force in y axis raw data 2 high 8 bits
// byte 9               Fy_d1           force in y axis raw data 1 low 8 bits
// byte 10              Fy_d0           force in y axis raw data 0 low 8 bits
// byte 11              Fz_d3           force in z axis raw data 3 high 8 bits
// byte 12              Fz_d2           force in z axis raw data 2 high 8 bits
// byte 13              Fz_d1           force in z axis raw data 1 low 8 bits
// byte 14              Fz_d0           force in z axis raw data 0 low 8 bits
// byte 15              Tx_d3           torque in x axis raw data 3 high 8 bits
// byte 16              Tx_d2           torque in x axis raw data 2 high 8 bits
// byte 17              Tx_d1           torque in x axis raw data 1 low 8 bits
// bute 18              Tx_d0           torque in x axis raw data 0 low 8 bits
// byte 19              Ty_d3           torque in y axis raw data 3 high 8 bits
// bute 20              Ty_d2           torque in y axis raw data 2 high 8 bits
// byte 21              Ty_d1           torque in y axis raw data 1 low 8 bits
// byte 22              Ty_d0           torque in y axis raw data 0 low 8 bits
// byte 23              Tz_d3           torque in z axis raw data 3 high 8 bits
// byte 24              Tz_d2           torque in z axis raw data 2 high 8 bits
// byte 25              Tz_d1           torque in z axis raw data 1 low 8 bits
// byte 26              Tz_d0           torque in z axis raw data 0 low 8 bits
// byte 27              Checksum       high 8 bits of Checksum byte, sum of bytes 0 to bytes 26
// byte 28              Checksum       low  8 bits of Checksum byte, sum of bytes 0 to bytes 26



// read - return last value measured by sensor
bool AC_ForceTorque_2DR304_Serial::get_reading(Vector3f &reading_force_N, Vector3f &reading_torque_Nm)
{
    
    if (uart == nullptr) {
        return false;
    }
    int16_t nbytes = uart->available();
    if(nbytes==0){
        empty_time++;
        if(empty_time >= 15){
            uart->write(ask_dr304, sizeof(ask_dr304));
            empty_time = 0;
            resolve_mode = 0;
            linebuf_len = 0;
        }
        return false;
    }

    float sum_fx_N = 0;
    float sum_fy_N = 0;
    float sum_fz_N = 0;
    float sum_Tx_Nm = 0;
    float sum_Ty_Nm = 0;
    float sum_Tz_Nm = 0;
    uint16_t count = 0;
    uint16_t count_out_of_positive_range = 0;
    uint16_t count_out_of_negtive_range = 0;

    // read any available lines from the inclination
    
    while (nbytes-- > 0) {
        int16_t r = uart->read();
        if (r < 0) {
            continue;
        }
        //hal.console->printf("read well\n");

        uint8_t c = (uint8_t)r;
        // if buffer is empty and this byte is 0x01, add to buffer
        if (linebuf_len == 0) {
            if (c == DR304_FRAME_HEADER1) {
                linebuf[linebuf_len++] = c;
                // hal.console->printf("0x01 well \n");  //debug
            }
        } 
        else if (linebuf_len == 1) {
            // if buffer has 1 element and this byte is 0x03, add it to buffer
            // if not clear the buffer
            if (c == DR304_FRAME_HEADER2) {
                linebuf[linebuf_len++] = c;
                // hal.console->printf("have answer 03\n");   //debug
            } else if (c == 0x10) {
                linebuf[linebuf_len++] = c;
                resolve_mode = 1;
                // hal.console->printf("clear answer 10\n");  //debug
            }else {
                linebuf_len = 0;
            }
        } 
        else if (resolve_mode == 0)
        {
            // hal.console->printf("begin cal f and t \n");
             // add character to buffer
            linebuf[linebuf_len++] = c;
            // if buffer now has 18 items try to decode it
            if (linebuf_len == DR304_FRAME_LENGTH) {
                // calculate checksum
                // hal.console->printf("linebuf_len == DR304_FRAME_LENGTH!\n"); 
                // hal.console->printf("linebuf = "); 
                // for(int j = 0; j<linebuf_len; j++)
                //     hal.console->printf("%02x ", linebuf[j]); //debug
                // hal.console->printf("\n");     
                uint16_t crc = (linebuf[DR304_FRAME_LENGTH-1]<<8) | linebuf[DR304_FRAME_LENGTH-2];
                int checkCRC = calc_crc_modbus(linebuf, DR304_FRAME_LENGTH - 2);
                // hal.console->printf("received_crc%x \n", crc);              // debug
                // hal.console->printf("%x ", checkCRC);
                // hal.console->printf("%02x ", checkCRC & 0xff);
                if (crc == calc_crc_modbus(linebuf, DR304_FRAME_LENGTH - 2))
                {
                    //输出已经收到一帧数据 
                    //hal.console->printf("ForceTorque have resieved data!"); 
                    // calculate Fx raw data.The obtained data needs to be restored using two's complement.
                    int32_t fx_raw = ((uint32_t)linebuf[3] << 24) | ((uint32_t)linebuf[4] << 16) | ((uint16_t)linebuf[5] << 8) | linebuf[6];
 
                    // calculate Fy raw data.The obtained data needs to be restored using two's complement.
                    int32_t fy_raw = ((uint32_t)linebuf[7] << 24) | ((uint32_t)linebuf[8] << 16) | ((uint16_t)linebuf[9] << 8) | linebuf[10];
                    // calculate Fz raw data.The obtained data needs to be restored using two's complement.
                    int32_t fz_raw = ((uint32_t)linebuf[11] << 24) | ((uint32_t)linebuf[12] << 16) | ((uint16_t)linebuf[13] << 8) | linebuf[14];
                    // if(fz_raw&0x80000000)fz_raw=-(static_cast<uint32_t>(~(fz_raw - 1)));
                    // calculate Tx raw data.The obtained data needs to be restored using two's complement.
                    int32_t Tx_raw = ((uint32_t)linebuf[15] << 24) | ((uint32_t)linebuf[16] << 16) | ((uint16_t)linebuf[17] << 8) | linebuf[18];
                    // calculate Ty raw data.The obtained data needs to be restored using two's complement.
                    int32_t Ty_raw = ((uint32_t)linebuf[19] << 24) | ((uint32_t)linebuf[20] << 16) | ((uint16_t)linebuf[21] << 8) | linebuf[22];
                    // calculate Tz raw data.The obtained data needs to be restored using two's complement.
                    int32_t Tz_raw = ((uint32_t)linebuf[23] << 24) | ((uint32_t)linebuf[24] << 16) | ((uint16_t)linebuf[25] << 8) | linebuf[26];
                    
                    //赋值
                    float fx = (float)(fx_raw*0.001);
                    float fy = (float)(fy_raw*0.001);
                    float fz = (float)(fz_raw*0.001);
                    float Tx = (float)(Tx_raw*0.0001);
                    float Ty = (float)(Ty_raw*0.0001);
                    float Tz = (float)(Tz_raw*0.0001);

                    // hal.console->printf("once_fz:%f\n", fz); // debug

                    if (fx > FORCETORQUE_FORCE_MAX_N || fy > FORCETORQUE_FORCE_MAX_N || fz > FORCETORQUE_FORCE_MAX_N || Tx > FORCETORQUE_TORQUE_MAX_NM || Ty > FORCETORQUE_TORQUE_MAX_NM || Tz > FORCETORQUE_TORQUE_MAX_NM) {
                        // this reading is out of positive range
                        count_out_of_positive_range++;
                    } else if((fx < - FORCETORQUE_FORCE_MAX_N) || (fy < - FORCETORQUE_FORCE_MAX_N) || (fz < - FORCETORQUE_FORCE_MAX_N) || (Tx < - FORCETORQUE_TORQUE_MAX_NM) || (Ty < - FORCETORQUE_TORQUE_MAX_NM) || (Tz < - FORCETORQUE_TORQUE_MAX_NM)){
                        // this reading is out of negtive range
                        count_out_of_negtive_range++;
                    } else {
                        // add degree to sum
                        //hal.console->printf("555inclination tilt sensor uart: %f\t, %lu\t,  %lu\r\n", roll, roll_raw, (roll_raw - ROLL_YAW_OFFSET));
                        sum_fx_N += fx;
                        sum_fy_N += fy;
                        sum_fz_N += fz;
                        sum_Tx_Nm += Tx;
                        sum_Ty_Nm += Ty;
                        sum_Tz_Nm += Tz;
                        count++;
                    }
                }
                uart->write(ask_dr304, sizeof(ask_dr304));
                // clear buffer
                linebuf_len = 0;
            }
        }
        else {
            linebuf[linebuf_len++] = c;
            if(linebuf_len == 8){
                for (uint8_t i = 2; i < 8;i++){
                    if (linebuf[i]!= zerocheckbuf[i]){
                        linebuf_len = 0;
                        resolve_mode = 0;
                        uart->write(sendzerobuf, sizeof(sendzerobuf));
                        return false;
                    }
                }
                linebuf_len = 0;
                resolve_mode = 0;
                uart->write(ask_dr304, sizeof(ask_dr304));
                return true;
            }
        }
    }
    if (count > 0) {
        // return average distance of readings
        reading_force_N.x = sum_fx_N / count;
        reading_force_N.y = sum_fy_N / count;
        reading_force_N.z = sum_fz_N / count;
        reading_torque_Nm.x = sum_Tx_Nm / count;
        reading_torque_Nm.y = sum_Ty_Nm / count;
        reading_torque_Nm.z = sum_Tz_Nm / count;
        //hal.console->printf("ForceTorque calc right fx:%f,fy:%f,fz:%f,tx:%f,ty:%f,tz:%f",
        //reading_force_N.x,reading_force_N.y,reading_force_N.z,reading_torque_Nm.x,reading_torque_Nm.y,reading_torque_Nm.z);
        return true;
    }

    if (count_out_of_positive_range > 0) {
        // if out of range readings return maximum range for the positive angle
        reading_force_N.x = FORCETORQUE_FORCE_MAX_N;
        reading_force_N.y = FORCETORQUE_FORCE_MAX_N;
        reading_force_N.z = FORCETORQUE_FORCE_MAX_N;
        reading_torque_Nm.x = FORCETORQUE_TORQUE_MAX_NM;
        reading_torque_Nm.y = FORCETORQUE_TORQUE_MAX_NM;
        reading_torque_Nm.z = FORCETORQUE_TORQUE_MAX_NM;
        hal.console->printf("ForceTorque calc high");
        return true;
    }

    if (count_out_of_negtive_range > 0) {
        // if out of range readings return maximum range for the negtive angle
        reading_force_N.x = -FORCETORQUE_FORCE_MAX_N;
        reading_force_N.y = -FORCETORQUE_FORCE_MAX_N;
        reading_force_N.z = -FORCETORQUE_FORCE_MAX_N;
        reading_torque_Nm.x = -FORCETORQUE_TORQUE_MAX_NM;
        reading_torque_Nm.y = -FORCETORQUE_TORQUE_MAX_NM;
        reading_torque_Nm.z = -FORCETORQUE_TORQUE_MAX_NM;
        hal.console->printf("ForceTorque calc low");
        return true;
    }

    // no readings so return false
    return false;
}
// if we set the serialx_protocol=50, serialx_BAUD=115 and incli_type=Type::Two_Serial_Serial,
void AC_ForceTorque_2DR304_Serial::init_serial(uint8_t serial_instance)
{
    // if (serial_instance > 0) {
    //     return;
    // }

    uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ForceTorque, serial_instance);
    if (uart != nullptr) {
        uart->begin(initial_baudrate(serial_instance), rx_bufsize(), tx_bufsize());
        hal.console->printf("ForceTorque: using serial port %d,addr:%p\n", serial_instance,&uart);    //debug
    }else
        hal.console->printf("uart init failed\n");  //debug
    uart->write(sendzerobuf, sizeof(sendzerobuf));
}

bool AC_ForceTorque_2DR304_Serial::sendsetzero() const
{
    if(uart != nullptr){
        uart->write(sendzerobuf, sizeof(sendzerobuf));
        return true;
    }
    return false;
}