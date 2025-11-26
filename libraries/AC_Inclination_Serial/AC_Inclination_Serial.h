#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

class AC_Inclination_Serial
{
public:

    void init_serial1(uint8_t serial_instance);
    void init_serial2(uint8_t serial_instance);

    void get_reading();
    void get_angle(AP_HAL::UARTDriver *uart, uint8_t *linebuf, uint8_t &linebuf_len, float &angle);
  
protected:

    // baudrate used during object construction:
    uint32_t initial_baudrate1(uint8_t serial_instance) const;
    uint32_t initial_baudrate2(uint8_t serial_instance) const;

    // the value 0 is special to the UARTDriver - it's "use default"
    uint16_t rx_bufsize() const { return 0; }
    uint16_t tx_bufsize() const { return 0; }
    
    AP_HAL::UARTDriver *uart1 = nullptr;
    AP_HAL::UARTDriver *uart2 = nullptr;

    uint16_t read_timeout_ms() const { return 200; }

private: 
    void Log_ICLI();

    int test = 0;

    float _angle1;
    float _angle2;
 
    uint8_t linebuf1[9];
    uint8_t linebuf_len1;
    uint8_t linebuf2[9];
    uint8_t linebuf_len2;
    uint8_t u_tx_buf[8] = {0x02,0x03,0x00,0x00,0x00,0x02,0xC4,0x38};

 


};
