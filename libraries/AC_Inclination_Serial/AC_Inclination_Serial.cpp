#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include "AC_Inclination_Serial.h"

extern const AP_HAL::HAL& hal;

#define INCLI_FRAME_HEADER1 0x02
#define INCLI_FRAME_HEADER2 0x03
#define INCLI_FRAME_LENGTH 9

void AC_Inclination_Serial::init_serial1(uint8_t serial_instance)
{
    uart1 = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ANGLE1, serial_instance);
    if (uart1 != nullptr) {
        uart1->begin(initial_baudrate1(serial_instance), rx_bufsize(), tx_bufsize());
    }
}

void AC_Inclination_Serial::init_serial2(uint8_t serial_instance)
{
    uart2 = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ANGLE2, serial_instance);
    if (uart2 != nullptr) {
        uart2->begin(initial_baudrate2(serial_instance), rx_bufsize(), tx_bufsize());
    }
}


uint32_t AC_Inclination_Serial::initial_baudrate1(const uint8_t serial_instance) const
{
    return AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_ANGLE1, serial_instance);
}


uint32_t AC_Inclination_Serial::initial_baudrate2(const uint8_t serial_instance) const
{
    return AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_ANGLE2, serial_instance);
}

void AC_Inclination_Serial::get_reading()
{
    if (uart1 == nullptr) {
        if (AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_ANGLE1, 0)) {
            init_serial1(0); 
        } else {
            return; 
        }
    }

    if (uart2 == nullptr) {
        if (AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_ANGLE2, 0)) {
            init_serial2(0); 
        } else {
            return; 
        }
    }
    
    if (uart1 != nullptr && uart2 != nullptr) {

        uart1->write(u_tx_buf, sizeof(u_tx_buf));
        uart2->write(u_tx_buf, sizeof(u_tx_buf));

    }else{
        return;
    }

    get_angle(uart1, linebuf1, linebuf_len1, _angle1);       
    get_angle(uart2, linebuf2, linebuf_len2, _angle2);       

    gcs().send_text(MAV_SEVERITY_INFO, "inclination angle1: %.2f", _angle1);
    gcs().send_text(MAV_SEVERITY_INFO, "inclination angle2: %.2f", _angle2);
   
    Log_ICLI();
}

void AC_Inclination_Serial::get_angle(AP_HAL::UARTDriver *uart, uint8_t *linebuf, uint8_t &linebuf_len, float &angle)
{
    int16_t nbytes = uart->available();
    while(nbytes-- > 0){
        if (uart == nullptr) {
            return;
        }
        int16_t r = uart->read();
        if(r < 0){
            continue;
        }
        uint8_t c = (uint8_t)r;

        if(linebuf_len == 0){
            if(c == INCLI_FRAME_HEADER1){
                linebuf[linebuf_len++] = c;
            }
        }else if(linebuf_len == 1){
                if(c == INCLI_FRAME_HEADER2){
                    linebuf[linebuf_len++] = c;
                }else{
                    linebuf_len = 0;
                }
        }else{
                linebuf[linebuf_len++] = c;
                if (linebuf_len == INCLI_FRAME_LENGTH) {
                    uint16_t crc = (linebuf[INCLI_FRAME_LENGTH-1]<<8) | linebuf[INCLI_FRAME_LENGTH-2];
                    uint16_t crc_cal = calc_crc_modbus(linebuf, INCLI_FRAME_LENGTH-2);
                    
                    if (crc == crc_cal) {
                        angle = (float)linebuf[3]*100 + (float)linebuf[4] + (float)linebuf[5]/100;
                    }

                    linebuf_len = 0;
                }                              
        }
    }
}

void AC_Inclination_Serial::Log_ICLI()
{
    const struct log_ANGLE pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ANGLE_MSG),
                time_us      : AP_HAL::micros64(),
                angle1       : _angle1,
                angle2       : _angle2,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));

}
