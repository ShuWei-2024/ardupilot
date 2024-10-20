#pragma once

#include "AC_ForceTorque.h"
#include "AC_ForceTorque_Backend_Serial.h"

class AC_ForceTorque_2DR304_Serial : public AC_ForceTorque_Backend_Serial
{

public:

    using AC_ForceTorque_Backend_Serial::AC_ForceTorque_Backend_Serial;
    void init_serial(uint8_t serial_instance) override;
    bool sendsetzero() const override;

private:
    // get a reading
    bool get_reading(Vector3f &reading_force_N, Vector3f &reading_torque_Nm) override;

    uint8_t  linebuf[40];
    uint8_t  linebuf_len;
    uint8_t ask_dr304[8] = {0x01, 0x03, 0x0a, 0x00, 0x00, 0x0c, 0x46, 0x17};
    uint8_t sendzerobuf[13] = {0x01, 0x10, 0x0A, 0x20, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x07, 0xCE, 0xD5};
    uint8_t zerocheckbuf[8] = {0x01, 0x10, 0x0A, 0x20, 0x00, 0x02, 0x43, 0xDA};
    int empty_time = 0;
    uint8_t resolve_mode = 0; //   0:常规，1：zero_check, 2: ans_check
};