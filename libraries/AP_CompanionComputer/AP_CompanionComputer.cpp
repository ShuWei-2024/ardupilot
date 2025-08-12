#include "AP_CompanionComputer.h"
#include <AP_SerialManager/AP_SerialManager.h>


extern const AP_HAL::HAL& hal;

// 参数定义
const AP_Param::GroupInfo AP_CompanionComputer::var_info[] = {
    // @Param: _ENABLE_CC
    // @DisplayName: Enable Companion computer
    // @Description: Enable communication with companion computer
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_CompanionComputer, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: PORT
    // @DisplayName: instance of companion computer Serial Port
    // @Description: The nth serial port instance found starting from serial0 that uses serial protocol 49
    // @Values: 0:instance0, 1:instance1
    // @User: Advanced
    AP_GROUPINFO("PORT", 2, AP_CompanionComputer, _port_index, 0),

    AP_GROUPEND
};

AP_CompanionComputer::AP_CompanionComputer() 
{
    AP_Param::setup_object_defaults(this, var_info);
    _rx_state = WAITING_HEADER1;
    _rx_count = 0;
    _uart = nullptr;
}

void AP_CompanionComputer::init() 
{
    if (!_enable) {
        return;
    }
    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_2CC, 0);
    // hal.console->printf("---------companion computer find serial-------:\n");  //debug-print
    
    if (_uart != nullptr) {
        _uart->begin(115200, 512, 128);
        // hal.console->printf("---------companion uart begin-------:\n");  //debug-print
    }    
}

void AP_CompanionComputer::update()
{
    if (!_enable || _uart == nullptr)
        return;

    int16_t nbytes = _uart->available();
    while (nbytes-- > 0) {
        uint8_t byte = _uart->read();
        process_received_data(byte);
    }
}

void AP_CompanionComputer::process_received_data(uint8_t oneByte) 
{
    const uint32_t now = AP_HAL::millis();
    
    switch (_rx_state) {
    case WAITING_HEADER1:
        if (oneByte == COMPANION_FRAME_HEADER1) {
            _rx_state = WAITING_HEADER2;
            _rx_start_time = now;
        }

        break;

    case WAITING_HEADER2:
        if (oneByte == COMPANION_FRAME_HEADER2) {
            _rx_buffer[0] = COMPANION_FRAME_HEADER1;
            _rx_buffer[1] = COMPANION_FRAME_HEADER2;
            _rx_count = 2;
            _rx_state = RECEIVING_DATA;
        } else {
            _rx_state = WAITING_HEADER1;
        }

        break;

    case RECEIVING_DATA:
        if (now - _rx_start_time > PACKET_TIMEOUT_MS){
            _rx_state = WAITING_HEADER1;
            _rx_count = 0;
            break;
        }

        _rx_buffer[_rx_count++] = oneByte;
        if (_rx_count >= sizeof(_rx_buffer)) {
            // 完整数据包接收
            memcpy(&_received_packet, _rx_buffer, sizeof(_received_packet));
            
            if (validate_packet(_received_packet)) {
                _last_received_ms = now;//TODO:好像多余
                /*
                // --------------receive package------------------
                //uint8_t ctrl_mode = _rx_buffer[5];

                // //calculate image X axis error
                // int16_t imageX_err = ((uint16_t)_rx_buffer[7] << 8) | _rx_buffer[6];
                // //calculate image Y axis error
                // int16_t imageY_err = ((uint16_t)_rx_buffer[9] << 8) | _rx_buffer[8];
                // //calculate image Z axis error
                // int16_t imageZ_err = ((uint16_t)_rx_buffer[11] << 8) | _rx_buffer[10];

                // //calculate target quad-rotor longitude 
                // int32_t target_lon = ((uint32_t)_rx_buffer[19] << 24) | ((uint32_t)_rx_buffer[18] << 16) | ((uint16_t)_rx_buffer[17] << 8) | _rx_buffer[16];
                // //calculate target quad-rotor latitude 
                // int32_t target_lat = ((uint32_t)_rx_buffer[23] << 24) | ((uint32_t)_rx_buffer[22] << 16) | ((uint16_t)_rx_buffer[21] << 8) | _rx_buffer[20];
                // //calculate target quad-rotor altitude 
                // int16_t target_alt = ((uint16_t)_rx_buffer[25] << 8) | _rx_buffer[24];
                // //calculate target quad-rotor heading 
                // int16_t target_yaw = ((uint16_t)_rx_buffer[27] << 8) | _rx_buffer[26];
                // //calculate target quad-rotor velocity 
                // int16_t target_vel = ((uint16_t)_rx_buffer[29] << 8) | _rx_buffer[28];
                
                */
                if(print_count++ > 10){
                    print_count = 0;
                    hal.console->printf("companion computer uart: %u ,%u, %u;;  %ld, %ld;;  %d, %d, %d\r\n",
                                        _received_packet.x_axis_err,
                                        _received_packet.y_axis_err,
                                        _received_packet.z_axis_err,
                                        _received_packet.target_lon,
                                        _received_packet.target_lat,
                                        _received_packet.target_alt,
                                        _received_packet.target_yaw,
                                        _received_packet.target_velocity);
                }
            }//TODO:错误处理，从bak里拿回上一帧数据
            
            _rx_state = WAITING_HEADER1;
        }
        break;
    }
}

void AP_CompanionComputer::send_data() 
{
    if (!_enable || _uart == nullptr) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (now - _last_sent_ms < 20) { // 50Hz
        return;
    }

    CompanionSendPacket pkt = {0};
    pkt.header1 = COMPANION_FRAME_HEADER1;
    pkt.header2 = COMPANION_FRAME_HEADER2;
    pkt.cmd_source = COMPANION_CMD_SOURCE_FC;
    pkt.cmd_content = COMPANION_CMD_CONTENT;
    pkt.data_length = sizeof(CompanionSendPacket) - 7; // 2+2+1+2head 2 byte,  commond 2 byte, checksum 2byte, end sign 2 byte
    
    // get flight control mode
    pkt.ctrl_mode = _received_packet.ctrl_mode;

    //get battery percentage
    const AP_BattMonitor &battery = AP::battery();
    uint8_t percentage; 
    if (battery.has_cell_voltages(1)) {            
        if (AP::battery().capacity_remaining_pct(percentage, 1)){
            pkt.battery_percent = percentage;
        }else{
            pkt.battery_percent = 0;
        }
    } else {
        pkt.battery_percent = 0;
    }
    
    // get copter's position and attitude    
    Location loc;
    const auto &ahrs = AP::ahrs();
    if (ahrs.get_location(loc)) {
        pkt.my_lon = loc.lng;
        pkt.my_lat = loc.lat;
        pkt.my_alt = loc.alt / 100; // cm转m
    }
    
    // 获取速度
    Vector3f velocity;
    if (ahrs.get_velocity_NED(velocity)) {
        pkt.my_velocity = velocity.length() * 100; // m/s转cm/s
    }
    
    // 获取姿态
    const Vector3f &att = ahrs.get_rotation_body_to_ned().to_euler312();
    pkt.my_roll  = degrees(att.x) * 100; // 度转0.01度
    pkt.my_pitch = degrees(att.y) * 100;
    pkt.my_yaw   = degrees(att.z) * 100;
    
    // 计算校验和
    pkt.checksum = calculate_checksum(reinterpret_cast<uint8_t*>(&pkt), COMPANION_SEND_TOTAL_LENGTH-2); // byte0-8
    pkt.end_sign = COMPANION_END_SIGN;

    // 发送数据
    _uart->write((const uint8_t*)&pkt, sizeof(pkt));
    _last_sent_ms = now;
}

uint8_t AP_CompanionComputer::calculate_checksum(const uint8_t *data, uint8_t len) const 
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum & 0xFF;
}

bool AP_CompanionComputer::validate_packet(const CompanionReceivePacket& pkt) const 
{
    // 验证头尾
    if (pkt.header1 != COMPANION_FRAME_HEADER1 ||
        pkt.header2 != COMPANION_FRAME_HEADER2 ||
        pkt.end_sign != COMPANION_END_SIGN) {
        return false;
    }
    
    // 验证校验和
    uint8_t calc_checksum = calculate_checksum(reinterpret_cast<const uint8_t*>(&pkt), COMPANION_RECV_TOTAL_LENGTH-2);
    return (calc_checksum == pkt.checksum);
}