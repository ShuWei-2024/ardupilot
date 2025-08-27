#include "AP_CompanionComputer.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Logger/AP_Logger.h>

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
    _rx_state = RxState::WAITING_HEADER1;
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
    case RxState::WAITING_HEADER1:
        if (oneByte == COMPANION_FRAME_HEADER1) {
            _rx_state = RxState::WAITING_HEADER2;
            _rx_start_time = now;
            _rx_count = 0;
        }
        break;

    case RxState::WAITING_HEADER2:
        if (oneByte == COMPANION_FRAME_HEADER2) {
            _rx_state = RxState::WAITING_SOURCE;
            // 存储帧头
            _rx_buffer[_rx_count++] = COMPANION_FRAME_HEADER1;
            _rx_buffer[_rx_count++] = COMPANION_FRAME_HEADER2;
        } else {
            _rx_state = RxState::WAITING_HEADER1;
        }
        break;

    case RxState::WAITING_SOURCE:
        _rx_state = RxState::WAITING_TYPE;
        _cmd_source = oneByte;
        _rx_buffer[_rx_count++] = oneByte;
        break;

    case RxState::WAITING_TYPE:
        if(oneByte > 0x03){
            _rx_state = RxState::WAITING_HEADER1;
            _rx_count = 0;
            break;
        }
        _rx_state = RxState::WAITING_LENGTH;
        _cmd_type = oneByte;
        _rx_buffer[_rx_count++] = oneByte;
        break;

    case RxState::WAITING_LENGTH:
        _rx_state = RxState::RECEIVING_DATA;
        _data_len = oneByte;
        _rx_buffer[_rx_count++] = oneByte;
        break;

    case RxState::RECEIVING_DATA:
        if (now - _rx_start_time > PACKET_TIMEOUT_MS) {
            _rx_state = RxState::WAITING_HEADER1;
            _rx_count = 0;
            break;
        }

        _rx_buffer[_rx_count++] = oneByte;
        
        // 检查是否接收到完整数据包（包括帧头、校验和、结束符）
        if (_rx_count >= (_data_len + 7)) {
            // 完整数据包接收，进行校验
            if (validate_packet(_rx_buffer)) {
                // 提取指令类型
                _cmd_type = _rx_buffer[3]; // 指令类型在第4个字节
                
                // 根据指令类型解析数据
                switch (_cmd_type) {
                case 0x01: // 飞行控制信息
                    parse_flight_control_data(_rx_buffer);
                    break;
                case 0x02: // 飞控参数设置
                    parse_parameter_data(_rx_buffer);
                    break;
                case 0x03: // 系统控制
                    parse_system_command(_rx_buffer);
                    break;
                default:
                    // 未知指令类型
                    break;
                }
            }

            _rx_state = RxState::WAITING_HEADER1;
            _rx_count = 0;
        }
        break;
    }
}

// 解析飞行控制数据
void AP_CompanionComputer::parse_flight_control_data(uint8_t* buffer)
{
    memcpy(&_received_packet, buffer, sizeof(_received_packet));
    _parsed_packet.ctrl_mode = _received_packet.ctrl_mode;
    _parsed_packet.x_axis_err = _received_packet.x_axis_err;
    _parsed_packet.y_axis_err = _received_packet.y_axis_err;
    _parsed_packet.z_axis_err = _received_packet.z_axis_err;
    _parsed_packet.max_velocity = float(_received_packet.max_velocity) / 100.0;
    _parsed_packet.desired_yaw = float(_received_packet.desired_yaw) / 100.0;
    _parsed_packet.target_lon = double(_received_packet.target_lon) / 1e7;
    _parsed_packet.target_lat = double(_received_packet.target_lat) / 1e7;
    _parsed_packet.target_alt = float(_received_packet.target_alt) / 100.0;
    _parsed_packet.target_yaw = float(_received_packet.target_yaw) / 100.0;
    _parsed_packet.target_velocity = float(_received_packet.target_velocity) / 100.0;
    _parsed_packet.yaw_max_rate = float(_received_packet.yaw_max_rate) / 1000.0;

#if HAL_LOGGING_ENABLED
    Log_C2HC();
#endif
}

// 解析参数设置数据（待实现）
void AP_CompanionComputer::parse_parameter_data(uint8_t *buffer)
{
    // 根据具体参数格式实现解析逻辑
    // 示例：提取参数索引和值
    // uint8_t param_index = buffer[3]; // 假设参数索引在第四个字节
    // float param_value = *reinterpret_cast<float*>(&buffer[4]); // 假设参数值从第五个字节开始
    // set_parameter(param_index, param_value);
    send_response(0x01, 0x01);
}


// 解析系统控制命令
void AP_CompanionComputer::parse_system_command(uint8_t* buffer)
{
    // 提取命令数据（假设命令数据在第四个字节）
    uint8_t cmd_data = buffer[3];
    uint8_t status = 0x01;
    switch (cmd_data) {
    case 0x02: // 重启
        // 执行重启逻辑
        break;
    case 0x03: // 关机
        // 执行关机逻辑
        break;
    default:
        // 未知命令
        send_response(0xFF, 0x02);
        break;
    }
    send_response(cmd_data, status);
}

// 校验数据包（需要根据协议实现）TODO：要根据长度改
bool AP_CompanionComputer::validate_packet(const uint8_t *buffer) const
{
    // 计算校验和
    // 校验和在倒数第二个字节，结束符在最后一个字节
    uint8_t calculated_checksum = 0;
    for (int i = 0; i < _rx_count - 2; i++) {
        calculated_checksum += buffer[i];
    }

    return (calculated_checksum == buffer[_rx_count - 2]) && (buffer[_rx_count - 1] == 0xFF);
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
    pkt.ctrl_mode = _parsed_packet.ctrl_mode;

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
    // pkt.my_alt = _parsed_packet.x_axis_err;         //debug，务必删除

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

void AP_CompanionComputer::send_response(uint8_t data, uint8_t status)
{
    // 计算数据包总长度：同步字节(2) + 指令来源(1) + 指令类型(1) + 数据长度(1) + 数据(2) + 校验位(1) + 结束位(1)
    const uint8_t packet_length = 9;
    uint8_t response_buffer[packet_length];
    response_buffer[0] = COMPANION_FRAME_HEADER1; // 同步字节1
    response_buffer[1] = COMPANION_FRAME_HEADER2; // 同步字节2
    response_buffer[2] = COMPANION_CMD_SOURCE_FC; // 来自FCU
    response_buffer[3] = 0x02; // 控制指令反馈
    response_buffer[4] = 0x02; // 数据位长度
    response_buffer[5] = data;   // 收到的指令数据
    response_buffer[6] = status; // 执行状态
    // 计算校验和（从同步字节到数据结束）
    uint8_t checksum = 0;
    for (int i = 0; i < 7; i++)
    {
        checksum += response_buffer[i];
    }
    response_buffer[7] = checksum; // 校验位
    // 填充结束位
    response_buffer[8] = 0xFF; // 结束符

    // 通过串口发送数据
    _uart->write(response_buffer, packet_length);
}

uint8_t AP_CompanionComputer::calculate_checksum(const uint8_t *data, uint8_t len) const 
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum & 0xFF;
}


#if HAL_LOGGING_ENABLED
void AP_CompanionComputer::Log_C2HC() const
{
    // if (_log_c2hc_bit == uint32_t(-1)) {     //需要配合修改地面站
    //     return;
    // }
    // if (!AP::logger().should_log(_log_c2hc_bit))
    // {
    //     return;
    // }
    const struct log_c2hc pkt = {
        LOG_PACKET_HEADER_INIT(LOG_C2HC_MSG),
        time_us : AP_HAL::micros64(),
        ctrl_mode : _parsed_packet.ctrl_mode,
        x_axis_err : _parsed_packet.x_axis_err,
        y_axis_err : _parsed_packet.y_axis_err,
        z_axis_err : _parsed_packet.z_axis_err,
        max_velocity : _parsed_packet.max_velocity,
        desired_yaw : _parsed_packet.desired_yaw,
        target_lon : _parsed_packet.target_lon,
        target_lat : _parsed_packet.target_lat,
        target_alt : _parsed_packet.target_alt,
        target_yaw : _parsed_packet.target_yaw,
        target_velocity : _parsed_packet.target_velocity,
        yaw_max_rate : _parsed_packet.yaw_max_rate,
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
    
}
#endif // HAL_LOGGING_ENABLED