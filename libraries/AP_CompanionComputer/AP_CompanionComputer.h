#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_AHRS/AP_AHRS.h>
#include "AP_CompanionComputer_config.h"

class AP_CompanionComputer {
public:
    AP_CompanionComputer();

    /* Do not allow copies */
    AP_CompanionComputer(const AP_CompanionComputer &other) = delete;
    AP_CompanionComputer &operator=(const AP_CompanionComputer&) = delete;
    
    // get singleton instance 
    static AP_CompanionComputer *get_singleton(){return _singleton;}
    
    void init();
    void update();
    void send_data();
    
    bool is_new_mode() { return _new_mode_flag; }
    bool is_new_param() { return _new_param_flag; }
    void clear_new_mode_flag() { _new_mode_flag = false; }
    void clear_new_param_flag() { _new_param_flag = false; }

    AP_Int8 enabled() { return _enable; };
    
    const CompanionReceivePacket& get_received_packet() const { return _received_packet; }
    const Mode1Param& get_mode1_param() const { return _param; }
    void set_c2hc_log_bit(uint32_t log_c2hc_bit) { _log_c2hc_bit = log_c2hc_bit; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_CompanionComputer *_singleton;
    
    // Parameters
    AP_Int8 _enable;
    AP_Int8 _port_index; // the index of instance for companion computer
    
    // UART related
    AP_HAL::UARTDriver *_uart;
    
    // receive state machine
    enum class RxState {
        WAITING_HEADER1,
        WAITING_HEADER2,
        WAITING_SOURCE,
        WAITING_TYPE,
        WAITING_LENGTH,
        RECEIVING_DATA
    } _rx_state;

    // received buff
    CompanionReceivePacket _received_packet;
    std::array<uint8_t, COMPANION_RECV_TOTAL_LENGTH> _rx_buffer;
    uint8_t _rx_count;
    uint32_t _rx_start_time;
    uint32_t _last_sent_ms;
    
    // Command info
    uint8_t _cmd_source;
    uint8_t _cmd_type;
    uint8_t _data_len;
    
    // Mode and parameter flags
    uint8_t _last_ctrl_mode = 9; // 初始值不等于任何模式
    bool _new_mode_flag = false;
    bool _new_param_flag = false;
    
    // Logging
    uint32_t _log_c2hc_bit = -1;
    
    // Configuration constants
    constexpr static uint32_t PACKET_TIMEOUT_MS = 200;
    
    // Processing functions
    void process_received_data(uint8_t oneByte);
    void parse_flight_control_data();
    void parse_parameter_data();
    void parse_system_command();
    void send_response(uint8_t data, uint8_t status);
    
    // Utility functions
    uint8_t calculate_checksum(const uint8_t *data, uint8_t len) const;
    bool validate_packet() const;
    void Log_C2HC() const;
    
    // Data structures
    Mode1Param _param;
};

namespace AP {
  AP_CompanionComputer &companioncomputer();
};