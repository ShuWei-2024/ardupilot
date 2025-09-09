#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_AHRS/AP_AHRS.h>
#include "AP_CompanionComputer_config.h"
#include "Encrypt/encrypt.hpp"

class AP_CompanionComputer {
public:
    AP_CompanionComputer();

    /* Do not allow copies */
    AP_CompanionComputer(const AP_CompanionComputer &other) = delete;
    AP_CompanionComputer &operator=(const AP_CompanionComputer&) = delete;

    void init();
    void update();
    void send_data();
    
    AP_Int8 enabled() { return _enable; };
    static const struct AP_Param::GroupInfo var_info[];
    const ParsedPacket& get_parsed_packet() const { return _parsed_packet; }    //传出数据
    void set_c2hc_log_bit(uint32_t log_c2hc_bit) { _log_c2hc_bit = log_c2hc_bit; }

private:
    constexpr static uint32_t PACKET_TIMEOUT_MS = 200;
    AP_Int8 _enable;
    AP_Int8 _port_index;        // the index of instance for companion computer
    AP_Int8 _aes;

    // receive state machine
    enum class RxState
    {
        WAITING_HEADER1,
        WAITING_HEADER2,
        WAITING_SOURCE,
        WAITING_TYPE,
        WAITING_LENGTH,
        RECEIVING_DATA
    } _rx_state;

    // received buff
    CompanionReceivePacket _received_packet;
    ParsedPacket _parsed_packet;
    std::array<uint8_t, COMPANION_RECV_TOTAL_LENGTH> _rx_buffer;//拼帧用
    std::array<uint8_t, 128> _rx_buffer_decrypted;//解密后的串口数据
    std::array<uint8_t, 128> _encrypted_buffer;     //加密的数据
    uint8_t _rx_count;
    uint32_t _rx_start_time;
    uint32_t _last_received_ms;
    uint32_t _last_sent_ms;

    void process_received_data(uint8_t oneByte);
    void parse_flight_control_data();
    void parse_parameter_data();
    void parse_system_command();
    void send_response(uint8_t data, uint8_t status);

    AP_HAL::UARTDriver *_uart;
    AES_Encrypt _aes_encryptor;
    uint8_t calculate_checksum(const uint8_t *data, uint8_t len) const;
    bool validate_packet() const;
    int print_count = 0;
    uint32_t _log_c2hc_bit = -1;
    void Log_C2HC() const;

    uint8_t _cmd_source;
    uint8_t _cmd_type;
    uint8_t _data_len;
};