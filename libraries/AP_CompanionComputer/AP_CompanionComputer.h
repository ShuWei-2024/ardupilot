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

    void init();

    void update();

    // accessor for uart
   // AP_HAL::UARTDriver get_uart() { return _uart; }

    AP_Int8 enabled() { return _enable; };

    void send_data();

    static const struct AP_Param::GroupInfo var_info[];

    const ParsedPacket& get_parsed_packet() const { return _parsed_packet; }    //传出数据
    
    void set_c2hc_log_bit(uint32_t log_c2hc_bit) { _log_c2hc_bit = log_c2hc_bit; }

private:
    #define PACKET_TIMEOUT_MS 200
    AP_Int8 _enable;          
    AP_Int8 _port_index;        // the index of instance for companion computer, first companion computer serial is 0:Serial0

    // receive state machine
    enum RxState
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
    uint8_t _rx_buffer[sizeof(CompanionReceivePacket)];
    uint8_t _rx_count;
    uint32_t _rx_start_time;
    uint32_t _last_received_ms;
    void process_received_data(uint8_t byte);
    void parse_flight_control_data(uint8_t *buffer);
    void parse_parameter_data(uint8_t *buffer);
    void parse_system_command(uint8_t *buffer);
    void send_response(uint8_t data, uint8_t status);

    // send counter
    uint32_t _last_sent_ms;

    AP_HAL::UARTDriver *_uart;
    // crc checksum
    uint8_t calculate_checksum(const uint8_t *data, uint8_t len) const;
    // receive validate
    bool validate_packet(const uint8_t *pkt) const;
    int print_count = 0;
    uint32_t _log_c2hc_bit = -1;
    void Log_C2HC() const;

    uint8_t _cmd_source;
    uint8_t _cmd_type;
    uint8_t _data_len;
};