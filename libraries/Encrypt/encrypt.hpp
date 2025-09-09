#pragma once
#include "aes.h"
#include "ctr_mode.h"
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

class AES_Encrypt {
    public:
    static constexpr uint8_t SS_MAX_FRAME = 255;

    AES_Encrypt(const uint8_t *key, const uint8_t *iv) : _key(key), _iv(iv) {};

    bool init() {
        return tc_aes128_set_encrypt_key(&_aes_key, _key) == TC_CRYPTO_SUCCESS;
    }

    /* 加密：明文->完整帧 */
    size_t encrypt_frame(const uint8_t *plain, size_t plen, uint8_t *out,
                         size_t out_size) {     // 密文帧->明文
        if (plen == 0 || plen > SS_MAX_FRAME ||
            out_size < TC_AES_BLOCK_SIZE + 2 + plen)
            return 0;

        memcpy(out, _iv, TC_AES_BLOCK_SIZE); // IV
        uint8_t ctr[TC_AES_BLOCK_SIZE];
        memcpy(ctr, _iv, TC_AES_BLOCK_SIZE);
        if (tc_ctr_mode(out + TC_AES_BLOCK_SIZE + 2, plen, plain, plen, ctr,
                        &_aes_key) != TC_CRYPTO_SUCCESS)
            return 0;

        out[TC_AES_BLOCK_SIZE] = plen >> 8;
        out[TC_AES_BLOCK_SIZE + 1] = plen & 0xFF;
        return TC_AES_BLOCK_SIZE + 2 + plen;
    }

    /* 接收侧*/
    size_t decrypt_frame(const uint8_t *in, size_t in_len, uint8_t *plain,
                         size_t max_plain) {
        if (in_len < TC_AES_BLOCK_SIZE + 2)
            return 0;

        uint16_t clen =
            ((uint16_t)in[TC_AES_BLOCK_SIZE] << 8) | in[TC_AES_BLOCK_SIZE + 1];
        if (clen == 0 || clen > SS_MAX_FRAME || clen > max_plain ||
            in_len != TC_AES_BLOCK_SIZE + 2 + clen)
            return 0;

        uint8_t ctr[TC_AES_BLOCK_SIZE];
        memcpy(ctr, in, TC_AES_BLOCK_SIZE);
        if (tc_ctr_mode(plain, clen, in + TC_AES_BLOCK_SIZE + 2, clen, ctr,
                        &_aes_key) != TC_CRYPTO_SUCCESS)
            return 0;

        return clen;
    }
    void feed_rx(uint8_t byte) { // 单字节喂入
        switch (_rx_state) {
        case WAIT_HDR:
            _rx_hdr[_rx_idx++] = byte;
            if (_rx_idx == 18) { // 头收齐
                memcpy(_rx_iv, _rx_hdr, 16);
                _rx_body_len = ((uint16_t)_rx_hdr[16] << 8) | _rx_hdr[17];
                if (_rx_body_len == 0 || _rx_body_len > SS_MAX_FRAME) {
                    _rx_state = WAIT_HDR; // 非法长度，直接丢弃重收
                    _rx_idx = 0;
                } else {
                    _rx_state = WAIT_BODY;
                    _rx_idx = 0;
                }
            }
            break;

        case WAIT_BODY:
            _rx_body[_rx_idx++] = byte;
            if (_rx_idx == _rx_body_len) // body 收齐
                _rx_state = DONE;
            break;

        case DONE: // 上一帧还没取走，新字节直接丢弃
            break;
        }
    }
    
    size_t parse(uint8_t *plain, size_t max_plain) {    // 尝试解出一帧
        if (_rx_state != DONE) return 0;
    if (_rx_body_len > max_plain) { // 用户缓冲区不够
        _rx_state = WAIT_HDR;
        _rx_idx = 0;
        return 0;
    }

    uint8_t ctr[TC_AES_BLOCK_SIZE];
    memcpy(ctr, _rx_iv, TC_AES_BLOCK_SIZE);
    if (tc_ctr_mode(plain, _rx_body_len, _rx_body, _rx_body_len, ctr,
                    &_aes_key) != TC_CRYPTO_SUCCESS) {
        _rx_state = WAIT_HDR;
        _rx_idx = 0;
        return 0;
    }

    size_t ret = _rx_body_len;
    /* 复位状态机，准备下一帧 */
    _rx_state = WAIT_HDR;
    _rx_idx = 0;
    return ret;
}

private : 
    const uint8_t *_key;
    const uint8_t *_iv;
    struct tc_aes_key_sched_struct _aes_key;

    /* 拼帧状态机 */
    enum { WAIT_HDR = 0, WAIT_BODY, DONE } _rx_state = WAIT_HDR;
    uint8_t _rx_hdr[18];            // 临时存 18 B 头
    uint8_t _rx_iv[16];             // 头里解析出的 IV
    uint16_t _rx_body_len = 0;      // 期望的 body 长度
    uint16_t _rx_idx = 0;           // 当前已收字节数（头或体）
    uint8_t _rx_body[SS_MAX_FRAME]; // body 缓冲区
};