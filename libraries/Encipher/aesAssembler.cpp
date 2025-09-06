#include "aesEncipher.hpp"
#include "stdio.h"

AESAssembler::AESAssembler() {
    _status = PARSE_STATE_GOT_FINISH_FLAGS;

    memset(&_msg, 0, sizeof(_msg));
}

AESAssembler::~AESAssembler() {}

void AESAssembler::prepare(const uint8_t *data, uint32_t len) {
    if (data == nullptr || len == 0) {
        return;
    }

    for (uint32_t i = 0; i < len; i++) {
        _buffer.add(data[i]);
    }
}

// msg out: msg.payload 需要 delete[]
bool AESAssembler::parse(Message &msg) {
    bool ret = false;

    if (_buffer.length() == 0) {
        return false;
    }

    if (_status == PARSE_STATE_GOT_FINISH_FLAGS) {
        while (_buffer.length() >= HEADER_LENGTH) {
            uint8_t *header = _buffer.front(HEADER_LENGTH);

            if (memcmp(header, AES_HEADER, HEADER_LENGTH) == 0) {
                memcpy(_msg.header, header, HEADER_LENGTH);
                _status = PARSE_STATE_GOT_HEADER;
                _buffer.remove(HEADER_LENGTH);
                delete[] header;
                break;
            } else {
                _buffer.remove();
                delete[] header;
            }
        }
    }

    if (_status == PARSE_STATE_GOT_HEADER) {
        if (_buffer.length() >= 4) {
            uint8_t *len = _buffer.front(4);

            memcpy(&_msg.length, len, 4);
            _buffer.remove(4);

            if (_msg.length < 100) {
                _status = PARSE_STATE_GOT_LENGTH;
            } else {
                _status = PARSE_STATE_GOT_FINISH_FLAGS;
            }

            delete[] len;
        }
    }

    if (_status == PARSE_STATE_GOT_LENGTH) {
        if ((uint32_t)_buffer.length() >= _msg.length) {
            _msg.payload = _buffer.front(_msg.length);
            _buffer.remove(_msg.length);
            _status = PARSE_STATE_GOT_FINISH_FLAGS;

            msg = _msg;
            ret = true;
        }
    }

    return ret;
}