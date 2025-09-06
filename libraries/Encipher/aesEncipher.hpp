#pragma once

#include "staticQueue.hpp"
#include <string.h>
extern "C" {
#include <Tinycrypt/cbc_mode.h>
}

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#define ENCRYPT_SINGLE_SIZE 16             // 加密单包长度, 小于255
#define ENCRYPT_AES_IV_LEN 16              // AES向量长度
#define ENCRYPT_AES_KEY_LEN 16             // AES密钥长度
#define ENCRYPT_AES_IV "0123456789ABCDEF"  // AES向量
#define ENCRYPT_AES_KEY "0123456789ABCDEF" // AES密钥
#define ENCRYPT_PARSE_HEADER "f@g#"        // 包头
#define ENCRYPT_HEADER_LEN 4               // 包头长度

class AESEncipher {
  public:
    AESEncipher();
    AESEncipher(const uint8_t *iv, const uint8_t *key);

    struct AESEncryptInfo {
        uint8_t aesIv[ENCRYPT_AES_IV_LEN];
        uint8_t aesKey[ENCRYPT_AES_KEY_LEN];
        tc_aes_key_sched_struct aes_ctx;
    };

    // dest_buff out:加密后的结果，注意dest_buff使用后要delete []
    //			 结构: header(ENCRYPT_HEADER_LEN):len(4byte):ciphertext(len)
    // src_buff in:待加密的内容
    // len in:待加密的长度 out:加密后的长度(dest_buff的大小)
    bool aes_encrypt(uint8_t *&dest_buff, const uint8_t *src_buff,
                     uint32_t &len);

    // src_buff	in:待解密的内容，注意不包含 iv
    // dest_buff out:如果函数返回true，要 delete[]
    // len in:src_buff的大小 out:dest_buff的大小
    bool aes_decrypt(uint8_t *&dest_buff, const uint8_t *src_buff,
                     uint32_t &len);

    bool isEnable() { return _enAESEncipher; }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

  private:
    void init(const uint8_t *iv, const uint8_t *key);

    // return 要 delete[]
    uint8_t *padding(const uint8_t *str, uint32_t &len);
    void unpadding(const uint8_t *str, uint32_t &len);

    AESEncryptInfo _info;

    // parameter
    AP_Int8 _enAESEncipher;
};

#define HEADER_LENGTH ENCRYPT_HEADER_LEN
#define AES_HEADER ENCRYPT_PARSE_HEADER
#define BUFFER_SIZE 800

// 用于解析加密包，在接收方使用
class AESAssembler {
  public:
    AESAssembler();
    ~AESAssembler();

    struct Message {
        uint8_t header[HEADER_LENGTH];
        uint32_t length;
        uint8_t *payload;
    };

    void prepare(const uint8_t *data, uint32_t len);
    // msg out: msg.payload 需要 delete[]
    bool parse(Message &msg);

  private:
    enum Status {
        PARSE_STATE_GOT_HEADER,
        PARSE_STATE_GOT_LENGTH,
        PARSE_STATE_GOT_FINISH_FLAGS
    } _status;

    Message _msg;

    StaticQueue<uint8_t, BUFFER_SIZE> _buffer;
};