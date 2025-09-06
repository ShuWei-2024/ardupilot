#include "aesEncipher.hpp"
#include "stdio.h"

const AP_Param::GroupInfo AESEncipher::var_info[] = {

    // @Param: enAES
    // @DisplayName: enAES
    // @Description: Enable AESEncipher
    // @Values: 0:Disable 1:Enable
    // @User: Standard
    AP_GROUPINFO("enAES", 1, AESEncipher, _enAESEncipher, 1),

    AP_GROUPEND};

const uint8_t d_key[16] = {0x2b, 0x7e, 0x15, 0x27, 0x28, 0xae, 0xd2, 0x24,
                           0xab, 0xf7, 0x33, 0x26, 0x09, 0xcf, 0x56, 0x3c};

const uint8_t d_iv[16] = {0x6d, 0x01, 0x23, 0x03, 0x04, 0x15, 0x06, 0x66,
                          0x08, 0x79, 0x63, 0x08, 0x0c, 0x0d, 0x32, 0x64};

void AESEncipher::init(const uint8_t *iv, const uint8_t *key) {
    memset(_info.aesIv, 0, ENCRYPT_AES_IV_LEN);
    memset(_info.aesKey, 0, ENCRYPT_AES_KEY_LEN);
    if (iv == NULL) {
        memcpy(_info.aesIv, ENCRYPT_AES_IV, ENCRYPT_AES_IV_LEN);
    } else {
        memcpy(_info.aesIv, iv, ENCRYPT_AES_IV_LEN);
    }
    if (key == NULL) {
        memcpy(_info.aesKey, ENCRYPT_AES_KEY, ENCRYPT_AES_KEY_LEN);
    } else {
        memcpy(_info.aesKey, key, ENCRYPT_AES_KEY_LEN);
    }
}

AESEncipher::AESEncipher() { init(&d_iv[0], &d_key[0]); }

AESEncipher::AESEncipher(const uint8_t *iv, const uint8_t *key) {
    init(iv, key);
}

// dest_buff out:加密后的结果，注意dest_buff使用后要delete []
//			 结构: header(ENCRYPT_HEADER_LEN):len(4byte):ciphertext(len)
// src_buff in:待加密的内容
// len in:待加密的长度 out:加密后的长度(dest_buff的大小)
bool AESEncipher::aes_encrypt(uint8_t *&dest_buff, const uint8_t *src_buff,
                              uint32_t &len) {
    if (!tc_aes128_set_encrypt_key(&_info.aes_ctx, _info.aesKey)) {
        return false;
    }

    src_buff = padding(src_buff, len);

    uint8_t *dest = new uint8_t[ENCRYPT_HEADER_LEN + sizeof(uint32_t) + len];
    uint8_t *dest_bak = new uint8_t[ENCRYPT_AES_IV_LEN + len];

    if (!tc_cbc_mode_encrypt(dest_bak, len + ENCRYPT_AES_IV_LEN, src_buff, len,
                             (uint8_t *)_info.aesIv, &_info.aes_ctx)) {
        delete[] dest;
        delete[] dest_bak;
        delete[] src_buff;

        return false;
    }

    memcpy(dest, ENCRYPT_PARSE_HEADER, ENCRYPT_HEADER_LEN);
    memcpy(&dest[ENCRYPT_HEADER_LEN], &len, sizeof(len));
    memcpy(&dest[ENCRYPT_HEADER_LEN + sizeof(len)],
           &dest_bak[ENCRYPT_AES_IV_LEN], len);

    dest_buff = dest;
    len = ENCRYPT_HEADER_LEN + sizeof(uint32_t) + len;

    delete[] dest_bak;
    delete[] src_buff;

    return true;
}

// src_buff	in:待解密的内容，注意不包含 iv
// dest_buff out:如果函数返回true，要 delete[]
// len in:src_buff的大小 out:dest_buff的大小
bool AESEncipher::aes_decrypt(uint8_t *&dest_buff, const uint8_t *src_buff,
                              uint32_t &len) {
    if (!tc_aes128_set_decrypt_key(&_info.aes_ctx, _info.aesKey)) {
        return false;
    }

    uint8_t *src = new uint8_t[ENCRYPT_AES_IV_LEN + len];
    uint8_t *dest = new uint8_t[len + ENCRYPT_AES_IV_LEN];

    memcpy(src, (uint8_t *)_info.aesIv, ENCRYPT_AES_IV_LEN);
    memcpy(&src[ENCRYPT_AES_IV_LEN], src_buff, len);

    if (!tc_cbc_mode_decrypt(dest, len + ENCRYPT_AES_IV_LEN,
                             &src[ENCRYPT_AES_IV_LEN], len + ENCRYPT_AES_IV_LEN,
                             src, &_info.aes_ctx)) {
        delete[] dest;
        delete[] src;

        return false;
    }

    unpadding(dest, len);

    delete[] src;
    dest_buff = dest;

    return true;
}

// return 要 delete[]
uint8_t *AESEncipher::padding(const uint8_t *str, uint32_t &len) {
    uint8_t add = ENCRYPT_SINGLE_SIZE - (len % ENCRYPT_SINGLE_SIZE);

    uint8_t *ret = new uint8_t[len + add];

    memcpy(ret, str, len);
    memset(&ret[len], add, add);

    len += add;

    return ret;
}

void AESEncipher::unpadding(const uint8_t *str, uint32_t &len) {
    uint8_t todel = str[len - 1];

    len -= todel;
}