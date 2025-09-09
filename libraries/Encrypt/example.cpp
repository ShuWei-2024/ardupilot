#include "encrypt.hpp"
#include <stdio.h>
#include <string.h>

static const uint8_t key[16] = {0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
                                0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};
static const uint8_t iv[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                               0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};
int main() {
    AES_Encrypt ss(key, iv);
    ss.init();

    /* 加密 */
    uint8_t frame[300];
    size_t n = ss.encrypt_frame((uint8_t *)"hello", 5, frame, sizeof(frame));

    /* 逐字节喂 */
    for (size_t i = 0; i < n; ++i)
        ss.feed_rx(frame[i]);

    /* 解密 */
    uint8_t buf[300];
    size_t m = ss.parse(buf, sizeof(buf));
    printf("%.*s\n", m, buf);
    return 0;
}
