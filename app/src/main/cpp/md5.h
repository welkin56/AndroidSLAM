// MD5（RFC 1321）实现，替代 OpenSSL 的 <openssl/md5.h>。
//
// ORB-SLAM3 用 MD5 给词袋文件算校验和，而 NDK 没有公开的 OpenSSL/libcrypto。
// 函数名和签名与 OpenSSL 一致，System.cc 只需替换 include。
// 输出与 OpenSSL 一致，实现见 md5.cpp。
#ifndef SLAMAPP_MD5_H
#define SLAMAPP_MD5_H

#include <stddef.h>
#include <stdint.h>

#define MD5_DIGEST_LENGTH 16
#define MD5_LBLOCK 16

typedef struct MD5_CTX {
    uint32_t state[4];
    uint64_t len;      // 已输入的字节数，收尾时换算成 bit 数
    size_t   buflen;   // buffer 中当前字节数
    uint8_t  buffer[64];
} MD5_CTX;

#ifdef __cplusplus
extern "C" {
#endif

int MD5_Init(MD5_CTX *c);
int MD5_Update(MD5_CTX *c, const void *data, size_t len);
int MD5_Final(unsigned char *md, MD5_CTX *c);

#ifdef __cplusplus
}
#endif

#endif  // SLAMAPP_MD5_H
