// 一个标准 MD5（RFC 1321），替掉 OpenSSL 的 <openssl/md5.h>。
//
// 为什么需要它：ORB-SLAM3 的 System::CalculateCheckSum 用 OpenSSL 的 MD5 给词袋文件
// 算校验和（存/载地图时核对自己和词袋配不配套）。而安卓 NDK 里没有 OpenSSL，系统那个
// libcrypto 是 BoringSSL、不属于 NDK 公开 API，App 链不了。为了一个哈希函数去交叉编译
// 整份 OpenSSL 不值当。
//
// 函数名和签名照抄 OpenSSL，所以 thirdparty/orb_slam3/src/System.cc 里**只有一行**
// 被改动：#include <openssl/md5.h> → #include "md5.h"。
// 那几行 MD5_Init / MD5_Update / MD5_Final 一个字没动。
//
// 摘要和 OpenSSL 算出来的**逐字节相同**（实测与 md5sum 七条用例全中），
// 所以 PC 上存的地图和手机上存的能够互相认。实现见 md5.cpp。
#ifndef SLAMAPP_MD5_H
#define SLAMAPP_MD5_H

#include <stddef.h>
#include <stdint.h>

#define MD5_DIGEST_LENGTH 16
#define MD5_LBLOCK 16

typedef struct MD5_CTX {
    uint32_t state[4];
    uint64_t len;      // 已喂进来的字节数，收尾时要换成 bit 数写进尾巴
    size_t   buflen;   // buffer 里当前攒了多少字节
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
