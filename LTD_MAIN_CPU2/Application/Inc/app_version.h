#ifndef APP_VERSION_H
#define APP_VERSION_H

#include <stdint.h>

/* CPU2主控板固件版本号：主版本.次版本.修订号.构建号 */
#define CPU2_APP_VERSION_MAJOR      1u /* CPU2 应用主版本号。 */
#define CPU2_APP_VERSION_MINOR      20u /* CPU2 应用次版本号。 */
#define CPU2_APP_VERSION_PATCH      1u /* CPU2 应用修订版本号。 */
#define CPU2_APP_VERSION_BUILD      0u /* CPU2 应用构建版本号。 */
/* 32位版本编码：0xMMmmppbb，例如 V1.2.3 build 4 = 0x01020304 */
/* CPU2 应用版本号的 32 位打包值。 */
#define CPU2_APP_VERSION_U32 \
    (((uint32_t)CPU2_APP_VERSION_MAJOR << 24) | \
     ((uint32_t)CPU2_APP_VERSION_MINOR << 16) | \
     ((uint32_t)CPU2_APP_VERSION_PATCH << 8)  | \
     ((uint32_t)CPU2_APP_VERSION_BUILD))

#define CPU2_APP_VERSION_STRING "V1.20.1.0" /* CPU2 应用版本字符串。 */
/* 固件版本仅用于显示和追踪发布；跨CPU能力兼容由DeviceParameters.protocolVersion判断。 */

/* 兼容旧命名，现有代码仍可继续使用 APP_VERSION_*。 */
#define APP_VERSION_MAJOR  CPU2_APP_VERSION_MAJOR /* 兼容旧代码的应用主版本号别名。 */
#define APP_VERSION_MINOR  CPU2_APP_VERSION_MINOR /* 兼容旧代码的应用次版本号别名。 */
#define APP_VERSION_PATCH  CPU2_APP_VERSION_PATCH /* 兼容旧代码的应用修订版本号别名。 */
#define APP_VERSION_BUILD  CPU2_APP_VERSION_BUILD /* 兼容旧代码的应用构建版本号别名。 */
#define APP_VERSION_U32    CPU2_APP_VERSION_U32 /* 兼容旧代码的应用版本打包值别名。 */
#define APP_VERSION_STRING CPU2_APP_VERSION_STRING /* 兼容旧代码的应用版本字符串别名。 */

#endif /* APP_VERSION_H */
