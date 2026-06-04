#ifndef APP_VERSION_H
#define APP_VERSION_H

#include <stdint.h>

/* CPU2主控板固件版本号：主版本.次版本.修订号.构建号 */
#define CPU2_APP_VERSION_MAJOR      1u
#define CPU2_APP_VERSION_MINOR      11u
#define CPU2_APP_VERSION_PATCH      0u
#define CPU2_APP_VERSION_BUILD      0u
/* 32位版本编码：0xMMmmppbb，例如 V1.2.3 build 4 = 0x01020304 */
#define CPU2_APP_VERSION_U32 \
    (((uint32_t)CPU2_APP_VERSION_MAJOR << 24) | \
     ((uint32_t)CPU2_APP_VERSION_MINOR << 16) | \
     ((uint32_t)CPU2_APP_VERSION_PATCH << 8)  | \
     ((uint32_t)CPU2_APP_VERSION_BUILD))

#define CPU2_APP_VERSION_STRING "V1.11.0.0"
/* 固件版本仅用于显示和追踪发布；跨CPU能力兼容由DeviceParameters.protocolVersion判断。 */

/* 兼容旧命名，现有代码仍可继续使用 APP_VERSION_*。 */
#define APP_VERSION_MAJOR  CPU2_APP_VERSION_MAJOR
#define APP_VERSION_MINOR  CPU2_APP_VERSION_MINOR
#define APP_VERSION_PATCH  CPU2_APP_VERSION_PATCH
#define APP_VERSION_BUILD  CPU2_APP_VERSION_BUILD
#define APP_VERSION_U32    CPU2_APP_VERSION_U32
#define APP_VERSION_STRING CPU2_APP_VERSION_STRING

#endif /* APP_VERSION_H */