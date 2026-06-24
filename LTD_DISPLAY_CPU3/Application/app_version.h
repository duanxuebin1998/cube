#ifndef APP_VERSION_H
#define APP_VERSION_H

#include <stdint.h>

/* CPU3显示板固件版本号：主版本.次版本.修订号.构建号 */
#define CPU3_APP_VERSION_MAJOR      1u
#define CPU3_APP_VERSION_MINOR      16u
#define CPU3_APP_VERSION_PATCH      0u
#define CPU3_APP_VERSION_BUILD      0u
/* 32位版本编码：0xMMmmppbb，例如 V1.2.3 build 4 = 0x01020304 */
#define CPU3_APP_VERSION_U32 \
    (((uint32_t)CPU3_APP_VERSION_MAJOR << 24) | \
     ((uint32_t)CPU3_APP_VERSION_MINOR << 16) | \
     ((uint32_t)CPU3_APP_VERSION_PATCH << 8)  | \
     ((uint32_t)CPU3_APP_VERSION_BUILD))

#define CPU3_APP_VERSION_STRING "V1.16.0.0"
/* 固件版本仅用于显示和追踪发布；跨CPU能力兼容由DeviceParameters.protocolVersion判断。 */

/* 兼容现有数字显示路径：V1.000 */
#define CPU3_APP_VERSION_DISPLAY_VALUE \
    ((int)(CPU3_APP_VERSION_MAJOR * 1000u + \
           CPU3_APP_VERSION_MINOR * 100u + \
           CPU3_APP_VERSION_PATCH * 10u + \
           CPU3_APP_VERSION_BUILD))

#endif /* APP_VERSION_H */
