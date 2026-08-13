#ifndef APP_VERSION_H
/* APP_VERSION_H 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define APP_VERSION_H

#include <stdint.h>

/* CPU3显示板固件版本号：主版本.次版本.修订号.构建号 */
/* CPU3 固件主版本号；不兼容的产品级变更由该字段递增。 */
#define CPU3_APP_VERSION_MAJOR      1u
/* CPU3 固件次版本号；当前功能版本为 39。 */
#define CPU3_APP_VERSION_MINOR      39u
/* CPU3 固件修订号；用于兼容范围内的问题修复。 */
#define CPU3_APP_VERSION_PATCH      0u
/* CPU3 固件构建号；用于同一修订版下区分发布构建。 */
#define CPU3_APP_VERSION_BUILD      0u
/* 32位版本编码：0xMMmmppbb，例如 V1.2.3 build 4 = 0x01020304 */
/* 把主、次、修订和构建号各占 8 位打包成 32 位版本值；高字节为主版本，便于协议与日志进行数值比较。 */
#define CPU3_APP_VERSION_U32 \
    (((uint32_t)CPU3_APP_VERSION_MAJOR << 24) | \
     ((uint32_t)CPU3_APP_VERSION_MINOR << 16) | \
     ((uint32_t)CPU3_APP_VERSION_PATCH << 8)  | \
     ((uint32_t)CPU3_APP_VERSION_BUILD))

/* CPU3 固件的人类可读版本字符串；发布时必须与四个数值版本字段保持完全一致。 */
#define CPU3_APP_VERSION_STRING "V1.39.0.0"
/* 固件版本仅用于显示和追踪发布；跨CPU能力兼容由DeviceParameters.protocolVersion判断。 */

/* 兼容现有数字显示路径：V1.000 */
/* 兼容旧显示路径的十进制版本编码；仅用于界面显示，不能代替 32 位版本或协议版本进行兼容判断。 */
#define CPU3_APP_VERSION_DISPLAY_VALUE \
    ((int)(CPU3_APP_VERSION_MAJOR * 1000u + \
           CPU3_APP_VERSION_MINOR * 100u + \
           CPU3_APP_VERSION_PATCH * 10u + \
           CPU3_APP_VERSION_BUILD))

#endif /* APP_VERSION_H */
