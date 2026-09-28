#ifndef __DISPLAY_LANGUAGE_H__
#define __DISPLAY_LANGUAGE_H__

#include <stdbool.h>
#include <stdint.h>

/* 语言编号会写入FRAM；已经发布的编号不得重排或复用。 */
typedef enum {
    LANGUAGE_CHINESE = 0,
    LANGUAGE_ENGLISH = 1,
    LANGUAGE_RUSSIAN = 2,
    LANGUAGE_COUNT
} LANGUAGE_TYPE;

/* 文本域决定目标页面是否允许使用选中的语言。 */
typedef enum {
    DISPLAY_TEXT_DOMAIN_LEGACY = 0,
    DISPLAY_TEXT_DOMAIN_STATUS
} DisplayTextDomain;

/*
 * 函数用途：判断原始语言编号是否属于当前固件支持范围。
 * 调用场景：参数写入校验和FRAM加载归一化。
 * 关键约束：非法编号不得直接作为显示数组下标。
 */
bool DisplayLanguage_IsValid(int32_t raw_language);

/*
 * 函数用途：把原始语言编号归一化为当前固件可安全使用的语言。
 * 调用场景：读取持久化值、状态快照或显示运行态时。
 * 关键约束：非法编号统一回退英文，避免误选中文或数组越界。
 */
LANGUAGE_TYPE DisplayLanguage_Normalize(int32_t raw_language);

/*
 * 函数用途：按照页面文本域解析最终显示语言。
 * 调用场景：普通双语页面和支持俄文的设备状态页选择文案时。
 * 关键约束：普通页面只支持中英文，俄文及以后新增语言均回退英文。
 */
LANGUAGE_TYPE DisplayLanguage_Resolve(int32_t raw_language, DisplayTextDomain domain);

/*
 * 函数用途：返回现有中英双语数组可安全使用的列号。
 * 调用场景：普通菜单继续复用既有二维双语表时。
 * 关键约束：返回值始终只能为0或1。
 */
uint8_t DisplayLanguage_GetLegacyColumn(int32_t raw_language);

/*
 * 函数用途：判断当前选择是否为俄文。
 * 调用场景：设备状态页启用俄文专用布局或故障显示规则时。
 * 关键约束：判断前先按统一规则归一化非法值，不得直接比较未校验的持久化字节。
 */
bool DisplayLanguage_IsRussian(int32_t raw_language);

/*
 * 函数用途：从中英文文本中选择普通页面最终文案。
 * 调用场景：不使用二维数组的旧页面文本选择。
 * 关键约束：英文为空时回退中文，俄文模式按普通页面策略显示英文。
 */
const char *DisplayLanguage_SelectLegacyText(int32_t raw_language,
                                             const char *chinese,
                                             const char *english);

/*
 * 函数用途：从中英俄文本中选择设备状态页最终文案。
 * 调用场景：状态标题、字段标签和状态页专用提示。
 * 关键约束：目标翻译为空时先回退英文，再回退中文。
 */
const char *DisplayLanguage_SelectStatusText(int32_t raw_language,
                                             const char *chinese,
                                             const char *english,
                                             const char *russian);

#endif
