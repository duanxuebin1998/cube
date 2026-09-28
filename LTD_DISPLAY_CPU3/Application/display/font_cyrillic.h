#ifndef __FONT_CYRILLIC_H__
#define __FONT_CYRILLIC_H__

#include <stdint.h>

#define CYRILLIC_FONT_GLYPH_BYTES 14U
#define CYRILLIC_FONT_GLYPH_COUNT 66U

/*
 * 函数用途：按Unicode码点查找完整俄语字母表中的8x16点阵。
 * 调用场景：OLED统一UTF-8渲染器识别到俄语西里尔字母时。
 * 关键约束：仅覆盖俄语33个大小写字母并包含Ё/ё，未命中返回空指针。
 */
const uint8_t *FontCyrillic_GetGlyph(uint32_t codepoint);

#endif
