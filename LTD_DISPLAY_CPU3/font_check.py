import re
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
APP_MAIN_C = SCRIPT_DIR / "Application" / "app_main.c"
DISPLAY_C = SCRIPT_DIR / "Application" / "display" / "display.c"
DISPLAY_H = SCRIPT_DIR / "Application" / "display" / "display.h"
DISPLAY_TANKOPERA_C = SCRIPT_DIR / "Application" / "display" / "display_tankopera.c"
CPU3_SYSTEM_PARAMETER_H = (
    SCRIPT_DIR / "Application" / "system_param" / "system_parameter.h"
)
CPU2_SYSTEM_PARAMETER_H = (
    SCRIPT_DIR.parent / "LTD_MAIN_CPU2" / "Services" / "ParamStorage" / "system_parameter.h"
)
CYRILLIC_C = SCRIPT_DIR / "Application" / "display" / "font_cyrillic.c"
CYRILLIC_H = SCRIPT_DIR / "Application" / "display" / "font_cyrillic.h"
THIRD_PARTY_NOTICES = SCRIPT_DIR / "THIRD_PARTY_NOTICES.md"

# 只检查会进入 OLED 汉字显示链路的源码文件。
CHECK_FILES = (
    SCRIPT_DIR / "Application" / "display" / "display.c",
    SCRIPT_DIR / "Application" / "display" / "display_tankopera.c",
    SCRIPT_DIR / "Application" / "system_param" / "system_parameter.c",
)

SPECIAL_CHARS = "℃←？"
RUSSIAN_UPPERCASE = "АБВГДЕЁЖЗИЙКЛМНОПРСТУФХЦЧШЩЪЫЬЭЮЯ"
RUSSIAN_LOWERCASE = "абвгдеёжзийклмнопрстуфхцчшщъыьэюя"
RUSSIAN_LETTERS = frozenset(RUSSIAN_UPPERCASE + RUSSIAN_LOWERCASE)
RUSSIAN_STATUS_LINE_MAX_CHARS = 16
RUSSIAN_STATUS_MAX_CHARS = RUSSIAN_STATUS_LINE_MAX_CHARS * 2 + 1
RUSSIAN_STATUS_MAX_UTF8_BYTES = RUSSIAN_STATUS_LINE_MAX_CHARS * 2 * 2 + 1
OLED_COMPACT_GLYPH_ADVANCE = 4
RUSSIAN_STATUS_ALLOWED_ABBREVIATIONS = frozenset({"Темп. датчика"})
C_STRING_RE = re.compile(r'"([^"\\]*(?:\\.[^"\\]*)*)"')
C_STRING_CAPTURE = r'"((?:[^"\\]|\\.)*)"'
STOCK_MAP_RE = re.compile(
    r"static\s+const\s+uint8_t\s+StockMap\[\]\s*=([\s\S]*?);\s*static\s+const\s+int\s+StockmapLength"
)
CHAR_STOCK_MAP_RE = re.compile(
    r"static\s+const\s+uint8_t\s+CharStockMap\[\]\s*=([\s\S]*?);"
)
GLYPH_COMMENT_RE = re.compile(r'/\*\s*"([^"]+)",\s*(\d+)\s*\*/')
WORD_STOCK_RE = re.compile(
    r"static\s+const\s+uint8_t\s+(WordStock2?)\[\]\s*=\s*\{([\s\S]*?)\};"
)
CYRILLIC_TABLE_RE = re.compile(
    r"static\s+const\s+uint8_t\s+s_cyrillic_font\s*"
    r"\[\s*CYRILLIC_FONT_GLYPH_COUNT\s*\]\s*"
    r"\[\s*CYRILLIC_FONT_GLYPH_BYTES\s*\]\s*=\s*\{([\s\S]*?)\};"
)
STATE_TABLE_RE = re.compile(
    r"static\s+const\s+EquipStateDisplay\s+state_display_table\[\]\s*=\s*\{([\s\S]*?)\};"
)
DEVICE_STATE_ENUM_RE = re.compile(
    r"typedef\s+enum\s*\{\s*(STATE_STANDBY[\s\S]*?)\}\s*DeviceState\s*;"
)
STATUS_TEXT_CALL_RE = re.compile(
    rf"DisplayLanguage_SelectStatusText\(\s*lang\s*,\s*"
    rf"{C_STRING_CAPTURE}\s*,\s*{C_STRING_CAPTURE}\s*,\s*"
    rf"{C_STRING_CAPTURE}\s*\)"
)
STATUS_BADGE_ASSIGNMENT_RE = re.compile(
    rf"badge_text\s*=\s*DisplayLanguage_SelectStatusText\(\s*lang\s*,\s*"
    rf"{C_STRING_CAPTURE}\s*,\s*{C_STRING_CAPTURE}\s*,\s*"
    rf"{C_STRING_CAPTURE}\s*\)"
)
RUSSIAN_STATUS_SLOT_LABEL_RE = re.compile(
    r"static\s+const\s+char\s*\*\s*Display_GetRussianStatusSlotLabel\s*"
    r"\([^)]*\)\s*\{([\s\S]*?)^\}",
    re.MULTILINE,
)
RUSSIAN_AUXILIARY_STATUS_ENGLISH = ("Unknown", "ProtoErr")


@dataclass
class DisplayString:
    path: Path
    lineno: int
    text: str


@dataclass
class RussianStatus:
    state: str
    text: str


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def unescape_c_string(text: str) -> str:
    # 当前工程显示字符串主要是普通字面量；这里只处理常见转义，避免误伤中文。
    replacements = {
        r"\"": '"',
        r"\\": "\\",
        r"\r": "\r",
        r"\n": "\n",
        r"\t": "\t",
    }
    for old, new in replacements.items():
        text = text.replace(old, new)
    return text


def is_cjk(ch: str) -> bool:
    code = ord(ch)
    return (
        0x4E00 <= code <= 0x9FFF
        or 0x3400 <= code <= 0x4DBF
        or 0xF900 <= code <= 0xFAFF
    )


def is_font_char(ch: str) -> bool:
    return is_cjk(ch) or ch in SPECIAL_CHARS


def is_cyrillic_char(ch: str) -> bool:
    codepoint = ord(ch)
    return 0x0400 <= codepoint <= 0x052F


def parse_c_integer(value: str) -> int:
    normalized = re.sub(r"[uUlL]+$", "", value.strip())
    return int(normalized, 0)


def parse_unsigned_define(source: str, name: str) -> int:
    match = re.search(
        rf"^\s*#define\s+{re.escape(name)}\s+((?:0[xX][0-9A-Fa-f]+)|(?:\d+))[uUlL]*\s*$",
        source,
        re.MULTILINE,
    )
    if not match:
        raise RuntimeError(f"未找到无符号整数宏 {name}")
    return int(match.group(1), 0)


def parse_mapping_index(expression: str, codepoint: int) -> int:
    compact = re.sub(r"[\s()]", "", expression)
    integer = r"(?:0[xX][0-9A-Fa-f]+|\d+)[uUlL]*"

    match = re.fullmatch(rf"codepoint-({integer})", compact)
    if match:
        return codepoint - parse_c_integer(match.group(1))

    match = re.fullmatch(rf"({integer})\+codepoint-({integer})", compact)
    if match:
        return (
            parse_c_integer(match.group(1))
            + codepoint
            - parse_c_integer(match.group(2))
        )

    if re.fullmatch(integer, compact):
        return parse_c_integer(compact)

    raise RuntimeError(f"无法解析西里尔字模索引表达式: {expression}")


def parse_cyrillic_mapping(source: str) -> dict[int, int]:
    without_comments = strip_comments_keep_lines(source)
    compact = re.sub(r"\s+", " ", without_comments)
    integer = r"(?:0[xX][0-9A-Fa-f]+|\d+)[uUlL]*"
    range_re = re.compile(
        rf"(?:if|else if) \(\(codepoint >= ({integer})\) && "
        rf"\(codepoint <= ({integer})\)\) \{{ index = ([^;]+); \}}"
    )
    single_re = re.compile(
        rf"(?:if|else if) \(codepoint == ({integer})\) "
        r"\{ index = ([^;]+); \}"
    )
    mapping: dict[int, int] = {}

    def add_mapping(codepoint: int, index: int) -> None:
        if codepoint in mapping:
            raise RuntimeError(f"西里尔码点重复映射: U+{codepoint:04X}")
        mapping[codepoint] = index

    for match in range_re.finditer(compact):
        first = parse_c_integer(match.group(1))
        last = parse_c_integer(match.group(2))
        if first > last:
            raise RuntimeError(f"西里尔码点范围倒置: U+{first:04X}..U+{last:04X}")
        for codepoint in range(first, last + 1):
            add_mapping(codepoint, parse_mapping_index(match.group(3), codepoint))

    for match in single_re.finditer(compact):
        codepoint = parse_c_integer(match.group(1))
        add_mapping(codepoint, parse_mapping_index(match.group(2), codepoint))

    if "return s_cyrillic_font[index];" not in compact:
        raise RuntimeError("FontCyrillic_GetGlyph 未返回 s_cyrillic_font[index]")
    if not mapping:
        raise RuntimeError("未解析到 FontCyrillic_GetGlyph 码点映射")
    return mapping


def parse_cyrillic_font() -> tuple[list[list[int]], dict[int, int], int, int]:
    header = read_text(CYRILLIC_H)
    source = read_text(CYRILLIC_C)
    glyph_bytes = parse_unsigned_define(header, "CYRILLIC_FONT_GLYPH_BYTES")
    glyph_count = parse_unsigned_define(header, "CYRILLIC_FONT_GLYPH_COUNT")
    if not re.search(
        r"\bconst\s+uint8_t\s*\*\s*FontCyrillic_GetGlyph\s*"
        r"\(\s*uint32_t\s+[A-Za-z_]\w*\s*\)\s*;",
        header,
    ):
        raise RuntimeError("font_cyrillic.h 缺少 FontCyrillic_GetGlyph 声明")

    table_match = CYRILLIC_TABLE_RE.search(source)
    if not table_match:
        raise RuntimeError(f"未找到俄文字模表: {CYRILLIC_C}")

    table_body = strip_comments_keep_lines(table_match.group(1))
    rows = re.findall(r"\{([^{}]*)\}", table_body)
    glyphs = []
    for row_index, row in enumerate(rows):
        values = []
        for token in (part.strip() for part in row.split(",")):
            if not token:
                continue
            if not re.fullmatch(r"(?:0[xX][0-9A-Fa-f]+|\d+)[uUlL]*", token):
                raise RuntimeError(
                    f"俄文字模{row_index}包含无法解析的字节值: {token}"
                )
            value = parse_c_integer(token)
            if value > 0xFF:
                raise RuntimeError(
                    f"俄文字模{row_index}包含越界字节值: {token}"
                )
            values.append(value)
        glyphs.append(values)

    return glyphs, parse_cyrillic_mapping(source), glyph_count, glyph_bytes


def validate_cyrillic_font(
    glyphs: list[list[int]],
    mapping: dict[int, int],
    glyph_count: int,
    glyph_bytes: int,
) -> list[str]:
    errors = []
    expected_codepoints = {ord(ch) for ch in RUSSIAN_LETTERS}

    if len(RUSSIAN_UPPERCASE) != 33 or len(RUSSIAN_LOWERCASE) != 33:
        errors.append("检查器内置俄语字母表不是33个大写加33个小写")
    if glyph_count != 66:
        errors.append(f"CYRILLIC_FONT_GLYPH_COUNT 应为66，实际为{glyph_count}")
    if glyph_bytes != 14:
        errors.append(f"CYRILLIC_FONT_GLYPH_BYTES 应为14，实际为{glyph_bytes}")
    if len(glyphs) != glyph_count:
        errors.append(f"俄文字模行数异常: expected={glyph_count}, actual={len(glyphs)}")

    for index, glyph in enumerate(glyphs):
        if len(glyph) != glyph_bytes:
            errors.append(
                f"俄文字模{index}字节数异常: expected={glyph_bytes}, actual={len(glyph)}"
            )
            continue
        if not any(glyph):
            errors.append(f"俄文字模{index}为空白点阵")
        if all(value == 0xFF for value in glyph):
            errors.append(f"俄文字模{index}为全亮点阵")

    glyph_indices_by_bitmap = defaultdict(list)
    for index, glyph in enumerate(glyphs):
        if len(glyph) == glyph_bytes:
            glyph_indices_by_bitmap[tuple(glyph)].append(index)
    for indices in glyph_indices_by_bitmap.values():
        if len(indices) > 1:
            errors.append(
                "俄文字模包含完全重复点阵: "
                + ", ".join(str(index) for index in indices)
            )

    actual_codepoints = set(mapping)
    missing = expected_codepoints - actual_codepoints
    extra = actual_codepoints - expected_codepoints
    if missing:
        errors.append(
            "俄文字模缺少码点: "
            + " ".join(f"U+{value:04X}" for value in sorted(missing))
        )
    if extra:
        errors.append(
            "俄文字模包含非俄语码点: "
            + " ".join(f"U+{value:04X}" for value in sorted(extra))
        )

    indices = list(mapping.values())
    if any(index < 0 or index >= glyph_count for index in indices):
        errors.append("俄文字模映射包含越界索引")
    if len(indices) != len(set(indices)):
        errors.append("俄文字模映射包含重复索引")
    if set(indices) != set(range(glyph_count)):
        errors.append("俄文字模映射未完整覆盖0至65号字模")
    if ord("Ё") not in mapping or ord("ё") not in mapping:
        errors.append("俄文字模必须显式包含Ё和ё")

    errors.extend(validate_cyrillic_notice())
    return errors


def validate_cyrillic_notice() -> list[str]:
    errors = []
    if not THIRD_PARTY_NOTICES.is_file():
        return [f"缺少俄文字模第三方许可声明: {THIRD_PARTY_NOTICES}"]

    try:
        notice = read_text(THIRD_PARTY_NOTICES)
        source = read_text(CYRILLIC_C)
    except (OSError, UnicodeError) as exc:
        return [f"无法读取俄文字模第三方许可声明: {exc}"]

    for required in (
        "DejaVu Sans Mono",
        "https://github.com/dejavu-fonts/dejavu-fonts",
        "Bitstream Vera Fonts Copyright",
        "Permission is hereby granted",
        "Arev Fonts Copyright",
    ):
        if required not in notice:
            errors.append(f"俄文字模第三方许可声明缺少内容: {required}")

    if "../../THIRD_PARTY_NOTICES.md" not in source:
        errors.append("font_cyrillic.c 未指向 THIRD_PARTY_NOTICES.md")

    return errors


def parse_device_state_names(path: Path) -> set[str]:
    # 两端头文件的状态枚举标识均为ASCII；CPU2历史中文注释不是UTF-8，因此按字节保真读取。
    source = path.read_bytes().decode("latin-1")
    match = DEVICE_STATE_ENUM_RE.search(source)
    if not match:
        raise RuntimeError(f"未找到 DeviceState 枚举: {path}")

    body = strip_comments_keep_lines(match.group(1))
    states = set(re.findall(r"\b(STATE_[A-Za-z0-9_]+)\s*=", body))
    return {state for state in states if "_RESERVED_" not in state}


def parse_russian_statuses() -> tuple[list[RussianStatus], list[str]]:
    display_header = read_text(DISPLAY_H)
    display_source = read_text(DISPLAY_C)
    errors = []

    if not re.search(r"\bconst\s+char\s*\*\s*disp_ru\s*;", display_header):
        errors.append("EquipStateDisplay 缺少俄文字段 disp_ru")

    table_match = STATE_TABLE_RE.search(display_source)
    if not table_match:
        raise RuntimeError(f"未找到设备状态显示表: {DISPLAY_C}")

    table_body = strip_comments_keep_lines(table_match.group(1))
    statuses = []
    seen_states = set()
    for entry_match in re.finditer(r"\{([^{}]*)\}", table_body):
        entry = entry_match.group(1)
        state_match = re.search(r"\b(STATE_[A-Za-z0-9_]+)\b", entry)
        if not state_match:
            continue
        state = state_match.group(1)
        strings = [unescape_c_string(value) for value in C_STRING_RE.findall(entry)]
        if state in seen_states:
            errors.append(f"状态表存在重复状态: {state}")
        seen_states.add(state)
        if len(strings) != 3:
            errors.append(f"{state} 应包含中、英、俄3列字符串，实际为{len(strings)}列")
            continue
        statuses.append(RussianStatus(state=state, text=strings[2]))

    if not seen_states:
        raise RuntimeError("设备状态显示表没有可识别的 STATE_ 条目")
    if len(statuses) != len(seen_states):
        errors.append(
            f"俄文状态覆盖不完整: states={len(seen_states)}, russian={len(statuses)}"
        )

    cpu2_states = parse_device_state_names(CPU2_SYSTEM_PARAMETER_H)
    cpu3_states = parse_device_state_names(CPU3_SYSTEM_PARAMETER_H)
    if cpu2_states != cpu3_states:
        errors.append(
            "CPU2/CPU3 DeviceState 非保留状态不一致: "
            f"CPU2独有={sorted(cpu2_states - cpu3_states)}, "
            f"CPU3独有={sorted(cpu3_states - cpu2_states)}"
        )

    missing_states = cpu2_states - seen_states
    extra_states = seen_states - cpu2_states
    if missing_states:
        errors.append("状态表缺少俄文映射: " + ", ".join(sorted(missing_states)))
    if extra_states:
        errors.append("状态表包含未知状态: " + ", ".join(sorted(extra_states)))
    return statuses, errors


def parse_char_stock_map() -> str:
    source = read_text(DISPLAY_C)
    match = CHAR_STOCK_MAP_RE.search(source)
    if not match:
        raise RuntimeError(f"未找到 CharStockMap: {DISPLAY_C}")
    return "".join(
        unescape_c_string(fragment) for fragment in C_STRING_RE.findall(match.group(1))
    )


def parse_russian_status_support_texts() -> tuple[list[str], list[str], list[str]]:
    source = strip_comments_keep_lines(read_text(DISPLAY_C))
    errors = []
    badges = [
        unescape_c_string(match.group(3))
        for match in STATUS_BADGE_ASSIGNMENT_RE.finditer(source)
    ]
    if not badges:
        errors.append("未解析到设备状态页维护/模拟俄文徽标")

    calls_by_english = defaultdict(list)
    for match in STATUS_TEXT_CALL_RE.finditer(source):
        english = unescape_c_string(match.group(2))
        russian = unescape_c_string(match.group(3))
        calls_by_english[english].append(russian)

    auxiliary = []
    for english in RUSSIAN_AUXILIARY_STATUS_ENGLISH:
        texts = calls_by_english.get(english, [])
        if len(texts) != 1:
            errors.append(
                f"状态页辅助文案 {english} 应有1个俄文值，实际为{len(texts)}个"
            )
        else:
            auxiliary.append(texts[0])

    return badges, auxiliary, errors


def parse_russian_status_slot_labels() -> list[str]:
    source = strip_comments_keep_lines(read_text(DISPLAY_C))
    match = RUSSIAN_STATUS_SLOT_LABEL_RE.search(source)
    if not match:
        raise RuntimeError("未找到俄文状态页字段标签函数")
    return [
        unescape_c_string(value)
        for value in C_STRING_RE.findall(match.group(1))
        if any(is_cyrillic_char(ch) for ch in value)
    ]


def normalize_russian_status(text: str) -> str:
    return "".join(ch.upper() for ch in text if ch.isalnum())


def oled_compact_text_width(text: str) -> int:
    return len(text) * OLED_COMPACT_GLYPH_ADVANCE


def split_russian_status_lines(text: str) -> tuple[str, ...] | None:
    if len(text) <= RUSSIAN_STATUS_LINE_MAX_CHARS:
        return (text,)

    candidates = []
    for index, char in enumerate(text):
        if char != " ":
            continue
        first = text[:index].rstrip()
        second = text[index + 1 :].lstrip()
        if (
            first
            and second
            and len(first) <= RUSSIAN_STATUS_LINE_MAX_CHARS
            and len(second) <= RUSSIAN_STATUS_LINE_MAX_CHARS
        ):
            candidates.append((abs(len(first) - len(second)), index, first, second))

    if not candidates:
        return None
    _, _, first, second = min(candidates)
    return first, second


def validate_compact_russian_text(
    context: str, text: str, ascii_font_chars: frozenset[str]
) -> list[str]:
    errors = []
    invalid = sorted(
        {ch for ch in text if not ch.isascii() and ch not in RUSSIAN_LETTERS}
    )
    if invalid:
        errors.append(f"{context} 包含不支持字符: {''.join(invalid)}")

    unsupported_ascii = sorted(
        {ch for ch in text if ch.isascii() and ch not in ascii_font_chars}
    )
    if unsupported_ascii:
        errors.append(
            f"{context} 包含 CharStockMap 未覆盖的ASCII字符: "
            + "".join(unsupported_ascii)
        )
    return errors


def validate_russian_statuses(
    statuses: list[RussianStatus],
    badges: list[str],
    auxiliary: list[str],
    slot_labels: list[str],
    ascii_font_chars: frozenset[str],
    display_width: int,
    badge_clear_width: int,
) -> list[str]:
    errors = []
    states_by_text = defaultdict(list)
    states_by_normalized_text = defaultdict(list)

    for item in statuses:
        errors.extend(
            validate_compact_russian_text(item.state, item.text, ascii_font_chars)
        )
        if not any(ch in RUSSIAN_LETTERS for ch in item.text):
            errors.append(f"{item.state} 俄文为空或未包含俄语字母")
        if (
            "." in item.text
            and item.text not in RUSSIAN_STATUS_ALLOWED_ABBREVIATIONS
        ):
            errors.append(f"{item.state} 状态文案包含缩写点号: {item.text}")

        byte_count = len(item.text.encode("utf-8"))
        if byte_count > RUSSIAN_STATUS_MAX_UTF8_BYTES:
            errors.append(
                f"{item.state} 俄文UTF-8长度{byte_count}超过"
                f"{RUSSIAN_STATUS_MAX_UTF8_BYTES}字节: {item.text}"
            )
        if len(item.text) > RUSSIAN_STATUS_MAX_CHARS:
            errors.append(
                f"{item.state} 俄文字符数{len(item.text)}超过"
                f"{RUSSIAN_STATUS_MAX_CHARS}: {item.text}"
            )

        lines = split_russian_status_lines(item.text)
        if lines is None:
            errors.append(
                f"{item.state} 无法按空格拆成两行且每行不超过"
                f"{RUSSIAN_STATUS_LINE_MAX_CHARS}字符: {item.text}"
            )
        else:
            for line_index, line in enumerate(lines, start=1):
                line_width = oled_compact_text_width(line)
                if line_width > display_width:
                    errors.append(
                        f"{item.state} 第{line_index}行宽度{line_width}像素"
                        f"超过OLED行宽{display_width}: {line}"
                    )

        states_by_text[item.text].append(item.state)
        states_by_normalized_text[normalize_russian_status(item.text)].append(item)

    for text, states in states_by_text.items():
        if len(states) > 1:
            errors.append(f"俄文状态文案重复 {text}: {', '.join(states)}")

    for normalized, items in states_by_normalized_text.items():
        texts = {item.text for item in items}
        if normalized and len(items) > 1 and len(texts) > 1:
            states = ", ".join(item.state for item in items)
            errors.append(
                f"俄文状态仅靠空格或标点区分 {normalized}: {states}"
            )

    for index, text in enumerate(badges):
        context = f"状态徽标[{index}] {text}"
        errors.extend(validate_compact_russian_text(context, text, ascii_font_chars))

    for index, text in enumerate(slot_labels):
        context = f"状态字段标签[{index}] {text}"
        errors.extend(validate_compact_russian_text(context, text, ascii_font_chars))
        if "." in text and text not in RUSSIAN_STATUS_ALLOWED_ABBREVIATIONS:
            errors.append(f"{context} 包含缩写点号")
        if len(text) > RUSSIAN_STATUS_LINE_MAX_CHARS:
            errors.append(
                f"{context} 字符数{len(text)}超过"
                f"{RUSSIAN_STATUS_LINE_MAX_CHARS}"
            )
        width = oled_compact_text_width(text)
        if width > display_width:
            errors.append(
                f"{context} 宽度{width}像素超过OLED行宽{display_width}"
            )

    for text in auxiliary:
        context = f"状态辅助文案 {text}"
        errors.extend(validate_compact_russian_text(context, text, ascii_font_chars))
        lines = split_russian_status_lines(text)
        if lines is None:
            errors.append(
                f"{context} 无法按空格拆成两行且每行不超过"
                f"{RUSSIAN_STATUS_LINE_MAX_CHARS}字符"
            )
        elif any(oled_compact_text_width(line) > display_width for line in lines):
            errors.append(f"{context} 分行后仍超过OLED行宽{display_width}")

    return errors


def strip_comments_keep_lines(text: str) -> str:
    def block_replacer(match: re.Match) -> str:
        return "\n" * match.group(0).count("\n")

    text = re.sub(r"/\*[\s\S]*?\*/", block_replacer, text)
    text = re.sub(r"//.*", "", text)
    return text


def extract_braced_body(source: str, start: int, context: str) -> str:
    if (start < 0) or (start >= len(source)) or (source[start] != "{"):
        raise RuntimeError(f"未找到左花括号: {context}")

    depth = 0
    for index in range(start, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start + 1:index]

    raise RuntimeError(f"代码块缺少右花括号: {context}")


def extract_c_function_body(source: str, function_name: str) -> str:
    match = re.search(
        rf"\b{re.escape(function_name)}\s*\([^;{{}}]*\)\s*\{{",
        source,
    )
    if not match:
        raise RuntimeError(f"未找到函数定义: {function_name}")

    return extract_braced_body(source, match.end() - 1, function_name)


def validate_russian_display_contract() -> list[str]:
    errors = []
    app_main_source = strip_comments_keep_lines(read_text(APP_MAIN_C))
    display_source = strip_comments_keep_lines(read_text(DISPLAY_C))
    tankopera_source = strip_comments_keep_lines(read_text(DISPLAY_TANKOPERA_C))

    app_init_body = extract_c_function_body(app_main_source, "App_Init")
    startup_calls = (
        "DisplayInit();",
        "Cpu3_Params_LoadFromFRAM();",
        "DisplayAubonLogo();",
    )
    startup_call_positions = tuple(app_init_body.find(call) for call in startup_calls)
    if (-1 in startup_call_positions) or (
        startup_call_positions != tuple(sorted(startup_call_positions))
    ):
        errors.append(
            "启动显示顺序必须为OLED初始化、FRAM语言加载、按已保存语言绘制首屏"
        )

    startup_body = extract_c_function_body(display_source, "EquipFirstPower")
    for required in (
        "DisplayLanguage_SelectStatusText",
        '"Попытка связи..."',
        '"Версия:"',
    ):
        if required not in startup_body:
            errors.append(f"俄文启动页缺少约定内容: {required}")

    fault_detail_body = extract_c_function_body(
        display_source, "Display_ShowErrorReasonPage"
    )
    for required in (
        "!DisplayLanguage_IsRussian",
        "DisplayLanguage_SelectStatusText",
        '"Назад"',
    ):
        if required not in fault_detail_body:
            errors.append(f"俄文故障详情页缺少约定内容: {required}")

    torque_temperature_label = '"Темп. датчика"'
    torque_temperature_count = display_source.count(torque_temperature_label)
    if torque_temperature_count != 3:
        errors.append(
            "扭矩传感器温度标签数量异常: "
            f"expected=3, actual={torque_temperature_count}"
        )

    snapshot_body = extract_c_function_body(
        display_source, "Display_BuildStatusSnapshot"
    )
    for required in (
        "display_status_last_snapshot.valid",
        "snapshot->state != display_status_last_snapshot.state",
        "snapshot->ctx != display_status_last_snapshot.ctx",
        "display_status_page_index = 0",
        "display_status_page_hold_count = 0U",
    ):
        if required not in snapshot_body:
            errors.append(f"状态切换轮播复位缺少约定逻辑: {required}")

    equipment_body = extract_c_function_body(display_source, "DIS_Equipment")
    russian_branch = re.search(
        r"if\s*\(lang\s*==\s*LANGUAGE_RUSSIAN\)\s*\{",
        equipment_body,
    )
    if russian_branch is None:
        errors.append("未找到俄文状态标题分支")
    else:
        russian_branch_body = extract_braced_body(
            equipment_body,
            russian_branch.end() - 1,
            "DIS_Equipment 俄文分支",
        )
        for required in (
            "Display_DrawRussianStatusText",
            "status_badge_visible = false",
            "return;",
        ):
            if required not in russian_branch_body:
                errors.append(f"俄文状态标题分支缺少约定逻辑: {required}")

    save_body = extract_c_function_body(tankopera_source, "set_display_language")
    for required in (
        "if (Cpu3Local_WriteValueChecked",
        "language_save_retry = language",
        "language_save_failed = true",
        "setlanguage();",
    ):
        if required not in save_body:
            errors.append(f"语言保存失败处理缺少约定逻辑: {required}")

    parameter_save_body = extract_c_function_body(
        tankopera_source, "cmd_configpara_process"
    )
    for required in (
        "if (!Cpu3Local_WriteValueChecked",
        '"保存失败"',
        '"参数未更改"',
        "g_cpu3_uart_reinit_pending = 1",
    ):
        if required not in parameter_save_body:
            errors.append(f"参数配置本机写入反馈缺少约定逻辑: {required}")
    if "Cpu3Local_WriteValue(" in parameter_save_body:
        errors.append("参数配置仍使用无法返回持久化结果的本机写入接口")

    language_menu_body = extract_c_function_body(tankopera_source, "setlanguage")
    for required in (
        "if (language_save_failed)",
        "NowKeyPress == USE_KEY_SURE",
        "NowKeyPress == USE_KEY_BACK",
        "set_display_language(language_save_retry)",
        '"保存失败"',
        '"语言未更改"',
    ):
        if required not in language_menu_body:
            errors.append(f"语言保存失败页缺少约定内容: {required}")

    exit_body = extract_c_function_body(tankopera_source, "exitTankOpera")
    for required in (
        "language_save_failed = false",
        "language_save_retry = LANGUAGE_CHINESE",
    ):
        if required not in exit_body:
            errors.append(f"退出菜单未清理语言保存失败状态: {required}")

    return errors


def line_number(text: str, index: int) -> int:
    return text.count("\n", 0, index) + 1


def parse_stock_map() -> str:
    source = read_text(DISPLAY_C)
    match = STOCK_MAP_RE.search(source)
    if not match:
        raise RuntimeError(f"未找到 StockMap: {DISPLAY_C}")

    fragments = C_STRING_RE.findall(match.group(1))
    return "".join(unescape_c_string(part) for part in fragments)


def parse_glyph_tables() -> tuple[list[str], dict[str, int]]:
    source = read_text(DISPLAY_C)
    start = source.find("static const uint8_t WordStock[]")
    end = source.find("static const uint8_t NumberStock[]")
    if start < 0 or end < 0 or end <= start:
        raise RuntimeError(f"未找到 WordStock/WordStock2 字库区域: {DISPLAY_C}")

    section = source[start:end]
    glyph_sequence = []
    table_counts = {}
    expected_index = 0
    for table_match in WORD_STOCK_RE.finditer(section):
        name = table_match.group(1)
        table_body = table_match.group(2)
        glyph_matches = list(GLYPH_COMMENT_RE.finditer(table_body))
        glyphs = [match.group(1) for match in glyph_matches]
        byte_count = len(re.findall(r"0x[0-9A-Fa-f]{2}", table_body))
        if byte_count != len(glyphs) * 28:
            raise RuntimeError(
                f"{name} 点阵字节数异常: bytes={byte_count}, glyphs={len(glyphs)}"
            )
        for match in glyph_matches:
            actual_index = int(match.group(2))
            if actual_index != expected_index:
                raise RuntimeError(
                    f"{name} 点阵注释索引异常: expected={expected_index}, actual={actual_index}"
                )
            expected_index += 1
        table_counts[name] = len(glyphs)
        glyph_sequence.extend(glyphs)
    if set(table_counts) != {"WordStock", "WordStock2"}:
        raise RuntimeError(f"未完整解析 WordStock/WordStock2: {DISPLAY_C}")
    return glyph_sequence, table_counts


def validate_stock_map(
    stock_map: str, glyph_sequence: list[str], table_counts: dict[str, int]
) -> list[str]:
    errors = []

    if len(glyph_sequence) != len(stock_map):
        errors.append(
            f"索引与点阵数量不相等: StockMap={len(stock_map)}, WordStock/WordStock2={len(glyph_sequence)}"
        )

    duplicate_chars = [ch for ch, count in Counter(stock_map).items() if count > 1]
    for ch in duplicate_chars:
        indices = [index for index, value in enumerate(stock_map) if value == ch]
        errors.append(f"重复字符{ch}: 索引={indices}")

    for name, count in table_counts.items():
        if count > 255:
            errors.append(f"{name} 点阵数量超过8位索引上限: {count}")

    mismatch_count = 0
    for index, expected in enumerate(stock_map):
        actual = glyph_sequence[index] if index < len(glyph_sequence) else "<缺失>"
        if actual != expected:
            mismatch_count += 1
            if mismatch_count <= 10:
                errors.append(f"索引{index}: StockMap={expected}, 点阵={actual}")

    if mismatch_count > 10:
        errors.append(f"其余索引不一致数量: {mismatch_count - 10}")

    return errors


def collect_display_strings() -> list[DisplayString]:
    display_strings = []

    for path in CHECK_FILES:
        if not path.exists():
            continue

        raw = read_text(path)
        if path == DISPLAY_C:
            start = raw.find("static const uint8_t StockMap[]")
            end = raw.find("static const uint8_t NumberStock[]")
            if start < 0 or end < 0 or end <= start:
                raise RuntimeError(f"未找到需要排除的字库定义区域: {DISPLAY_C}")
            raw = raw[:start] + "\n" * raw[start:end].count("\n") + raw[end:]
        source = strip_comments_keep_lines(raw)
        raw_lines = raw.splitlines()

        for match in C_STRING_RE.finditer(source):
            text = unescape_c_string(match.group(1))
            if not any(is_font_char(ch) for ch in text):
                continue

            lineno = line_number(source, match.start())
            raw_line = raw_lines[lineno - 1] if lineno - 1 < len(raw_lines) else ""

            # system_parameter.c 里只有参数表名称会进入参数显示界面；
            # printf 调试输出不走 OLED 字库，避免纳入缺字统计。
            if path.name == "system_parameter.c":
                if "{(uint8_t*)" not in raw_line and "{(u8*)" not in raw_line:
                    continue

            display_strings.append(DisplayString(path, lineno, text))

    return display_strings


def collect_russian_strings() -> list[DisplayString]:
    display_dir = SCRIPT_DIR / "Application" / "display"
    paths = sorted(display_dir.glob("*.c")) + sorted(display_dir.glob("*.h"))
    russian_strings = []

    for path in paths:
        raw = read_text(path)
        source = strip_comments_keep_lines(raw)
        for match in C_STRING_RE.finditer(source):
            value = unescape_c_string(match.group(1))
            if not any(is_cyrillic_char(ch) for ch in value):
                continue
            russian_strings.append(
                DisplayString(
                    path=path,
                    lineno=line_number(source, match.start()),
                    text=value,
                )
            )

    return russian_strings


def validate_russian_strings(strings: list[DisplayString]) -> list[str]:
    errors = []

    for item in strings:
        invalid = sorted(
            {ch for ch in item.text if not ch.isascii() and ch not in RUSSIAN_LETTERS}
        )
        if invalid:
            errors.append(
                f"{item.path}:{item.lineno} 俄文字串包含不支持字符"
                f" {''.join(invalid)}: {item.text}"
            )

    return errors


def print_stock_validation(
    stock_map: str, glyph_sequence: list[str], table_counts: dict[str, int]
) -> bool:
    print("====== 字库一致性检查 ======")
    print(f"StockMap 字符数: {len(stock_map)}")
    print(f"WordStock/WordStock2 点阵注释数: {len(glyph_sequence)}")
    print(
        "点阵分段:",
        f"WordStock={table_counts['WordStock']}, WordStock2={table_counts['WordStock2']}",
    )

    errors = validate_stock_map(stock_map, glyph_sequence, table_counts)
    if not errors:
        print("结果: StockMap 与点阵顺序一致")
        return True

    print("结果: StockMap 与点阵顺序不一致")
    for error in errors:
        print("   -", error)
    return False


def print_limited_errors(errors: list[str], limit: int = 20) -> None:
    for error in errors[:limit]:
        print("   -", error)
    if len(errors) > limit:
        print(f"   - 其余错误数量: {len(errors) - limit}")


def print_cyrillic_validation(
    glyphs: list[list[int]],
    mapping: dict[int, int],
    glyph_count: int,
    glyph_bytes: int,
) -> bool:
    print("\n====== 俄文字模一致性检查 ======")
    print(f"声明字模数: {glyph_count}")
    print(f"实际字模数: {len(glyphs)}")
    print(f"每字模字节数: {glyph_bytes}")
    print(f"已映射俄语字母数: {len(mapping)}")
    print(
        "Ё/ё索引:",
        f"Ё={mapping.get(ord('Ё'), '<missing>')},",
        f"ё={mapping.get(ord('ё'), '<missing>')}",
    )

    errors = validate_cyrillic_font(glyphs, mapping, glyph_count, glyph_bytes)
    if not errors:
        print("结果: 66个俄语字母与14字节点阵映射完整")
        return True

    print("结果: 俄文字模或码点映射不一致")
    print_limited_errors(errors)
    return False


def print_russian_status_validation(
    statuses: list[RussianStatus],
    badges: list[str],
    auxiliary: list[str],
    slot_labels: list[str],
    ascii_font_chars: frozenset[str],
    display_width: int,
    badge_clear_width: int,
    parse_errors: list[str],
) -> bool:
    print("\n====== 俄文状态文案检查 ======")
    print(f"已解析俄文状态数: {len(statuses)}")
    print(f"唯一俄文状态数: {len({item.text for item in statuses})}")
    print(f"已解析状态徽标数: {len(badges)}")
    print(f"已解析俄文字段标签数: {len(slot_labels)}")
    wrapped_lines = [
        line
        for item in statuses
        for line in (split_russian_status_lines(item.text) or ())
    ]
    if statuses:
        max_chars = max(len(item.text) for item in statuses)
        max_bytes = max(len(item.text.encode("utf-8")) for item in statuses)
    else:
        max_chars = 0
        max_bytes = 0
    max_line_chars = max((len(line) for line in wrapped_lines), default=0)
    max_line_width = max(
        (oled_compact_text_width(line) for line in wrapped_lines), default=0
    )
    print(f"最大Unicode字符数: {max_chars}/{RUSSIAN_STATUS_MAX_CHARS}")
    print(f"最大UTF-8字节数: {max_bytes}/{RUSSIAN_STATUS_MAX_UTF8_BYTES}")
    print(
        f"分行后最大字符数: {max_line_chars}/"
        f"{RUSSIAN_STATUS_LINE_MAX_CHARS}"
    )
    print(f"分行后最大像素宽度: {max_line_width}/{display_width}")
    print("俄文状态模式不绘制维护/模拟徽标和电机图标")

    errors = parse_errors + validate_russian_statuses(
        statuses,
        badges,
        auxiliary,
        slot_labels,
        ascii_font_chars,
        display_width,
        badge_clear_width,
    )
    if not errors:
        print("结果: 状态俄文唯一，字符集与OLED静态双行布局符合要求")
        return True

    print("结果: 状态表俄文覆盖、唯一性、字符集或静态双行布局不符合要求")
    print_limited_errors(errors)
    return False


def print_russian_string_validation(strings: list[DisplayString]) -> bool:
    print("\n====== 俄文字串字符集检查 ======")
    print(f"检查俄文字串数量: {len(strings)}")
    errors = validate_russian_strings(strings)
    if not errors:
        print("结果: 俄文字串仅使用ASCII和完整俄语66字母集合")
        return True

    print("结果: 俄文字串包含字库未覆盖字符")
    print_limited_errors(errors)
    return False


def print_unused_report(stock_map: str, display_strings: list[DisplayString]) -> None:
    used_counter = Counter(
        ch
        for item in display_strings
        for ch in item.text
        if is_font_char(ch)
    )
    unused = [(index, ch) for index, ch in enumerate(stock_map) if ch not in used_counter]

    print("\n====== OLED 字库使用统计 ======")
    print(f"已使用字符数(去重): {len(set(stock_map) & set(used_counter))}")
    print(f"候选未使用字符数: {len(unused)}")
    if unused:
        print("候选未使用字符:", "".join(ch for _, ch in unused))
        print("候选未使用索引:", " ".join(f"{index}:{ch}" for index, ch in unused))
        print("说明: 该结果只基于当前纳入检查的 OLED 源码字符串，不作为自动删字依据。")


def print_missing_report(stock_map: str, display_strings: list[DisplayString]) -> bool:
    stock_set = set(stock_map)
    missing_counter = Counter()
    missing_examples = defaultdict(list)

    for item in display_strings:
        for ch in item.text:
            if is_font_char(ch) and ch not in stock_set:
                missing_counter[ch] += 1
                if len(missing_examples[ch]) < 5:
                    missing_examples[ch].append(
                        f"{item.path}:{item.lineno}  {item.text}"
                    )

    print("\n====== OLED 显示缺字统计 ======")
    print(f"检查字符串数量: {len(display_strings)}")

    if not missing_counter:
        print("结果: 未发现缺字")
    else:
        for ch, count in missing_counter.most_common():
            print(f"{ch}  x{count}")
            for example in missing_examples[ch]:
                print("   -", example)

    print("\n====== 汇总 ======")
    print("缺字总数(去重):", len(missing_counter))
    print("缺字总出现次数:", sum(missing_counter.values()))

    return not missing_counter


def main() -> int:
    try:
        stock_map = parse_stock_map()
        ascii_font_chars = frozenset(parse_char_stock_map())
        glyph_sequence, table_counts = parse_glyph_tables()
        display_strings = collect_display_strings()
        cyrillic_glyphs, cyrillic_mapping, cyrillic_count, cyrillic_bytes = (
            parse_cyrillic_font()
        )
        russian_statuses, russian_status_parse_errors = parse_russian_statuses()
        russian_badges, russian_auxiliary, russian_support_errors = (
            parse_russian_status_support_texts()
        )
        russian_slot_labels = parse_russian_status_slot_labels()
        russian_status_parse_errors.extend(russian_support_errors)
        russian_status_parse_errors.extend(validate_russian_display_contract())
        display_header = read_text(DISPLAY_H)
        display_width = parse_unsigned_define(display_header, "OLED_LINE8_END") + 1
        badge_clear_start = parse_unsigned_define(display_header, "OLED_LINE8_6")
        badge_clear_width = display_width - badge_clear_start
        russian_strings = collect_russian_strings()
    except Exception as exc:
        print(f"字库检查失败: {exc}", file=sys.stderr)
        return 2

    stock_ok = print_stock_validation(stock_map, glyph_sequence, table_counts)
    missing_ok = print_missing_report(stock_map, display_strings)
    cyrillic_ok = print_cyrillic_validation(
        cyrillic_glyphs,
        cyrillic_mapping,
        cyrillic_count,
        cyrillic_bytes,
    )
    russian_status_ok = print_russian_status_validation(
        russian_statuses,
        russian_badges,
        russian_auxiliary,
        russian_slot_labels,
        ascii_font_chars,
        display_width,
        badge_clear_width,
        russian_status_parse_errors,
    )
    russian_strings_ok = print_russian_string_validation(russian_strings)
    print_unused_report(stock_map, display_strings)
    return (
        0
        if stock_ok
        and missing_ok
        and cyrillic_ok
        and russian_status_ok
        and russian_strings_ok
        else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())
