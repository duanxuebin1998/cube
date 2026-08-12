import re
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
DISPLAY_C = SCRIPT_DIR / "Application" / "display" / "display.c"

# 只检查会进入 OLED 汉字显示链路的源码文件。
CHECK_FILES = (
    SCRIPT_DIR / "Application" / "display" / "display.c",
    SCRIPT_DIR / "Application" / "display" / "display_tankopera.c",
    SCRIPT_DIR / "Application" / "system_param" / "system_parameter.c",
)

SPECIAL_CHARS = "℃←？"
C_STRING_RE = re.compile(r'"([^"\\]*(?:\\.[^"\\]*)*)"')
STOCK_MAP_RE = re.compile(
    r"static\s+const\s+uint8_t\s+StockMap\[\]\s*=([\s\S]*?);\s*static\s+const\s+int\s+StockmapLength"
)
GLYPH_COMMENT_RE = re.compile(r'/\*\s*"([^"]+)",\s*(\d+)\s*\*/')
WORD_STOCK_RE = re.compile(
    r"static\s+const\s+uint8_t\s+(WordStock2?)\[\]\s*=\s*\{([\s\S]*?)\};"
)


@dataclass
class DisplayString:
    path: Path
    lineno: int
    text: str


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="ignore")


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


def strip_comments_keep_lines(text: str) -> str:
    def block_replacer(match: re.Match) -> str:
        return "\n" * match.group(0).count("\n")

    text = re.sub(r"/\*[\s\S]*?\*/", block_replacer, text)
    text = re.sub(r"//.*", "", text)
    return text


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
        glyph_sequence, table_counts = parse_glyph_tables()
        display_strings = collect_display_strings()
    except Exception as exc:
        print(f"字库检查失败: {exc}", file=sys.stderr)
        return 2

    stock_ok = print_stock_validation(stock_map, glyph_sequence, table_counts)
    missing_ok = print_missing_report(stock_map, display_strings)
    print_unused_report(stock_map, display_strings)
    return 0 if stock_ok and missing_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
