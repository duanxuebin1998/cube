import os
import re
from collections import Counter, defaultdict

# 1) 把你的 StockMap 原样粘贴到这里（保持和工程一致）
STOCK_MAP = (
"通讯尝试中液位跟随密度温℃版本水测量完成寻找标定零点校正获取称重步进未知无线提浮子至置阈值"
"向下运行上故障设备初始化罐底综合国满载空仪表配模式检修读实时区间每米自带宽拟静止电压效流速"
"力质体积磁致伸缩新老雷达计一机变送器算方轮询加转包抓主协议软件对外集超从待回检修单到监分布"
"部参数编码圈恢复出厂文份指令操作在←是否入返确认？退与显示换混相关据源气案改非法请输清写铁"
"失败直接隐藏字不范围异常油品类型保顶盘起高支撑原润滑苯胺乙烯丙酮酸甲酯丁腈冰二甲氯烷醇芳烃"
"环己氧基叔醚醛溶精制卤聚多元糠呋喃邻辛硫三偏硝盐氢钠仲石拱卧"
"球形蒸汽尺距离开手启闭短地址扫描路只能连台功打印容稍等几钟储管道径周期调低于盲报警语言发错"
"误础界面程序减比股长介信号后限例权屏幕维护视终继状态最探头浸小第悬停禁用弦工固产反先动当前"
"大英更传层滞域使磨结束针总阻六级内息命感顺有阈值角导本整瓦锡兰厚首波特率验位奇偶预留默"
)

# 2) 需要扫描的源码目录（改成你的工程路径）
ROOT = r"D:\CUBE\LTD_DISPLAY_CPU3"  # 例如 r"D:\CUBE\LTD_MAIN_CPU3"

# 3) 只扫描这些后缀
SUFFIX = (".c", ".h")

# 4) 提取 C 字符串的正则（支持 \" 转义）
cstr_re = re.compile(r'"([^"\\]*(?:\\.[^"\\]*)*)"')

def unescape_c(s: str) -> str:
    # 只处理常见转义，足够用于中文统计
    s = s.replace(r'\"', '"').replace(r"\\", "\\")
    s = s.replace(r"\r", "\r").replace(r"\n", "\n").replace(r"\t", "\t")
    return s

def is_cjk(ch: str) -> bool:
    o = ord(ch)
    return (
        0x4E00 <= o <= 0x9FFF or   # CJK Unified Ideographs
        0x3400 <= o <= 0x4DBF or   # CJK Extension A
        0xF900 <= o <= 0xFAFF      # CJK Compatibility Ideographs
    )

def collect_used_strings(root: str):
    files = []
    for dp, _, fn in os.walk(root):
        for f in fn:
            if f.endswith(SUFFIX):
                files.append(os.path.join(dp, f))

    used = []
    where = []  # (file, lineno, text)
    for path in files:
        with open(path, "r", encoding="utf-8", errors="ignore") as fp:
            for lineno, line in enumerate(fp, 1):
                for m in cstr_re.finditer(line):
                    raw = m.group(1)
                    text = unescape_c(raw)
                    # 只保留包含中文/特殊符号的字符串，减少噪声
                    if any(is_cjk(ch) for ch in text) or any(ch in text for ch in "℃←？"):
                        used.append(text)
                        where.append((path, lineno, text))
    return where

def main():
    stock_set = set(STOCK_MAP)
    where = collect_used_strings(ROOT)

    missing_counter = Counter()
    missing_examples = defaultdict(list)

    for path, lineno, text in where:
        for ch in text:
            if is_cjk(ch) or ch in "℃←？":
                if ch not in stock_set:
                    missing_counter[ch] += 1
                    if len(missing_examples[ch]) < 5:  # 每个缺字最多记录 5 条出处
                        missing_examples[ch].append(f"{path}:{lineno}  {text}")

    # 输出
    print("====== 缺字统计（按出现次数降序）======")
    for ch, cnt in missing_counter.most_common():
        print(f"{ch}  x{cnt}")
        for ex in missing_examples[ch]:
            print("   -", ex)

    print("\n====== 汇总 ======")
    print("缺字总数(去重)：", len(missing_counter))
    print("缺字总出现次数：", sum(missing_counter.values()))

if __name__ == "__main__":
    main()
