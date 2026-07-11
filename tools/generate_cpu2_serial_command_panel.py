#!/usr/bin/env python3
"""生成并校验 CPU2 串口助手命令面板配置。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "docs"
    / "01_协议与寄存器"
    / "CPU2通信与解耦"
    / "串口助手命令面板"
    / "CPU2串口助手命令面板配置.json"
)


def command(comment: str, payload: str) -> dict[str, object]:
    """创建一个文本发送按钮。"""
    return {
        "comment": comment,
        "payload": payload,
        "sendMode": "text",
        "delayMs": 100,
    }


def group(name: str, commands: Iterable[tuple[str, str]]) -> dict[str, object]:
    """创建一个不自动重复发送的命令组。"""
    return {
        "type": "group",
        "name": name,
        "repeatCount": 0,
        "commands": [command(comment, payload) for comment, payload in commands],
    }


def build_panel() -> dict[str, object]:
    """返回当前受控版本的完整命令面板。"""
    items = [
        group(
            "00 系统查询与安全控制",
            [
                ("查看命令帮助", "HELP"),
                ("读取 CPU2/协议/参数版本", "VER?"),
                ("读取设备状态与当前命令", "STAT?"),
                ("读取当前错误码与原因", "ERR?"),
                ("[安全] 通用停止并取消当前测量/调试", "STOP"),
            ],
        ),
        group(
            "01 正式测量与标定",
            [
                ("回零点（正式命令）", "I"),
                ("测罐底（正式命令）", "G"),
                ("液位测量（正式命令）", "K"),
                ("分布测量（正式命令）", "R"),
                ("水位测量（正式命令）", "W"),
                ("标定空载扭力", "O"),
                ("标定满载扭力", "P"),
                ("编码器零点清零（不清电机记步基准）", "L"),
            ],
        ),
        group(
            "02 通信与无线配对",
            [
                ("传感器与无线通信综合测试", "SC"),
                ("读取当前无线连接状态", "SPC"),
                ("扫描 CH9141K 从机列表", "SPS"),
                ("按 RSSI 连接信号最好的设备", "SPR"),
                ("[模板-先编辑] 按蓝牙名称精确匹配", "SPN=DEVICE_NAME"),
            ],
        ),
        group(
            "03 手动运动（先小距离）",
            [
                ("停止手动运动", "A0"),
                ("上行 10 mm", "A+10"),
                ("下行 10 mm", "A-10"),
                ("上行 100 mm", "A+100"),
                ("下行 100 mm", "A-100"),
                ("上行 500 mm", "A+500"),
                ("下行 500 mm", "A-500"),
                ("上行 1000 mm", "A+1000"),
                ("下行 1000 mm", "A-1000"),
            ],
        ),
        group(
            "04 点动与绝对定位测试",
            [
                ("点动上行 100 mm，1.5 m/min", "BJ+100,1.5"),
                ("点动下行 100 mm，1.5 m/min", "BJ-100,1.5"),
                ("绝对定位 1420 mm，1.5 m/min", "BJP1420,1.5"),
                ("绝对定位 1450 mm，1.5 m/min", "BJP1450,1.5"),
            ],
        ),
        group(
            "05 往返、重复性与耐久测试（循环命令）",
            [
                ("电机模型往返 200 mm", "B200"),
                ("电机模型往返 200 mm + 传感器通信", "B200S"),
                ("编码器往返 200 mm（默认速度/加速度）", "BE200"),
                ("编码器往返 200 mm + 传感器通信", "BE200S"),
                ("编码器往返 200 mm，2 m/min，加速度 5 倍", "BE200,2,5"),
                ("编码器往返 200 mm，2 m/min，加速度 5 倍 + 通信", "BE200,2,5,S"),
                ("[循环] 4 步进分辨率测试", "C"),
                ("[循环] 4 步进下行触底测试", "D"),
                ("[循环] 4 步进上行碰零点测试", "E"),
                ("[循环] 罐底测量重复性测试", "F"),
                ("[循环] 零点/罐底重复性测试", "H"),
                ("[循环] 液位测量重复性测试", "J"),
                ("[循环] 电机高温耐久测试", "M"),
                ("[循环] 电机 300 mm 往返测试", "N"),
                ("[安全] 通用停止上述循环/运动", "STOP"),
            ],
        ),
        group(
            "06 位置源切换与电机诊断",
            [
                ("读取位置源及尺带位置对比", "YS"),
                ("读取电机记步详细诊断", "YC"),
                ("TMC5130 静态 SPI 通信测试", "YT"),
                ("切换到电机记步（自动标定一周）", "YM"),
                ("切换回编码轮记步", "YE"),
            ],
        ),
        group(
            "07 卷筒拟合（按流程操作）",
            [
                ("开始全局自动采样（原零点为基准）", "T1"),
                ("开始局部自动采样（当前位置为零圈）", "T2"),
                ("停止自动采样", "T0"),
                ("手动添加当前样本", "TA"),
                ("读取拟合状态与样本", "TS"),
                ("执行全局拟合求解", "TR"),
                ("执行局部拟合求解", "TV"),
                ("应用拟合尺带厚度", "TP"),
                ("应用首圈周长和尺带厚度", "TU"),
            ],
        ),
        group(
            "08 AO 电流输出测试（单位 0.01 mA）",
            [
                ("AO 输出 4.00 mA", "AO400"),
                ("AO 输出 12.00 mA", "AO1200"),
                ("AO 输出 20.00 mA", "AO2000"),
                ("AO 输出 22.00 mA", "AO2200"),
                ("AO 扫描 4/12/20/22 mA", "AOS"),
            ],
        ),
        group(
            "09 高风险与界面演示",
            [
                ("[高风险-会覆盖参数] 恢复出厂参数", "Q"),
                ("单点测量界面模拟展示（非真实测量）", "X"),
            ],
        ),
    ]
    return {"groups": [], "items": items, "rootCommands": []}


def validate_panel(panel: dict[str, object]) -> tuple[int, int]:
    """校验导入结构和本项目的安全约束，返回组数与按钮数。"""
    if list(panel) != ["groups", "items", "rootCommands"]:
        raise ValueError("顶层字段或字段顺序不符合约定")
    if panel["groups"] != [] or panel["rootCommands"] != []:
        raise ValueError("groups 和 rootCommands 必须为空数组")

    items = panel["items"]
    if not isinstance(items, list) or not items:
        raise ValueError("items 必须是非空数组")
    if items[0].get("name") != "00 系统查询与安全控制":
        raise ValueError("第一组必须是系统查询与安全控制")

    required_payloads = {"STOP", "HELP", "VER?", "STAT?", "ERR?"}
    all_payloads: set[str] = set()
    command_count = 0
    for item in items:
        if set(item) != {"type", "name", "repeatCount", "commands"}:
            raise ValueError(f"命令组字段不完整：{item.get('name', '<unknown>')}")
        if item["type"] != "group" or item["repeatCount"] != 0:
            raise ValueError(f"命令组属性非法：{item['name']}")
        commands = item["commands"]
        if not isinstance(commands, list) or not commands:
            raise ValueError(f"命令组不能为空：{item['name']}")

        group_payloads: set[str] = set()
        for entry in commands:
            if set(entry) != {"comment", "payload", "sendMode", "delayMs"}:
                raise ValueError(f"命令字段不完整：{item['name']}")
            payload = entry["payload"]
            if not isinstance(entry["comment"], str) or not entry["comment"]:
                raise ValueError(f"按钮说明不能为空：{item['name']}")
            if not isinstance(payload, str) or not payload:
                raise ValueError(f"payload 不能为空：{item['name']}")
            if entry["sendMode"] != "text" or entry["delayMs"] != 100:
                raise ValueError(f"发送属性非法：{item['name']} / {payload}")
            if payload in group_payloads:
                raise ValueError(f"组内 payload 重复：{item['name']} / {payload}")
            group_payloads.add(payload)
            all_payloads.add(payload)
            command_count += 1

    missing = required_payloads - all_payloads
    if missing:
        raise ValueError(f"缺少系统命令：{', '.join(sorted(missing))}")
    if len(items) != 10 or command_count != 67:
        raise ValueError(f"命令面板规模异常：{len(items)} 组 / {command_count} 个按钮")
    return len(items), command_count


def render_panel(panel: dict[str, object]) -> str:
    """按稳定格式序列化，统一使用 UTF-8、CRLF 和末尾换行。"""
    text = json.dumps(panel, ensure_ascii=False, indent=2)
    return text.replace("\n", "\r\n") + "\r\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="输出 JSON 路径（默认生成仓库内正式配置）",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="只检查目标文件是否与生成结果完全一致，不写文件",
    )
    return parser.parse_args()


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    args = parse_args()
    panel = build_panel()
    group_count, command_count = validate_panel(panel)
    expected = render_panel(panel).encode("utf-8")
    output = args.output.resolve()

    if args.check:
        if not output.is_file():
            print(f"FAIL: 配置文件不存在：{output}")
            return 1
        if output.read_bytes() != expected:
            print(f"FAIL: 配置文件不是最新生成结果：{output}")
            return 1
        print(f"PASS: {output}（{group_count} 组，{command_count} 个按钮）")
        return 0

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(expected)
    print(f"已生成：{output}（{group_count} 组，{command_count} 个按钮）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
