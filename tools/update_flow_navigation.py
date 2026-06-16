# -*- coding: utf-8 -*-
"""Update cross-document navigation for CPU2/CPU3 flow HTML docs.

The script is intentionally data-driven and idempotent.  It creates the global
program-flow entrance, creates the cross-CPU business route page, and injects a
small relationship navigator into every CPU2/CPU3 flow page.
"""

from __future__ import annotations

import html
import os
import re
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[1]
DOC_NAV_DIR = ROOT / "docs" / "00_程序流程导航"
GLOBAL_INDEX = DOC_NAV_DIR / "index.html"
CROSS_ROUTE = DOC_NAV_DIR / "跨CPU业务链路.html"
CPU2_DIR = ROOT / "LTD_MAIN_CPU2" / "docs" / "00_程序流程"
CPU3_DIR = ROOT / "LTD_DISPLAY_CPU3" / "docs" / "00_程序流程"

STYLE_MARK = '<style id="cross-flow-nav-style">'
NAV_START = "<!-- CROSS-FLOW-NAV-START -->"
NAV_END = "<!-- CROSS-FLOW-NAV-END -->"


PAGE_DEFS: Dict[str, Dict[str, str]] = {
    "global": {"title": "程序流程统一入口", "path": str(GLOBAL_INDEX), "cpu": "全局"},
    "cross": {"title": "跨 CPU 业务链路", "path": str(CROSS_ROUTE), "cpu": "全局"},
    "cpu2_total": {
        "title": "CPU2 程序流程总览",
        "path": str(CPU2_DIR / "CPU2程序流程总览.html"),
        "cpu": "CPU2",
    },
    "cpu2_issue": {
        "title": "CPU2 问题点总表",
        "path": str(CPU2_DIR / "问题点总表.html"),
        "cpu": "CPU2",
    },
    "cpu2_01": {
        "title": "启动主循环与中断",
        "path": str(CPU2_DIR / "01_启动主循环与中断.html"),
        "cpu": "CPU2",
    },
    "cpu2_02": {
        "title": "测量命令总入口",
        "path": str(CPU2_DIR / "02_测量命令总入口.html"),
        "cpu": "CPU2",
    },
    "cpu2_03": {
        "title": "回零与罐底罐高",
        "path": str(CPU2_DIR / "03_回零与罐底罐高.html"),
        "cpu": "CPU2",
    },
    "cpu2_04": {
        "title": "液位测量与跟随",
        "path": str(CPU2_DIR / "04_液位测量与跟随.html"),
        "cpu": "CPU2",
    },
    "cpu2_05": {
        "title": "水位测量与跟随",
        "path": str(CPU2_DIR / "05_水位测量与跟随.html"),
        "cpu": "CPU2",
    },
    "cpu2_06": {
        "title": "密度与单点测量",
        "path": str(CPU2_DIR / "06_密度与单点测量.html"),
        "cpu": "CPU2",
    },
    "cpu2_07": {
        "title": "故障管理与恢复",
        "path": str(CPU2_DIR / "07_故障管理与恢复.html"),
        "cpu": "CPU2",
    },
    "cpu2_08": {
        "title": "电机与位置模型",
        "path": str(CPU2_DIR / "08_电机与位置模型.html"),
        "cpu": "CPU2",
    },
    "cpu2_09": {
        "title": "传感器与无线通信",
        "path": str(CPU2_DIR / "09_传感器与无线通信.html"),
        "cpu": "CPU2",
    },
    "cpu2_10": {
        "title": "Modbus 与 CPU3 通信",
        "path": str(CPU2_DIR / "10_Modbus与CPU3通信.html"),
        "cpu": "CPU2",
    },
    "cpu2_11": {
        "title": "HART 接口",
        "path": str(CPU2_DIR / "11_HART接口.html"),
        "cpu": "CPU2",
    },
    "cpu2_12": {
        "title": "参数存储与系统配置",
        "path": str(CPU2_DIR / "12_参数存储与系统配置.html"),
        "cpu": "CPU2",
    },
    "cpu2_13": {
        "title": "称重与继电器输出",
        "path": str(CPU2_DIR / "13_称重与继电器输出.html"),
        "cpu": "CPU2",
    },
    "cpu2_14": {
        "title": "BSP 外设驱动",
        "path": str(CPU2_DIR / "14_BSP外设驱动.html"),
        "cpu": "CPU2",
    },
    "cpu2_15": {
        "title": "公共工具与算法",
        "path": str(CPU2_DIR / "15_公共工具与算法.html"),
        "cpu": "CPU2",
    },
    "cpu2_16": {
        "title": "其它项目代码",
        "path": str(CPU2_DIR / "16_其它项目代码.html"),
        "cpu": "CPU2",
    },
    "cpu3_total": {
        "title": "CPU3 程序流程总览",
        "path": str(CPU3_DIR / "CPU3程序流程总览.html"),
        "cpu": "CPU3",
    },
    "cpu3_01": {
        "title": "启动主循环与调度",
        "path": str(CPU3_DIR / "01_启动主循环与调度.html"),
        "cpu": "CPU3",
    },
    "cpu3_02": {
        "title": "CPU2 内部通信与轮询",
        "path": str(CPU3_DIR / "02_CPU2内部通信与轮询.html"),
        "cpu": "CPU3",
    },
    "cpu3_03": {
        "title": "外部 COM 协议分发与 RS485 发送",
        "path": str(CPU3_DIR / "03_外部COM协议分发与RS485发送.html"),
        "cpu": "CPU3",
    },
    "cpu3_04": {
        "title": "DSM 协议与命令映射",
        "path": str(CPU3_DIR / "04_DSM协议与命令映射.html"),
        "cpu": "CPU3",
    },
    "cpu3_05": {
        "title": "Wartsila 与 SI7000 协议适配",
        "path": str(CPU3_DIR / "05_Wartsila与SI7000协议适配.html"),
        "cpu": "CPU3",
    },
    "cpu3_06": {
        "title": "显示刷新与按键事件",
        "path": str(CPU3_DIR / "06_显示刷新与按键事件.html"),
        "cpu": "CPU3",
    },
    "cpu3_07": {
        "title": "菜单参数与指令下发",
        "path": str(CPU3_DIR / "07_菜单参数与指令下发.html"),
        "cpu": "CPU3",
    },
    "cpu3_08": {
        "title": "本机参数、FRAM、时钟与外设恢复",
        "path": str(CPU3_DIR / "08_本机参数FRAM时钟与外设恢复.html"),
        "cpu": "CPU3",
    },
}


CPU2_ORDER = [
    "cpu2_01",
    "cpu2_02",
    "cpu2_03",
    "cpu2_04",
    "cpu2_05",
    "cpu2_06",
    "cpu2_07",
    "cpu2_08",
    "cpu2_09",
    "cpu2_10",
    "cpu2_11",
    "cpu2_12",
    "cpu2_13",
    "cpu2_14",
    "cpu2_15",
    "cpu2_16",
]

CPU3_ORDER = [
    "cpu3_01",
    "cpu3_02",
    "cpu3_03",
    "cpu3_04",
    "cpu3_05",
    "cpu3_06",
    "cpu3_07",
    "cpu3_08",
]


ROUTES = [
    {
        "id": "oil",
        "title": "液位测量与跟随链路",
        "summary": "从 CPU3 菜单或外部协议发起找液位/液位跟随，CPU2 执行找液位、闭环跟随和状态上报，CPU3 再显示或对外响应。",
        "steps": [
            ("CPU3 指令来源", "cpu3_07", "菜单发找液位、标定或修正指令"),
            ("CPU3 内部通道", "cpu3_02", "通过 UART5/Modbus 写 CPU2 命令或参数"),
            ("CPU2 命令入口", "cpu2_02", "统一进入测量公共准备和命令分发"),
            ("CPU2 测量核心", "cpu2_04", "完成粗找、精找、阈值确定和跟随闭环"),
            ("状态回读", "cpu2_10", "CPU2 更新输入寄存器和状态"),
            ("CPU3 显示/协议", "cpu3_06", "状态页刷新，外部 DSM/Wartsila/SI7000 可读取结果"),
        ],
    },
    {
        "id": "water",
        "title": "水位测量与跟随链路",
        "summary": "水位链路与液位类似，但传感模式、阈值和盲区策略不同，建议从命令入口一路跟到 CPU2 水位详细流程。",
        "steps": [
            ("CPU3 指令来源", "cpu3_07", "菜单或外部协议选择找水/水位跟随"),
            ("CPU3 内部通道", "cpu3_02", "写 CPU2 命令，运行期轮询输入寄存器"),
            ("CPU2 命令入口", "cpu2_02", "统一公共准备后分发到水位流程"),
            ("CPU2 测量核心", "cpu2_05", "找水、跟随、盲区和错误出口"),
            ("状态回读", "cpu2_10", "水位状态、错误码和结果进入通信寄存器"),
            ("CPU3 显示/协议", "cpu3_06", "显示端刷新水位状态或等待下一轮"),
        ],
    },
    {
        "id": "density",
        "title": "密度、单点和分布测量链路",
        "summary": "密度链路同时涉及 CPU2 测量算法、分布点回读和 CPU3 的 Wartsila/SI7000 协议适配。",
        "steps": [
            ("外部/菜单入口", "cpu3_05", "Wartsila/SI7000 或菜单触发密度类指令"),
            ("CPU3 内部通道", "cpu3_02", "写命令并在完成后分批读取密度分布点"),
            ("CPU2 命令入口", "cpu2_02", "按密度、单点、区间、瓦锡兰场景分发"),
            ("CPU2 测量核心", "cpu2_06", "完成单点、分布、区间密度测量和状态发布"),
            ("状态/点表回读", "cpu2_10", "结果、profile 完成标志和点表供 CPU3 读取"),
            ("外部协议响应", "cpu3_05", "Wartsila/SI7000 将缓存结果映射给外部系统"),
        ],
    },
    {
        "id": "readparams",
        "title": "读取部件参数、RSSI 与 AO 运行态链路",
        "summary": "读取部件参数由 CPU3 菜单下发，CPU2 周期刷新传感器快照并查询 CH9141K 当前连接 RSSI；协议 11 继续在 RSSI 后追加 AO 运行态，CPU3 轮询尾部输入寄存器后在状态页显示。",
        "steps": [
            ("CPU3 菜单入口", "cpu3_07", "读取部件参数位于测量/维护入口，下发 CMD_READ_PART_PARAMS"),
            ("CPU3 内部轮询", "cpu3_02", "运行轮询读取输入寄存器尾部 WirelessPairingStatus 和 AoOutputRuntime 字段"),
            ("CPU2 命令入口", "cpu2_02", "进入读取部件参数命令并保持 STATE_READPARAMETEROVER 持续刷新"),
            ("CPU2 传感器快照", "cpu2_09", "每 1s 刷新位置、称重、温度、频率、电容、角度，每 5s 查询蓝牙 RSSI"),
            ("CPU2 寄存器发布", "cpu2_10", "协议版本 11 在继电器运行态后追加 RSSI，再追加 AO 目标/实际/错误运行态"),
            ("CPU3 状态显示", "cpu3_06", "读取参数完成页显示 RSSI 或 N/A，状态页显示协议兼容和 AO/AD5421 故障原因"),
        ],
    },
    {
        "id": "ao",
        "title": "AO 模拟电流输出与 HART 链路",
        "summary": "AO 输出由 CPU3 菜单参数使能，CPU2 根据液位/故障/调试状态刷新 AoOutput 运行态，AD5421 输出电流，HART 和 CPU3 状态页读取同一运行数据。",
        "steps": [
            ("CPU3 AO 菜单", "cpu3_07", "AO使能为 0/1 设备参数，确认后通过内部 Modbus 写 CPU2"),
            ("CPU2 参数生效", "cpu2_12", "AoOutputEnable 默认关闭，写入后作为 AoOutput_Update 的硬开关"),
            ("液位结果来源", "cpu2_04", "找液位/跟随更新液位结果时同步刷新 AO 输出目标"),
            ("AO 服务与硬件", "cpu2_13", "AoOutput_Update 计算目标、限幅、节流写 AD5421，并记录运行态"),
            ("HART/AD5421 接口", "cpu2_11", "HART 命令 2/3 返回 AO 电流和百分比，AD5421 错误进入故障码"),
            ("CPU3 状态回读", "cpu3_06", "协议 11 输入尾段显示 AO 运行态和 AD5421 故障原因"),
        ],
    },
    {
        "id": "param",
        "title": "参数修改、保存和同步链路",
        "summary": "CPU3 菜单和外部协议都可能修改参数，必须区分 CPU3 本机参数、CPU2 设备参数和共享寄存器缓存。",
        "steps": [
            ("显示/协议入口", "cpu3_07", "用户输入参数或外部协议写保持寄存器"),
            ("本机参数支撑", "cpu3_08", "CPU3 通信/显示参数保存到 FRAM 并触发串口重配"),
            ("内部写入", "cpu3_02", "CPU2 设备参数通过 0x10 写入 CPU2"),
            ("CPU2 通信入口", "cpu2_10", "CPU2 接收保持寄存器写入并更新参数结构"),
            ("CPU2 参数存储", "cpu2_12", "延后保存、默认值、范围和系统配置"),
            ("业务生效", "cpu2_04", "测量流程在下一次执行时读取新参数"),
        ],
    },
    {
        "id": "fault",
        "title": "故障、恢复和状态展示链路",
        "summary": "CPU2 是主要故障产生和恢复位置，CPU3 负责轮询状态、显示提示，并在外部协议里反馈故障结果。",
        "steps": [
            ("故障产生", "cpu2_08", "电机、位置、称重碰撞或传感异常触发错误条件"),
            ("CPU2 故障管理", "cpu2_07", "SET_ERROR、状态切换、恢复尝试和错误输出"),
            ("CPU2 通信发布", "cpu2_10", "设备状态、错误码和测量状态进入寄存器"),
            ("CPU3 轮询缓存", "cpu3_02", "输入寄存器刷新 g_measurement"),
            ("CPU3 显示", "cpu3_06", "状态页和菜单页展示设备状态"),
            ("外部协议", "cpu3_04", "DSM/Wartsila/SI7000 读取缓存状态"),
        ],
    },
    {
        "id": "external",
        "title": "外部协议到 CPU2 测量链路",
        "summary": "外部 COM 先在 CPU3 分发，协议层只做映射和缓存，真正测量动作仍由 CPU2 命令入口完成。",
        "steps": [
            ("外部 COM", "cpu3_03", "COM1/COM2/COM3 按端口参数选择协议处理"),
            ("DSM 映射", "cpu3_04", "DSM 读写线圈/保持寄存器映射到缓存或 CPU2 指令"),
            ("Wartsila/SI7000", "cpu3_05", "协议适配层映射参数、结果和密度点"),
            ("CPU3 内部通道", "cpu3_02", "需要 CPU2 动作时通过内部 Modbus 下发"),
            ("CPU2 命令入口", "cpu2_02", "命令进入 CPU2 正式测量状态机"),
            ("CPU2 结果发布", "cpu2_10", "运行结果回到 CPU3 缓存并对外响应"),
        ],
    },
]


RELATIONS: Dict[str, Dict[str, object]] = {
    "cpu2_total": {
        "focus": "CPU2 所有流程页面的本地总览，适合从 CPU2 角度查启动、测量、通信、故障、外设和问题点。",
        "upstream": ["global", "cross"],
        "downstream": ["cpu2_01", "cpu2_02", "cpu2_04", "cpu2_06", "cpu2_10", "cpu2_issue"],
        "route": ["global", "cross", "cpu3_total"],
    },
    "cpu2_issue": {
        "focus": "集中查看 CPU2 各流程页整理出来的问题点、风险等级和整改建议。",
        "upstream": ["cpu2_total", "cpu2_04", "cpu2_05", "cpu2_06", "cpu2_07"],
        "downstream": ["cpu2_02", "cpu2_08", "cpu2_10", "cpu2_12"],
        "route": ["global", "cross", "cpu2_total"],
    },
    "cpu2_01": {
        "focus": "CPU2 上电后初始化外设、进入主循环，并为命令处理、故障恢复和通信刷新提供调度基础。",
        "upstream": ["global", "cpu2_total"],
        "downstream": ["cpu2_02", "cpu2_07", "cpu2_10", "cpu2_14"],
        "route": ["global", "cpu2_total", "cpu2_01", "cpu2_02"],
    },
    "cpu2_02": {
        "focus": "CPU2 正式测量命令总入口，负责公共准备、命令分发和各测量过程的状态切换。",
        "upstream": ["cpu3_07", "cpu3_04", "cpu3_05", "cpu3_02"],
        "downstream": ["cpu2_03", "cpu2_04", "cpu2_05", "cpu2_06", "cpu2_13", "cpu2_11"],
        "route": ["cross", "cpu3_07", "cpu3_02", "cpu2_02", "cpu2_04"],
    },
    "cpu2_03": {
        "focus": "回零、罐底和罐高建立机械位置基准，是液位、水位、密度测量前后共同依赖的位置基础。",
        "upstream": ["cpu2_02", "cpu2_08"],
        "downstream": ["cpu2_04", "cpu2_05", "cpu2_06", "cpu2_10"],
        "route": ["cpu2_total", "cpu2_02", "cpu2_03", "cpu2_04"],
    },
    "cpu2_04": {
        "focus": "液位找液、阈值确定、精找、跟随和方式二/连续方式闭环，是液位业务的 CPU2 核心页。",
        "upstream": ["cpu3_07", "cpu3_04", "cpu3_05", "cpu3_02", "cpu2_02", "cpu2_03"],
        "downstream": ["cpu2_07", "cpu2_10", "cpu3_02", "cpu3_06", "cpu3_04", "cpu2_issue"],
        "route": ["cross", "cpu3_07", "cpu3_02", "cpu2_02", "cpu2_04", "cpu2_10", "cpu3_06"],
    },
    "cpu2_05": {
        "focus": "水位找水、跟随、盲区和状态输出，和液位流程同属测量主链路但判断阈值不同。",
        "upstream": ["cpu3_07", "cpu3_02", "cpu2_02", "cpu2_03"],
        "downstream": ["cpu2_07", "cpu2_10", "cpu3_02", "cpu3_06", "cpu2_issue"],
        "route": ["cross", "cpu3_07", "cpu3_02", "cpu2_02", "cpu2_05", "cpu2_10"],
    },
    "cpu2_06": {
        "focus": "单点密度、区间密度、分布密度和瓦锡兰场景测量，决定 profile 结果和点表输出。",
        "upstream": ["cpu3_07", "cpu3_05", "cpu3_02", "cpu2_02"],
        "downstream": ["cpu2_10", "cpu3_02", "cpu3_05", "cpu2_issue"],
        "route": ["cross", "cpu3_05", "cpu3_02", "cpu2_02", "cpu2_06", "cpu3_05"],
    },
    "cpu2_07": {
        "focus": "CPU2 统一故障状态、错误码、恢复尝试和异常出口，是测量、电机、传感和通信的安全汇聚点。",
        "upstream": ["cpu2_04", "cpu2_05", "cpu2_06", "cpu2_08", "cpu2_09", "cpu2_13"],
        "downstream": ["cpu2_10", "cpu3_02", "cpu3_06", "cpu2_issue"],
        "route": ["cross", "cpu2_08", "cpu2_07", "cpu2_10", "cpu3_06"],
    },
    "cpu2_08": {
        "focus": "电机方向、速度、位置源、记步和丢步/碰撞判断，为所有测量闭环提供运动基础。",
        "upstream": ["cpu2_01", "cpu2_02", "cpu2_03", "cpu2_04", "cpu2_05", "cpu2_06"],
        "downstream": ["cpu2_07", "cpu2_10", "cpu2_14", "cpu2_issue"],
        "route": ["cpu2_total", "cpu2_02", "cpu2_08", "cpu2_07"],
    },
    "cpu2_09": {
        "focus": "传感器采集、无线通信、读取部件参数和蓝牙 RSSI 快照，为测量判断、状态输出和故障管理提供输入。",
        "upstream": ["cpu2_01", "cpu2_14"],
        "downstream": ["cpu2_04", "cpu2_05", "cpu2_06", "cpu2_07", "cpu2_10"],
        "route": ["cpu2_total", "cpu2_09", "cpu2_10", "cpu3_02", "cpu3_06"],
    },
    "cpu2_10": {
        "focus": "CPU2 与 CPU3 的共享寄存器和 Modbus 接口，是 CPU2 测量结果、无线 RSSI、AO 运行态和协议版本 11 尾部字段返回 CPU3 的主通道。",
        "upstream": ["cpu2_02", "cpu2_04", "cpu2_05", "cpu2_06", "cpu2_07", "cpu3_02"],
        "downstream": ["cpu3_02", "cpu3_04", "cpu3_05", "cpu3_06", "cpu2_12"],
        "route": ["cross", "cpu2_04", "cpu2_10", "cpu3_02", "cpu3_06"],
    },
    "cpu2_11": {
        "focus": "HART 对外接口和 AO 电流/百分比状态输出，适合和 CPU2 液位结果、AoOutput 服务、AD5421 驱动和状态发布一起阅读。",
        "upstream": ["cpu2_02", "cpu2_10", "cpu2_12", "cpu2_13"],
        "downstream": ["cpu2_04", "cpu2_07", "cpu2_13", "cpu3_06"],
        "route": ["cross", "cpu3_07", "cpu2_13", "cpu2_11", "cpu3_06"],
    },
    "cpu2_12": {
        "focus": "CPU2 参数默认值、范围、保存和系统配置，是测量算法、通信协议和现场配置的共同数据源。",
        "upstream": ["cpu3_07", "cpu3_02", "cpu2_10"],
        "downstream": ["cpu2_04", "cpu2_05", "cpu2_06", "cpu2_09", "cpu2_13"],
        "route": ["cross", "cpu3_07", "cpu3_02", "cpu2_10", "cpu2_12", "cpu2_04"],
    },
    "cpu2_13": {
        "focus": "称重、继电器和 AO 电流输出把测量状态转换为硬件输出，是现场报警、模拟量输出和 AD5421 运行态发布的关键链路。",
        "upstream": ["cpu2_01", "cpu2_04", "cpu2_05", "cpu2_06", "cpu2_12"],
        "downstream": ["cpu2_07", "cpu2_10", "cpu2_11"],
        "route": ["cross", "cpu3_07", "cpu2_12", "cpu2_13", "cpu2_10", "cpu3_06"],
    },
    "cpu2_14": {
        "focus": "BSP 外设驱动为电机、串口、传感器、FRAM、继电器和看门狗提供底层硬件能力。",
        "upstream": ["cpu2_01"],
        "downstream": ["cpu2_08", "cpu2_09", "cpu2_10", "cpu2_12", "cpu2_13"],
        "route": ["cpu2_total", "cpu2_14", "cpu2_08"],
    },
    "cpu2_15": {
        "focus": "公共算法和工具函数支撑多个业务页，适合在看具体问题时回查共用计算逻辑。",
        "upstream": ["cpu2_total"],
        "downstream": ["cpu2_04", "cpu2_05", "cpu2_06", "cpu2_10", "cpu2_12"],
        "route": ["cpu2_total", "cpu2_15", "cpu2_04"],
    },
    "cpu2_16": {
        "focus": "测试、调试和其它项目代码入口，主要用于现场排查和模拟流程，不作为正常业务主链路起点。",
        "upstream": ["cpu2_total", "cpu2_14"],
        "downstream": ["cpu2_07", "cpu2_08", "cpu2_10", "cpu2_issue"],
        "route": ["cpu2_total", "cpu2_16", "cpu2_issue"],
    },
    "cpu3_total": {
        "focus": "CPU3 所有流程页面的本地总览，适合从显示端、外部协议和 CPU2 轮询角度查找入口。",
        "upstream": ["global", "cross"],
        "downstream": ["cpu3_01", "cpu3_02", "cpu3_03", "cpu3_06", "cpu3_07"],
        "route": ["global", "cross", "cpu2_total"],
    },
    "cpu3_01": {
        "focus": "CPU3 上电初始化、主循环调度、显示任务、外部 COM 和 CPU2 轮询的优先级关系。",
        "upstream": ["global", "cpu3_total"],
        "downstream": ["cpu3_02", "cpu3_03", "cpu3_06", "cpu3_07", "cpu3_08"],
        "route": ["cpu3_total", "cpu3_01", "cpu3_02"],
    },
    "cpu3_02": {
        "focus": "CPU3 作为 CPU2 Modbus 主站，负责写指令/参数、读输入/保持寄存器、密度点分批回读，并解析协议版本 11 的 RSSI 与 AO 运行态尾部字段。",
        "upstream": ["cpu3_01", "cpu3_03", "cpu3_04", "cpu3_05", "cpu3_07"],
        "downstream": ["cpu2_10", "cpu2_02", "cpu2_04", "cpu2_05", "cpu2_06", "cpu3_06"],
        "route": ["cross", "cpu3_07", "cpu3_02", "cpu2_02", "cpu2_04", "cpu2_10"],
    },
    "cpu3_03": {
        "focus": "三路外部 COM 的协议选择、帧分发、RS485 发送和异常恢复，是外部系统进入 CPU3 的入口。",
        "upstream": ["cpu3_01", "cpu3_08"],
        "downstream": ["cpu3_04", "cpu3_05", "cpu3_02"],
        "route": ["cross", "cpu3_03", "cpu3_04", "cpu3_02", "cpu2_02"],
    },
    "cpu3_04": {
        "focus": "DSM 协议读写映射，把外部读写线圈/寄存器转换为 CPU3 缓存访问或 CPU2 指令下发。",
        "upstream": ["cpu3_03"],
        "downstream": ["cpu3_02", "cpu2_02", "cpu2_04", "cpu2_05", "cpu2_12"],
        "route": ["cross", "cpu3_03", "cpu3_04", "cpu3_02", "cpu2_02"],
    },
    "cpu3_05": {
        "focus": "Wartsila 和 SI7000 协议适配，将 CPU2 测量结果、参数和密度点映射给外部系统。",
        "upstream": ["cpu3_03", "cpu3_02", "cpu2_10"],
        "downstream": ["cpu3_02", "cpu2_06", "cpu2_04", "cpu2_05"],
        "route": ["cross", "cpu3_05", "cpu3_02", "cpu2_06", "cpu3_05"],
    },
    "cpu3_06": {
        "focus": "显示刷新、按键事件、状态页展示和调试等待页，负责把 CPU2 状态、读取部件参数 RSSI、AO/AD5421 运行态和 CPU3 本机交互变成现场可见界面。",
        "upstream": ["cpu3_01", "cpu3_02", "cpu3_07", "cpu3_08", "cpu2_10"],
        "downstream": ["cpu3_07", "cpu2_02", "cpu2_12"],
        "route": ["cross", "cpu2_10", "cpu3_02", "cpu3_06", "cpu3_07"],
    },
    "cpu3_07": {
        "focus": "菜单参数、指令确认、保护确认、测量/调试菜单分组、AO 输出使能和 CPU2 指令/参数下发，是人工操作进入测量链路的主入口。",
        "upstream": ["cpu3_06", "cpu3_08"],
        "downstream": ["cpu3_02", "cpu2_02", "cpu2_04", "cpu2_05", "cpu2_06", "cpu2_12"],
        "route": ["cross", "cpu3_06", "cpu3_07", "cpu3_02", "cpu2_02"],
    },
    "cpu3_08": {
        "focus": "CPU3 本机通信/显示参数、FRAM、RTC、OLED 和 UART 恢复，为显示端稳定运行提供支撑。",
        "upstream": ["cpu3_01"],
        "downstream": ["cpu3_03", "cpu3_06", "cpu3_07"],
        "route": ["cpu3_total", "cpu3_08", "cpu3_03"],
    },
}


INJECT_CSS = """
.cross-flow-nav{margin:16px 0 20px;padding:16px 18px;border:1px solid #cfe0f2;border-radius:10px;background:#fff;box-shadow:0 8px 22px rgba(31,59,91,.07)}
.cross-flow-nav *{box-sizing:border-box}
.cross-flow-nav__head{display:flex;justify-content:space-between;gap:12px;align-items:flex-start;margin-bottom:12px}
.cross-flow-nav__kicker{display:block;color:#4f6b86;font-size:12px;font-weight:700;margin-bottom:3px}
.cross-flow-nav__title{font-size:18px;font-weight:850;color:#17233a}
.cross-flow-nav__quick{display:flex;flex-wrap:wrap;gap:7px;justify-content:flex-end}
.cross-flow-nav__quick a,.cross-flow-nav__links a{display:inline-flex;align-items:center;min-height:28px;padding:4px 9px;border:1px solid #d7e5f3;border-radius:999px;background:#f6faff;color:#245d96;text-decoration:none;font-size:12px;font-weight:700}
.cross-flow-nav__grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:10px}
.cross-flow-nav__card{border:1px solid #dce7f2;border-radius:8px;background:#fbfdff;padding:12px}
.cross-flow-nav__card b{display:block;margin-bottom:5px;color:#16263b}
.cross-flow-nav__card p{margin:0 0 8px;color:#43556b;line-height:1.62;font-size:13px}
.cross-flow-nav__links{display:flex;flex-wrap:wrap;gap:6px}
@media(max-width:720px){.cross-flow-nav__head{display:block}.cross-flow-nav__quick{justify-content:flex-start;margin-top:10px}.cross-flow-nav{padding:14px}}
""".strip()


GLOBAL_CSS = """
:root{--ink:#17233a;--muted:#5c6f85;--line:#d5e1ed;--panel:#fff;--bg:#f2f6fb;--blue:#2f78c8;--green:#248b70;--amber:#b97816;--red:#b84a55;--violet:#6658bd;--shadow:0 12px 32px rgba(31,59,91,.10)}
*{box-sizing:border-box}
html{overflow-x:hidden}
body{margin:0;background:linear-gradient(180deg,#f6f9fd 0,#edf3f9 100%);color:var(--ink);font:15px/1.72 "Microsoft YaHei",Segoe UI,Arial,sans-serif;overflow-x:hidden}
a{color:#1f65aa;text-decoration:none}
code{font-family:Consolas,"Courier New",monospace;background:#eef4fb;border:1px solid #d7e4f2;border-radius:5px;padding:1px 5px;overflow-wrap:anywhere;word-break:break-all}
.wrap{max-width:1240px;margin:0 auto;padding:28px 28px 64px}
.hero{background:linear-gradient(135deg,#163a62,#0a7775);color:#fff;border-radius:12px;padding:30px 34px;box-shadow:var(--shadow)}
.hero h1{margin:0 0 10px;font-size:31px;line-height:1.25}
.hero p{margin:0;max-width:980px;color:#e6f4ff}
.meta-grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:12px;margin-top:22px}
.meta{border:1px solid rgba(255,255,255,.24);background:rgba(255,255,255,.09);border-radius:8px;padding:12px 14px}
.meta span{display:block;color:#cbe7ff;font-size:12px;font-weight:700}
.meta strong{display:block;margin-top:4px;color:#fff}
.topnav{position:sticky;top:0;z-index:5;margin:18px 0;padding:10px;background:rgba(255,255,255,.94);border:1px solid #d8e4f0;border-radius:10px;box-shadow:0 6px 18px rgba(38,65,96,.08);display:flex;flex-wrap:wrap;gap:8px}
.topnav a{padding:7px 10px;border-radius:6px;background:#f4f8fd;border:1px solid #dce8f6;color:#23527d;font-size:13px;font-weight:700}
section{background:var(--panel);border:1px solid #d7e2ef;border-radius:10px;margin:18px 0;padding:22px 24px;box-shadow:0 8px 24px rgba(35,58,92,.06)}
h2{margin:0 0 10px;font-size:24px}
h3{margin:16px 0 8px;font-size:18px}
.lead{color:var(--muted);margin:0 0 14px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:12px}
.card{border:1px solid #dbe6f2;border-radius:8px;background:#fbfdff;padding:13px}
.card strong{display:block;margin-bottom:5px;color:#15263c}
.card p{margin:0 0 8px;color:#46596f}
.links{display:flex;flex-wrap:wrap;gap:7px}
.pill{display:inline-flex;align-items:center;min-height:28px;padding:4px 9px;border-radius:999px;background:#f4f8fd;border:1px solid #d8e5f4;color:#245d96;font-size:12px;font-weight:700}
.split{display:grid;grid-template-columns:1fr 1fr;gap:14px}
.flow-wrap{width:100%;max-width:100%;min-width:0;overflow-x:auto;overflow-y:hidden;border:1px solid #d7e2ef;border-radius:10px;background:#fbfdff;margin-top:12px;-webkit-overflow-scrolling:touch}
.route-svg{display:block;min-width:0!important;max-width:100%!important;width:100%!important;height:auto!important}
.node{stroke-width:2}
.node.cpu3{fill:#eaf4ff;stroke:#5d9be2}
.node.cpu2{fill:#eaf8f1;stroke:#39a97e}
.node.ext{fill:#fff6df;stroke:#d39a31}
.node.state{fill:#f3f1ff;stroke:#7464c9}
.node.error{fill:#fff0f1;stroke:#c55c66}
.edge{fill:none;stroke:#61758e;stroke-width:2}
.edge-red{fill:none;stroke:#b84a55;stroke-width:2}
.nt{text-anchor:middle;font-size:16px;font-weight:800;fill:#17233a}
.ns{text-anchor:middle;font-size:12px;fill:#5c6f85}
.route-card{border-left:4px solid #2f78c8}
.route-card:nth-child(2n){border-left-color:#248b70}
.route-card:nth-child(3n){border-left-color:#b97816}
.steps{display:grid;gap:8px;margin-top:10px}
.step{display:grid;grid-template-columns:minmax(0,116px) minmax(0,1fr);gap:8px;align-items:start;padding:9px;border:1px solid #e0e8f2;border-radius:8px;background:#fff;min-width:0}
.step b{font-size:13px;color:#28476a}
.step span,.step b{overflow-wrap:anywhere;word-break:break-word}
.step span{display:block;color:#4d6076;font-size:13px}
.note{border:1px solid #bad8fb;background:#eef7ff;border-radius:8px;padding:11px 13px}
@media(max-width:800px){.wrap{padding:16px;max-width:100vw}.hero h1{font-size:24px}.meta-grid,.split{grid-template-columns:1fr}section{padding:18px;max-width:100%}.step{grid-template-columns:1fr}.topnav{position:static}.topnav a{white-space:normal;overflow-wrap:anywhere}.flow-wrap{overflow-x:auto}.route-svg{width:1000px!important;min-width:1000px!important;max-width:none!important}.nt{font-size:14px}.ns{font-size:11px}}
""".strip()


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as fp:
        fp.write(text)


def page_path(key: str) -> Path:
    return Path(PAGE_DEFS[key]["path"])


def rel_href(from_file: Path, to_key: str, anchor: str = "") -> str:
    target = page_path(to_key)
    href = Path(os.path.relpath(target, from_file.parent)).as_posix()
    return href + anchor


def link(from_file: Path, key: str, label: str | None = None, anchor: str = "") -> str:
    title = label if label is not None else PAGE_DEFS[key]["title"]
    return f'<a href="{html.escape(rel_href(from_file, key, anchor))}">{html.escape(title)}</a>'


def link_pill(from_file: Path, key: str, label: str | None = None, anchor: str = "") -> str:
    title = label if label is not None else PAGE_DEFS[key]["title"]
    return f'<a class="pill" href="{html.escape(rel_href(from_file, key, anchor))}">{html.escape(title)}</a>'


def pills(from_file: Path, keys: Sequence[str]) -> str:
    return "".join(link_pill(from_file, key) for key in keys)


def route_href(from_file: Path, route_id: str) -> str:
    return rel_href(from_file, "cross", f"#{route_id}")


def ensure_style(text: str) -> str:
    text = re.sub(
        r"\s*<style id=\"cross-flow-nav-style\">.*?</style>",
        "",
        text,
        flags=re.S,
    )
    style = f"\n<style id=\"cross-flow-nav-style\">\n{INJECT_CSS}\n</style>"
    if "</head>" not in text:
        raise RuntimeError("HTML page has no </head> marker")
    return text.replace("</head>", style + "\n</head>", 1)


def remove_nav_block(text: str) -> str:
    pattern = re.escape(NAV_START) + r".*?" + re.escape(NAV_END)
    return re.sub(r"\s*" + pattern + r"\s*", "\n", text, flags=re.S)


def insert_after_first(text: str, marker: str, block: str) -> str:
    idx = text.find(marker)
    if idx < 0:
        return ""
    idx += len(marker)
    return text[:idx] + block + text[idx:]


def make_relation_block(key: str) -> str:
    from_file = page_path(key)
    relation = RELATIONS.get(key, {})
    focus = str(relation.get("focus", "本页属于程序流程文档体系，可通过统一入口、跨 CPU 链路和本 CPU 总览继续定位上下游。"))
    upstream = list(relation.get("upstream", ["global"]))
    downstream = list(relation.get("downstream", []))
    route = list(relation.get("route", ["global", "cross"]))
    current_title = PAGE_DEFS[key]["title"]
    quick_links = [
        link(from_file, "global", "统一入口"),
        link(from_file, "cross", "跨 CPU 链路"),
    ]
    if PAGE_DEFS[key]["cpu"] == "CPU2":
        quick_links.extend(
            [
                link(from_file, "cpu2_total", "CPU2 总览"),
                link(from_file, "cpu3_total", "CPU3 总览"),
            ]
        )
    elif PAGE_DEFS[key]["cpu"] == "CPU3":
        quick_links.extend(
            [
                link(from_file, "cpu3_total", "CPU3 总览"),
                link(from_file, "cpu2_total", "CPU2 总览"),
            ]
        )
    else:
        quick_links.extend(
            [
                link(from_file, "cpu2_total", "CPU2 总览"),
                link(from_file, "cpu3_total", "CPU3 总览"),
            ]
        )

    cards = [
        (
            "上游入口",
            "先看这些页面，可以知道本页由哪个菜单、协议、主循环或测量状态触发。",
            upstream,
        ),
        (
            "本页定位",
            focus,
            [key],
        ),
        (
            "下游输出",
            "本页处理完成后，通常会影响这些测量、通信、显示、故障或参数页面。",
            downstream,
        ),
        (
            "建议阅读路线",
            "按业务链路阅读时，建议按这些页面顺序前后跳转。",
            route,
        ),
    ]

    card_html = []
    for title, desc, keys in cards:
        links = "".join(link(from_file, item) for item in keys if item in PAGE_DEFS)
        if not links:
            links = link(from_file, "cross", "查看跨 CPU 链路")
        card_html.append(
            f'<article class="cross-flow-nav__card"><b>{html.escape(title)}</b>'
            f"<p>{html.escape(desc)}</p><div class=\"cross-flow-nav__links\">{links}</div></article>"
        )

    return (
        f"\n{NAV_START}\n"
        '<section class="cross-flow-nav" id="cross-flow-nav" aria-label="程序流程关联导航">'
        '<div class="cross-flow-nav__head"><div>'
        '<span class="cross-flow-nav__kicker">程序流程关联导航</span>'
        f'<div class="cross-flow-nav__title">{html.escape(current_title)} 在整机链路中的位置</div>'
        '</div><div class="cross-flow-nav__quick">'
        + "".join(quick_links)
        + "</div></div>"
        '<div class="cross-flow-nav__grid">'
        + "".join(card_html)
        + "</div></section>\n"
        f"{NAV_END}\n"
    )


def inject_page(key: str) -> None:
    path = page_path(key)
    if not path.exists():
        return
    text = read_text(path)
    text = ensure_style(remove_nav_block(text))
    block = make_relation_block(key)
    updated = insert_after_first(text, "</nav>", block)
    if not updated:
        updated = insert_after_first(text, "</header>", block)
    if not updated:
        raise RuntimeError(f"Cannot find insertion point in {path}")
    write_text(path, updated)


def route_steps(from_file: Path, route: Mapping[str, object]) -> str:
    items = []
    for index, (stage, key, desc) in enumerate(route["steps"], start=1):  # type: ignore[index]
        items.append(
            '<div class="step">'
            f"<b>{index}. {html.escape(stage)}</b>"
            f"<span>{link(from_file, key)}：{html.escape(desc)}</span>"
            "</div>"
        )
    return "".join(items)


def make_global_index() -> str:
    from_file = GLOBAL_INDEX
    cpu2_links = "".join(link_pill(from_file, key) for key in CPU2_ORDER)
    cpu3_links = "".join(link_pill(from_file, key) for key in CPU3_ORDER)
    route_cards = []
    for route in ROUTES:
        route_cards.append(
            '<article class="card route-card">'
            f'<strong>{html.escape(route["title"])}</strong>'
            f'<p>{html.escape(route["summary"])}</p>'
            f'<div class="links"><a class="pill" href="{html.escape(rel_href(from_file, "cross", "#" + str(route["id"])))}">查看链路</a>'
            + "".join(link_pill(from_file, step[1]) for step in route["steps"][0:2])  # type: ignore[index]
            + "</div></article>"
        )
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>CUBE 程序流程统一入口</title>
<style>
{GLOBAL_CSS}
</style>
</head>
<body>
<div class="wrap">
<header class="hero">
<h1>CUBE 程序流程统一入口</h1>
<p>把 CPU2 测量执行程序、CPU3 显示/协议程序和跨 CPU 业务链路放到同一个阅读入口。单页仍按源码单独梳理，本页只负责告诉你从哪里进入、往哪里继续看。</p>
<div class="meta-grid">
<div class="meta"><span>覆盖对象</span><strong>CPU2 + CPU3</strong></div>
<div class="meta"><span>阅读方式</span><strong>先业务链路，后单页细节</strong></div>
<div class="meta"><span>页面关系</span><strong>上游 / 本页 / 下游</strong></div>
<div class="meta"><span>维护脚本</span><strong>tools/update_flow_navigation.py</strong></div>
</div>
</header>
<nav class="topnav">
<a href="#path">阅读路径</a>
<a href="#route">跨 CPU 业务链路</a>
<a href="#cpu2">CPU2 页面</a>
<a href="#cpu3">CPU3 页面</a>
<a href="#maintain">维护规则</a>
</nav>
<section id="path">
<h2>1. 推荐阅读路径</h2>
<p class="lead">不要从某个 HTML 孤立跳转。先按整机业务链路确定“谁触发、谁执行、谁上报”，再进入 CPU2/CPU3 的详细流程图。</p>
<div class="grid">
<article class="card"><strong>第一步：看整机链路</strong><p>用于判断某个功能跨了 CPU3 菜单、外部协议、CPU2 命令入口和 CPU2 测量核心中的哪些页面。</p><div class="links">{link_pill(from_file, "cross", "跨 CPU 业务链路")}</div></article>
<article class="card"><strong>第二步：看 CPU3 入口</strong><p>如果入口来自按键菜单、DSM、Wartsila、SI7000 或外部 COM，先看 CPU3 侧如何映射到 CPU2。</p><div class="links">{link_pill(from_file, "cpu3_total", "CPU3 总览")}{link_pill(from_file, "cpu3_07", "菜单入口")}{link_pill(from_file, "cpu3_03", "外部 COM")}</div></article>
<article class="card"><strong>第三步：看 CPU2 执行</strong><p>确认 CPU2 命令入口、测量细节、故障出口、通信寄存器和问题点。</p><div class="links">{link_pill(from_file, "cpu2_total", "CPU2 总览")}{link_pill(from_file, "cpu2_02", "命令入口")}{link_pill(from_file, "cpu2_issue", "问题总表")}</div></article>
</div>
</section>
<section id="route">
<h2>2. 按业务链路跳转</h2>
<p class="lead">这些路线把“显示端/外部协议入口 -> CPU3 内部通信 -> CPU2 测量执行 -> 状态回读”串起来，避免在两个 CPU 的目录之间硬切。</p>
<div class="grid">
{''.join(route_cards)}
</div>
</section>
<section id="cpu2">
<h2>3. CPU2 流程页面</h2>
<p class="lead">CPU2 是测量执行、运动控制、故障管理、参数保存和对 CPU3 寄存器发布的核心。</p>
<div class="links">{link_pill(from_file, "cpu2_total", "CPU2 程序流程总览")}{link_pill(from_file, "cpu2_issue", "CPU2 问题点总表")}{cpu2_links}</div>
</section>
<section id="cpu3">
<h2>4. CPU3 流程页面</h2>
<p class="lead">CPU3 是显示端、外部协议网关、CPU2 Modbus 主站和本机参数维护入口。</p>
<div class="links">{link_pill(from_file, "cpu3_total", "CPU3 程序流程总览")}{cpu3_links}</div>
</section>
<section id="maintain">
<h2>5. 维护规则</h2>
<div class="grid">
<article class="card"><strong>单页继续按源码单独整理</strong><p>每个功能页仍需要独立阅读源码、单独画业务级 SVG，不用统一模板批量凑图。</p></article>
<article class="card"><strong>跨页只表达业务关系</strong><p>统一入口和跨 CPU 链路页不替代详细流程图，只说明上下游、触发源和结果去向。</p></article>
<article class="card"><strong>重新生成后恢复导航</strong><p>如果 CPU2/CPU3 流程 HTML 被重新生成，执行 <code>py tools/update_flow_navigation.py</code> 恢复统一入口和页面关联导航。</p></article>
</div>
</section>
</div>
</body>
</html>
"""


def make_cross_route() -> str:
    from_file = CROSS_ROUTE
    route_cards = []
    for route in ROUTES:
        route_cards.append(
            f'<article class="card route-card" id="{html.escape(str(route["id"]))}">'
            f'<h3>{html.escape(route["title"])}</h3>'
            f'<p>{html.escape(route["summary"])}</p>'
            f'<div class="steps">{route_steps(from_file, route)}</div>'
            "</article>"
        )
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>跨 CPU 业务链路导航</title>
<style>
{GLOBAL_CSS}
</style>
</head>
<body>
<div class="wrap">
<header class="hero">
<h1>跨 CPU 业务链路导航</h1>
<p>按整机业务把 CPU3 入口、内部 Modbus、CPU2 命令入口、测量执行、状态回读和显示/协议响应串成连续路线。这里解决“从一个 HTML 跳到另一个 HTML 太生硬”的问题。</p>
<div class="meta-grid">
<div class="meta"><span>入口侧</span><strong>菜单 / 外部协议 / 显示</strong></div>
<div class="meta"><span>通道</span><strong>CPU3 UART5 Modbus</strong></div>
<div class="meta"><span>执行侧</span><strong>CPU2 测量状态机</strong></div>
<div class="meta"><span>回读侧</span><strong>寄存器 / 显示 / 协议响应</strong></div>
</div>
</header>
<nav class="topnav">
<a href="{html.escape(rel_href(from_file, "global"))}">统一入口</a>
<a href="#map">整机闭环图</a>
<a href="#oil">液位</a>
<a href="#water">水位</a>
<a href="#density">密度</a>
<a href="#param">参数</a>
<a href="#fault">故障</a>
<a href="#external">外部协议</a>
</nav>
<section id="map">
<h2>1. 整机业务闭环图</h2>
<p class="lead">图中只写业务动作，不写函数名。函数名和源码证据保留在各自详细页面里。</p>
<div class="flow-wrap">
<svg class="route-svg" viewBox="0 0 1200 760" role="img" aria-label="CPU2 CPU3 跨 CPU 业务闭环">
<defs>
<marker id="route-arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="#61758e"/></marker>
<marker id="route-arrow-red" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="#b84a55"/></marker>
</defs>
<rect class="node ext" x="70" y="55" width="230" height="78" rx="10"/><text class="nt" x="185" y="88">用户/上位机</text><text class="ns" x="185" y="111">按键菜单、DSM、Wartsila、SI7000</text>
<rect class="node cpu3" x="390" y="55" width="260" height="78" rx="10"/><text class="nt" x="520" y="88">CPU3 入口映射</text><text class="ns" x="520" y="111">菜单确认、协议解析、参数检查</text>
<rect class="node cpu3" x="770" y="55" width="260" height="78" rx="10"/><text class="nt" x="900" y="88">CPU3 内部 Modbus 主站</text><text class="ns" x="900" y="111">写 CPU2 命令/参数，轮询输入结果</text>
<rect class="node cpu2" x="770" y="215" width="260" height="82" rx="10"/><text class="nt" x="900" y="250">CPU2 通信寄存器入口</text><text class="ns" x="900" y="273">保持寄存器写入、输入寄存器发布</text>
<rect class="node cpu2" x="390" y="215" width="260" height="82" rx="10"/><text class="nt" x="520" y="250">CPU2 测量命令入口</text><text class="ns" x="520" y="273">公共准备、命令分发、状态切换</text>
<rect class="node cpu2" x="70" y="215" width="230" height="82" rx="10"/><text class="nt" x="185" y="250">CPU2 运动/传感支撑</text><text class="ns" x="185" y="273">电机、位置、传感、称重</text>
<rect class="node cpu2" x="160" y="395" width="260" height="92" rx="10"/><text class="nt" x="290" y="430">CPU2 测量执行</text><text class="ns" x="290" y="453">液位、水位、密度、回零、输出</text>
<rect class="node error" x="490" y="395" width="230" height="92" rx="10"/><text class="nt" x="605" y="430">故障/异常出口</text><text class="ns" x="605" y="453">SET_ERROR、命令切换、状态恢复</text>
<rect class="node state" x="800" y="395" width="250" height="92" rx="10"/><text class="nt" x="925" y="430">CPU2 状态发布</text><text class="ns" x="925" y="453">测量结果、设备状态、错误码</text>
<rect class="node cpu3" x="800" y="575" width="250" height="82" rx="10"/><text class="nt" x="925" y="610">CPU3 缓存刷新</text><text class="ns" x="925" y="633">g_measurement、参数镜像、点表</text>
<rect class="node cpu3" x="390" y="575" width="260" height="82" rx="10"/><text class="nt" x="520" y="610">显示和外部协议响应</text><text class="ns" x="520" y="633">状态页刷新、DSM/Wartsila/SI7000 读出</text>
<path class="edge" d="M300 94 L390 94" marker-end="url(#route-arrow)"/>
<path class="edge" d="M650 94 L770 94" marker-end="url(#route-arrow)"/>
<path class="edge" d="M900 133 L900 215" marker-end="url(#route-arrow)"/>
<path class="edge" d="M770 256 L650 256" marker-end="url(#route-arrow)"/>
<path class="edge" d="M390 256 L300 256" marker-end="url(#route-arrow)"/>
<path class="edge" d="M185 297 C195 342 235 365 290 395" marker-end="url(#route-arrow)"/>
<path class="edge" d="M520 297 C450 335 360 360 290 395" marker-end="url(#route-arrow)"/>
<path class="edge-red" d="M420 441 L490 441" marker-end="url(#route-arrow-red)"/>
<path class="edge" d="M720 441 L800 441" marker-end="url(#route-arrow)"/>
<path class="edge" d="M925 487 L925 575" marker-end="url(#route-arrow)"/>
<path class="edge" d="M800 616 L650 616" marker-end="url(#route-arrow)"/>
<path class="edge" d="M520 575 C470 505 370 485 290 487" marker-end="url(#route-arrow)"/>
<path class="edge" d="M900 575 C1010 505 1030 300 1030 133" marker-end="url(#route-arrow)"/>
</svg>
</div>
</section>
<section>
<h2>2. 分业务路线</h2>
<p class="lead">每条路线都按“入口 -> 通道 -> CPU2 执行 -> 回读/显示”展开。点击步骤可直接进入对应的详细流程图页面。</p>
<div class="grid">
{''.join(route_cards)}
</div>
</section>
<section>
<h2>3. 使用建议</h2>
<div class="grid">
<article class="card"><strong>查一个命令为什么没执行</strong><p>先看 CPU3 是否把命令写入内部 Modbus，再看 CPU2 命令入口是否接收，最后看对应测量页的异常出口。</p></article>
<article class="card"><strong>查结果为什么没显示</strong><p>先看 CPU2 是否发布输入寄存器，再看 CPU3 内部轮询是否刷新缓存，最后看显示页或外部协议页。</p></article>
<article class="card"><strong>查参数为什么没生效</strong><p>区分 CPU3 本机参数、CPU2 设备参数和协议缓存，按参数链路逐页核对。</p></article>
</div>
</section>
</div>
</body>
</html>
"""


def update_root_readme() -> None:
    path = ROOT / "docs" / "README.md"
    if not path.exists():
        return
    text = read_text(path)
    if "`00_程序流程导航`" not in text:
        text = text.replace(
            "| `00_构建与版本` | CPU2/CPU3 构建说明、升级日志、版本改动与测试方案等版本交付资料 |",
            "| `00_程序流程导航` | CPU2/CPU3 程序流程统一入口、跨 CPU 业务链路和流程文档跳转导航 |\n"
            "| `00_构建与版本` | CPU2/CPU3 构建说明、升级日志、版本改动与测试方案等版本交付资料 |",
            1,
        )
    if "`00_程序流程导航/index.html`" not in text:
        marker = "| 文档 | 用途 |\n| --- | --- |\n"
        entry = (
            "| `00_程序流程导航/index.html` | CPU2/CPU3 程序流程统一入口，按整机业务链路跳转到 CPU2 和 CPU3 详细流程页 |\n"
            "| `00_程序流程导航/跨CPU业务链路.html` | 液位、水位、密度、参数、故障和外部协议的跨 CPU 路线图 |\n"
        )
        text = text.replace(marker, marker + entry, 1)
    write_text(path, text)


def generate_pages() -> None:
    write_text(GLOBAL_INDEX, make_global_index())
    write_text(CROSS_ROUTE, make_cross_route())


def inject_all_pages() -> None:
    keys = ["cpu2_total", "cpu2_issue", *CPU2_ORDER, "cpu3_total", *CPU3_ORDER]
    for key in keys:
        inject_page(key)


def validate_files() -> None:
    missing = [key for key, meta in PAGE_DEFS.items() if key not in {"global", "cross"} and not Path(meta["path"]).exists()]
    if missing:
        raise RuntimeError("Missing expected pages: " + ", ".join(missing))
    bad_encoding = []
    for path in [GLOBAL_INDEX, CROSS_ROUTE, *[page_path(key) for key in ["cpu2_total", "cpu2_issue", *CPU2_ORDER, "cpu3_total", *CPU3_ORDER]]]:
        text = read_text(path)
        if "\ufffd" in text or "锟" in text:
            bad_encoding.append(str(path))
    if bad_encoding:
        raise RuntimeError("Encoding replacement characters found: " + ", ".join(bad_encoding))


def main() -> None:
    generate_pages()
    inject_all_pages()
    update_root_readme()
    validate_files()
    print(f"updated flow navigation: {GLOBAL_INDEX}")
    print(f"updated cross cpu routes: {CROSS_ROUTE}")
    print("injected relation navigator into CPU2/CPU3 flow pages")


if __name__ == "__main__":
    main()
