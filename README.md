# CUBE

> CUBE 是二代计量仪的嵌入式固件与工程资料仓库。仓库以 CPU2 主控固件和 CPU3 显示通信固件为核心，同时维护两块板之间的共享协议、构建配置、版本记录和项目文档。

## 项目组成

| 工程 | 主要职责 |
| --- | --- |
| `LTD_MAIN_CPU2/` | 测量流程、运动控制、传感器接入、参数与 FRAM 持久化、故障处理、输出控制，以及 CPU2/CPU3 共享数据的权威实现 |
| `LTD_DISPLAY_CPU3/` | OLED 显示、菜单与按键、CPU2 数据轮询和参数同步，以及 DSM、Wärtsilä、LTD、LH、SI 等外部协议适配 |

两块板的主要关系如下：

```text
上位机 / PLC / 外部系统
          ↕
CPU3 显示与通信板
          ↕  UART5 + RS485（共享 Modbus）
CPU2 主控测量板
          ↕
传感器 / 电机 / 编码器 / 称重 / AO / HART 等外设
```

CPU2 负责核心测量与设备控制，CPU3 负责人机界面和外部通信适配。涉及共享参数、测量结果、命令或故障语义的改动，需要同时核对 CPU2、CPU3、共享寄存器映射和协议版本。

## 仓库目录

### Git 中维护的主要内容

| 路径 | 用途 |
| --- | --- |
| `LTD_MAIN_CPU2/` | CPU2 STM32F429 固件工程 |
| `LTD_DISPLAY_CPU3/` | CPU3 STM32F429 固件工程 |
| `docs/` | 唯一正式项目文档源，包含构建、协议、需求、问题、测试、功能安全和硬件资料 |
| `cmake/` | CPU2/CPU3 共用的 Arm GNU 工具链配置 |
| `.github/workflows/` | CPU2/CPU3 持续集成构建配置 |
| `.agents/`、`AGENTS.md` | 本仓库的自动化协作和工程维护规则 |
| `CHANGELOG.md` | 固件版本改动记录 |

### 本机工作区内容

以下目录可能存在于开发机，但不作为 Git 业务资料维护：

| 路径 | 用途 |
| --- | --- |
| `build/LTD_MAIN_CPU2/`、`build/LTD_DISPLAY_CPU3/` | 两个固件工程的本地构建输出 |
| `tools/` | 本机构建、检查、文档预览和 Git 门禁工具 |
| `docs/00_程序流程导航/` | 本机生成并同步的程序流程导航 |
| `docs-site/` | 从 `docs/` 同步生成的本机文档预览站点 |
| `tmp/` | 可清理的任务临时产物 |
| `outputs/` | 需要在本机长期保留、但默认不提交的验证证据 |

## 快速构建

### 环境要求

- CMake 3.20 或更高版本
- Ninja
- Arm GNU Toolchain，并确保 `arm-none-eabi-gcc`、`arm-none-eabi-objcopy`、`arm-none-eabi-size` 等命令已加入 `PATH`

### CPU2

```bash
cmake -S LTD_MAIN_CPU2 -B build/LTD_MAIN_CPU2 -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug
cmake --build build/LTD_MAIN_CPU2
```

### CPU3

```bash
cmake -S LTD_DISPLAY_CPU3 -B build/LTD_DISPLAY_CPU3 -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug
cmake --build build/LTD_DISPLAY_CPU3
```

发布候选应使用全量清理构建：

```bash
cmake --build build/LTD_MAIN_CPU2 --clean-first
cmake --build build/LTD_DISPLAY_CPU3 --clean-first
```

构建目录会生成 `.elf`、`.hex`、`.bin` 和 `.map` 文件，其中还包括带固件版本号的 HEX。构建成功只能证明当前源码完成编译和链接，不能替代烧写、台架、硬件或现场验证。

更完整的环境、产物和发布前检查说明见：

- [CPU2 构建说明](docs/00_构建与版本/CPU2构建说明.md)
- [CPU3 构建说明](docs/00_构建与版本/CPU3构建说明.md)

## 代码入口

第一次阅读代码时，建议从以下入口开始：

| 目标 | CPU2 | CPU3 |
| --- | --- | --- |
| 启动与硬件初始化 | `LTD_MAIN_CPU2/Core/Src/main.c` | `LTD_DISPLAY_CPU3/Core/Src/main.c` |
| 应用初始化与主循环 | `LTD_MAIN_CPU2/Application/Src/app_main.c` | `LTD_DISPLAY_CPU3/Application/app_main.c` |
| 核心业务 | `LTD_MAIN_CPU2/Services/` | `LTD_DISPLAY_CPU3/Application/` |
| 通信实现 | `LTD_MAIN_CPU2/Services/Modbus/` | `LTD_DISPLAY_CPU3/Communication/` |

修改共享数据契约时，重点检查两端的 `system_parameter.h`、寄存器地址、结构体与寄存器之间的映射，以及 CPU3 的显示和外部协议消费点。

## 文档与版本入口

- [文档库入口](docs/README.md)：按构建、协议、需求、问题、测试、功能安全和硬件资料分类查找。
- [构建与版本文档](docs/00_构建与版本/README.md)：查看构建说明、版本总览、升级和版本测试资料。
- [协议与寄存器文档](docs/01_协议与寄存器/README.md)：查看 CPU2/CPU3 共享协议、外部协议和传感器协议。
- [CHANGELOG](CHANGELOG.md)：查看已发布或待发布的固件改动。

README 不固定抄录当前固件和协议版本，避免首页随版本演进失真。需要确认当前口径时，以以下来源为准：

- CPU2 固件版本：`LTD_MAIN_CPU2/Application/Inc/app_version.h`
- CPU3 固件版本：`LTD_DISPLAY_CPU3/Application/app_version.h`
- CPU2 共享协议定义：`LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.h`
- CPU3 共享协议定义：`LTD_DISPLAY_CPU3/Application/system_param/system_parameter.h`
- 版本组合、兼容性和验证记录：`CHANGELOG.md` 与 `docs/00_构建与版本/版本改动与测试/`

## 开发约定

- 正式文档只维护在 `docs/`；`docs-site/` 只是本机预览层。
- 修改文件前确认编码和换行。部分历史固件源码使用 GBK/CP936，避免批量转码造成中文乱码。
- 共享协议、参数存储、故障码和显示语义变更必须核对通信两端及对应正式文档。
- 不要把构建或静态检查结果表述为真实硬件、通信、运动、输出或现场验证。
- 自动化执行和提交前门禁以 [AGENTS.md](AGENTS.md) 及仓库级 CUBE skill 为准。
