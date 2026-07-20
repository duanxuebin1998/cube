# DSM 协议机器数据

本目录保存 DSM CPU1 对外协议的机器可读维护源。当前规范版本为V5.4工作区，V5.3及更早版本继续作为历史能力快照保留。

| 文件或目录 | 用途 |
|---|---|
| [dsm_protocol_matrix.json](dsm_protocol_matrix.json) | 定义物理层、请求格式、历史 BCC、命令、响应 profile、版本标识和各版本完整命令能力 |
| [golden_frames/README.md](golden_frames/README.md) | 确定性协议帧及其证据边界 |

修改本目录数据后，应在仓库根目录运行：

```powershell
py tools\check_dsm_protocol_contract.py --write-doc
py tools\check_dsm_protocol_contract.py
```

机器矩阵负责结构化契约，`docs/01_协议与接口/DSM_CPU1对外通信协议.md` 负责协议解释、当前实现偏差、维护线差异和验证边界；两者必须由契约检查保持一致。
