# DSM 协议 golden frames

本目录保存 DSM CPU1 对外协议的确定性帧向量。

| 文件 | 用途 |
|---|---|
| [dsm_protocol_golden_frames.json](dsm_protocol_golden_frames.json) | 正常 ACK、数据帧、版本帧、错误帧、V6 扫频帧和软件复位无响应样例 |

证据类型：

- `documented`：协议正文已经列出的确定性样例。
- `synthetic`：按字段格式和历史 BCC 算法构造的契约向量，不代表目标板抓包。
- `documented_payload_synthetic_bcc`：有效载荷来自正文，BCC 和完整帧由契约算法补齐。
- `synthetic_checksum_boundary_nonsemantic_payload`：只验证 BCC 为 NUL、LF 或 CR 时的固定长度接收；载荷不代表合法业务数值。

所有向量由 `tools\check_dsm_protocol_contract.py` 校验。真实设备抓包后，应增加独立的现场证据标识，不得把静态向量改写成实机验证结果。
