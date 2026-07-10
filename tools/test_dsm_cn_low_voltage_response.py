from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DSM_SOURCE = ROOT / "LTD_MAIN_CPU2" / "Services" / "Sensor" / "dsm_sensor_communication.c"
SENSOR_SOURCE = ROOT / "LTD_MAIN_CPU2" / "Services" / "Sensor" / "sensor.c"


def _extract_function(source: str, name: str) -> str:
    start = source.index(f"uint32_t {name}(")
    brace = source.index("{", start)
    depth = 0
    for pos in range(brace, len(source)):
        if source[pos] == "{":
            depth += 1
        elif source[pos] == "}":
            depth -= 1
            if depth == 0:
                return source[start : pos + 1]
    raise AssertionError(f"function {name} body not found")


def test_cn_response_accepts_low_voltage_prefix():
    source = DSM_SOURCE.read_text(encoding="gbk")
    body = _extract_function(source, "Read_VibrationTube_ID")

    compact = "".join(body.split())
    prefix_guard = "if((*p!='N')&&(*p!='E')&&(*p!='e')){returnSENSOR_RESP_FORMAT_ERROR;}"
    assert prefix_guard in compact
    assert compact.index(prefix_guard) < compact.index("memcpy(id_out,p,id_len);")
    assert "while(*end!='\\0'&&*end!='\\r'&&*end!='\\n'&&*end!='*')" in compact


def _parse_dsm_text_id_reference(id_text: str) -> int:
    """按 Sensor_ParseDsmTextId 的数字提取契约计算预期编号。"""
    digits = [char for char in id_text if "0" <= char <= "9"]
    if not digits:
        raise ValueError("DSM text id contains no digits")
    value = int("".join(digits))
    if value > 0xFFFFFFFF:
        raise ValueError("DSM text id exceeds uint32_t")
    return value


def test_cn_low_voltage_prefix_keeps_the_same_sensor_id_contract():
    sensor_source = SENSOR_SOURCE.read_text(encoding="gbk")
    parse_body = "".join(_extract_function(sensor_source, "Sensor_ParseDsmTextId").split())
    probe_body = "".join(_extract_function(sensor_source, "Sensor_ProbeDsmSensor").split())

    assert "if((*id_text>='0')&&(*id_text<='9'))" in parse_body
    assert "if(value>((UINT32_MAX-digit)/10U)){returnSENSOR_RESP_FORMAT_ERROR;}" in parse_body
    assert "if(has_digit==0U){returnSENSOR_RESP_FORMAT_ERROR;}" in parse_body
    assert probe_body.index("Read_VibrationTube_ID(") < probe_body.index("Sensor_ParseDsmTextId(")
    assert probe_body.index("Sensor_ParseDsmTextId(") < probe_body.index("DSM_EnableDensityMode(")

    expected = 2009924
    assert _parse_dsm_text_id_reference("N2009924H") == expected
    assert _parse_dsm_text_id_reference("E2009924H") == expected
    assert _parse_dsm_text_id_reference("e2009924H") == expected
    assert _parse_dsm_text_id_reference("N4294967295H") == 0xFFFFFFFF

    for invalid in ("N", "E", "e", "X", "ERROR", "N4294967296H", "E999999999999999999H"):
        try:
            _parse_dsm_text_id_reference(invalid)
        except ValueError:
            continue
        raise AssertionError(f"invalid DSM text id was accepted: {invalid}")


if __name__ == "__main__":
    test_cn_response_accepts_low_voltage_prefix()
    test_cn_low_voltage_prefix_keeps_the_same_sensor_id_contract()
    print("DSM CN low-voltage response and sensor-id parsing contract ok")
