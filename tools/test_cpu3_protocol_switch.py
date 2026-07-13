import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parent))

import cpu3_protocol_switch as switch_tool


class FakeConnection:
    def __init__(self, injected_response: bytes):
        self.injected_response = injected_response
        self.buffer = bytearray()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        return False

    def reset_input_buffer(self):
        self.buffer.clear()

    def reset_output_buffer(self):
        return None

    def write(self, data: bytes):
        self.buffer.extend(data)
        self.buffer.extend(self.injected_response)
        return len(data)

    def flush(self):
        return None

    @property
    def in_waiting(self):
        return len(self.buffer)

    def read(self, length: int):
        data = bytes(self.buffer[:length])
        del self.buffer[:length]
        return data


class FakeSerialModule:
    EIGHTBITS = 8
    STOPBITS_ONE = 1
    SerialException = OSError

    def __init__(self, injected_response: bytes):
        self.injected_response = injected_response

    def Serial(self, **kwargs):
        return FakeConnection(self.injected_response)


class ProtocolSwitchToolTests(unittest.TestCase):
    def test_golden_frames(self):
        self.assertEqual(
            switch_tool.build_switch_frame(1, 5).hex(),
            "01464c5444052c46",
        )
        self.assertEqual(
            switch_tool.build_ack_frame(1, 5).hex(),
            "01460005200e",
        )

    def test_local_echo_is_not_accepted(self):
        frame = switch_tool.build_switch_frame(1, 5)
        ack = switch_tool.build_ack_frame(1, 5)

        self.assertFalse(
            switch_tool.exchange(
                FakeSerialModule(b""),
                "COM_FAKE",
                switch_tool.PROFILES["dsm"],
                frame,
                ack,
                0.01,
            )
        )

    def test_ack_after_local_echo_is_accepted(self):
        frame = switch_tool.build_switch_frame(1, 5)
        ack = switch_tool.build_ack_frame(1, 5)

        self.assertTrue(
            switch_tool.exchange(
                FakeSerialModule(ack),
                "COM_FAKE",
                switch_tool.PROFILES["dsm"],
                frame,
                ack,
                0.01,
            )
        )


if __name__ == "__main__":
    unittest.main()
