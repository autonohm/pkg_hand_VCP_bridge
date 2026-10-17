# Copyright 2026 PIB Robotics Team

import pytest

from hand_nucleo.protocol import (
    ProtocolError,
    SpeedMode,
    encode_command,
    format_finger,
    format_grip,
    format_hold,
    format_status,
    format_stop,
)
from hand_nucleo.serial_client import SerialTransportError, VcpSerialClient


def test_grip_variants():
    assert format_grip(0, 4) == "G:0:4"
    assert format_grip(1, 4, SpeedMode.GLOBAL, 50) == "G:1:4:V:50"
    assert (
        format_grip(
            1, 4, SpeedMode.AXIS, axis_speeds=[100, 80, 60, 60, 80, 50]
        )
        == "G:1:4:Vx:100,80,60,60,80,50"
    )


def test_finger_variants():
    assert format_finger(0, 0, 2000) == "F:0:0:2000"
    assert format_finger(1, 2, 4095, True, 100) == "F:1:2:4095:100"


def test_control_commands():
    assert format_stop(0) == "STOP:0"
    assert format_hold(1) == "HOLD:1"
    assert format_status() == "GET:STATUS"
    assert encode_command("G:1:4") == b"G:1:4\r\n"


@pytest.mark.parametrize(
    "call",
    [
        lambda: format_grip(2, 0),
        lambda: format_grip(0, 8),
        lambda: format_grip(0, 0, SpeedMode.GLOBAL, 101),
        lambda: format_grip(0, 0, SpeedMode.AXIS, axis_speeds=[50]),
        lambda: format_finger(0, 6, 0),
        lambda: format_finger(0, 0, 4096),
        lambda: format_finger(0, 0, 0, False, 10),
        lambda: format_stop(2),
    ],
)
def test_invalid_requests_are_rejected(call):
    with pytest.raises(ProtocolError):
        call()


def test_axis_speed_boundary_fits_parser_buffer():
    command = format_grip(
        1,
        7,
        SpeedMode.AXIS,
        axis_speeds=[100, 100, 100, 100, 100, 100],
    )
    assert len(command.encode("ascii")) <= 32


class FakeSerial:
    def __init__(self, responses):
        self.responses = list(responses)
        self.writes = []
        self.timeout = 1.0
        self.is_open = True

    def write(self, payload):
        self.writes.append(payload)
        return len(payload)

    def flush(self):
        pass

    def readline(self):
        return self.responses.pop(0) if self.responses else b""

    def close(self):
        self.is_open = False


def test_serial_client_reads_ack_and_uses_vcp_settings():
    fake = FakeSerial([b"OK\r\n"])
    factory_arguments = {}

    def factory(**kwargs):
        factory_arguments.update(kwargs)
        return fake

    client = VcpSerialClient("/dev/test", serial_factory=factory)

    response = client.transact("G:1:4")

    assert response.successful
    assert response.raw_response == "OK"
    assert fake.writes == [b"G:1:4\r\n"]
    assert factory_arguments["baudrate"] == 460800
    assert factory_arguments["bytesize"] == 8
    assert factory_arguments["parity"] == "N"
    assert factory_arguments["stopbits"] == 1


def test_serial_client_collects_multiline_status():
    fake = FakeSerial(
        [
            b"[STATUS] RightHand: (not detailed)\r\n",
            b"[STATUS] LeftHand: (not detailed)\r\n",
            b"OK\n",
        ]
    )
    client = VcpSerialClient("/dev/test", serial_factory=lambda **_: fake)

    response = client.transact("GET:STATUS")

    assert response.status_lines == (
        "[STATUS] RightHand: (not detailed)",
        "[STATUS] LeftHand: (not detailed)",
    )
    assert response.terminal == "OK"


def test_serial_client_retries_err_buf_once():
    fake = FakeSerial([b"ERR BUF\r\n", b"OK\r\n"])
    client = VcpSerialClient(
        "/dev/test", retry_delay=0, serial_factory=lambda **_: fake
    )

    response = client.transact("STOP:1")

    assert response.successful
    assert fake.writes == [b"STOP:1\r\n", b"STOP:1\r\n"]


def test_serial_client_drops_connection_on_timeout():
    fake = FakeSerial([])
    client = VcpSerialClient(
        "/dev/test", response_timeout=0.01, serial_factory=lambda **_: fake
    )

    with pytest.raises(SerialTransportError, match="timed out"):
        client.transact("HOLD:1")

    assert not fake.is_open
    assert not client.connected
