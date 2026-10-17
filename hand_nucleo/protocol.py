# Copyright 2026 PIB Robotics Team

"""Validation and formatting for the PIB hand VCP protocol."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Sequence


MAX_COMMAND_BYTES = 32
LINE_ENDING = "\r\n"


class ProtocolError(ValueError):
    """Raised when a ROS request cannot be represented by the VCP protocol."""


class SpeedMode(IntEnum):
    """Supported speed variants of the grip command."""

    DEFAULT = 0
    GLOBAL = 1
    AXIS = 2


@dataclass(frozen=True)
class FirmwareResponse:
    """A complete firmware response, including its terminal line."""

    lines: tuple[str, ...]

    @property
    def terminal(self) -> str:
        """Return the final response line."""
        return self.lines[-1] if self.lines else ""

    @property
    def successful(self) -> bool:
        """Whether the firmware acknowledged the command."""
        return self.terminal == "OK"

    @property
    def raw_response(self) -> str:
        """Return all response lines without transport line endings."""
        return "\n".join(self.lines)

    @property
    def status_lines(self) -> tuple[str, ...]:
        """Return status payload lines from a GET:STATUS response."""
        return tuple(
            line for line in self.lines if line.startswith("[STATUS]")
        )


def _validate_integer(
    name: str, value: int, minimum: int, maximum: int
) -> None:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ProtocolError(f"{name} must be an integer")
    if not minimum <= value <= maximum:
        raise ProtocolError(f"{name} must be between {minimum} and {maximum}")


def _validate_side(side: int) -> None:
    _validate_integer("side", side, 0, 1)


def _validate_command_length(command: str) -> str:
    try:
        command_bytes = command.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ProtocolError(
            "command must contain ASCII characters only"
        ) from exc

    if len(command_bytes) > MAX_COMMAND_BYTES:
        raise ProtocolError(
            f"command is {len(command_bytes)} bytes; maximum is "
            f"{MAX_COMMAND_BYTES}"
        )
    return command


def encode_command(command: str) -> bytes:
    """Encode one validated command with the required CRLF terminator."""
    return (_validate_command_length(command) + LINE_ENDING).encode("ascii")


def format_grip(
    side: int,
    grip_id: int,
    speed_mode: int = SpeedMode.DEFAULT,
    speed_percent: int = 0,
    axis_speeds: Sequence[int] | None = None,
) -> str:
    """Format a G command and validate its complete request shape."""
    _validate_side(side)
    _validate_integer("grip_id", grip_id, 0, 7)
    _validate_integer("speed_mode", speed_mode, 0, 2)
    speeds = list(axis_speeds or [])

    if speed_mode == SpeedMode.DEFAULT:
        if speed_percent != 0 or speeds:
            raise ProtocolError(
                "default speed mode cannot contain speed values"
            )
        command = f"G:{side}:{grip_id}"
    elif speed_mode == SpeedMode.GLOBAL:
        _validate_integer("speed_percent", speed_percent, 0, 100)
        if speeds:
            raise ProtocolError("global speed mode cannot contain axis speeds")
        command = f"G:{side}:{grip_id}:V:{speed_percent}"
    else:
        if speed_percent != 0:
            raise ProtocolError(
                "axis speed mode cannot contain a global speed"
            )
        if len(speeds) != 6:
            raise ProtocolError("axis speed mode requires exactly six speeds")
        for index, speed in enumerate(speeds):
            _validate_integer(f"axis_speeds[{index}]", speed, 0, 100)
        command = f"G:{side}:{grip_id}:Vx:{','.join(map(str, speeds))}"

    return _validate_command_length(command)


def format_finger(
    side: int,
    finger: int,
    position: int,
    use_speed: bool = False,
    speed: int = 0,
) -> str:
    """Format an F command and validate its logical hand coordinates."""
    _validate_side(side)
    _validate_integer("finger", finger, 0, 5)
    _validate_integer("position", position, 0, 4095)
    _validate_integer("speed", speed, 0, 2**32 - 1)

    if not use_speed and speed != 0:
        raise ProtocolError("speed requires use_speed to be true")

    command = (
        f"F:{side}:{finger}:{position}:{speed}"
        if use_speed
        else f"F:{side}:{finger}:{position}"
    )
    return _validate_command_length(command)


def format_stop(side: int) -> str:
    """Format a STOP command."""
    _validate_side(side)
    return f"STOP:{side}"


def format_hold(side: int) -> str:
    """Format a HOLD command."""
    _validate_side(side)
    return f"HOLD:{side}"


def format_status() -> str:
    """Format the status query command."""
    return "GET:STATUS"
