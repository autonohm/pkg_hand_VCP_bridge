# Copyright 2026 PIB Robotics Team

"""Thread-safe request/response transport for the hand VCP connection."""

from __future__ import annotations

import threading
import time
from typing import Any, Callable

import serial

from hand_nucleo.protocol import FirmwareResponse, encode_command


class SerialTransportError(RuntimeError):
    """Raised when a command cannot be sent or acknowledged over serial."""


class VcpSerialClient:
    """Serialize VCP requests and reconnect after transient device failures."""

    def __init__(
        self,
        port: str,
        baudrate: int = 460800,
        response_timeout: float = 1.0,
        retry_delay: float = 0.05,
        serial_factory: Callable[..., Any] | None = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.response_timeout = response_timeout
        self.retry_delay = retry_delay
        self._serial_factory = serial_factory or serial.Serial
        self._serial: Any | None = None
        self._lock = threading.Lock()

    @property
    def connected(self) -> bool:
        """Return whether an open serial handle is currently available."""
        with self._lock:
            return self._is_open()

    def try_connect(self) -> bool:
        """Open the configured port if it is not already open."""
        with self._lock:
            try:
                self._ensure_connected()
            except Exception:
                self._drop_connection()
                return False
            return True

    def close(self) -> None:
        """Close the serial device, if it is open."""
        with self._lock:
            self._drop_connection()

    def transact(self, command: str) -> FirmwareResponse:
        """Send one command and read its terminal OK or ERR response."""
        payload = encode_command(command)

        with self._lock:
            for attempt in range(2):
                try:
                    self._ensure_connected()
                    self._serial.write(payload)
                    self._serial.flush()
                    response = self._read_response()
                except SerialTransportError:
                    self._drop_connection()
                    raise
                except Exception as exc:
                    self._drop_connection()
                    raise SerialTransportError(str(exc)) from exc

                if response.terminal == "ERR BUF" and attempt == 0:
                    time.sleep(self.retry_delay)
                    continue
                return response

        raise SerialTransportError("serial request failed")

    def _ensure_connected(self) -> None:
        if self._is_open():
            return

        self._serial = self._serial_factory(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.response_timeout,
            write_timeout=self.response_timeout,
        )

    def _is_open(self) -> bool:
        return self._serial is not None and bool(
            getattr(self._serial, "is_open", True)
        )

    def _drop_connection(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None

    def _read_response(self) -> FirmwareResponse:
        lines: list[str] = []
        deadline = time.monotonic() + self.response_timeout

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise SerialTransportError("serial response timed out")

            previous_timeout = getattr(self._serial, "timeout", None)
            if previous_timeout is not None:
                self._serial.timeout = remaining
            try:
                raw_line = self._serial.readline()
            finally:
                if previous_timeout is not None:
                    self._serial.timeout = previous_timeout

            if not raw_line:
                raise SerialTransportError("serial response timed out")

            try:
                line = raw_line.decode("ascii").rstrip("\r\n")
            except UnicodeDecodeError as exc:
                raise SerialTransportError(
                    "serial response is not ASCII"
                ) from exc

            lines.append(line)
            if line == "OK" or line == "ERR" or line.startswith("ERR "):
                return FirmwareResponse(tuple(lines))
