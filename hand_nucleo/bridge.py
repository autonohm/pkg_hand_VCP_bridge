# Copyright 2026 PIB Robotics Team

"""ROS2 services for the PIB hand Nucleo VCP serial protocol."""

from __future__ import annotations

import rclpy
from datatypes.srv import (
    NucleoFinger,
    NucleoGrip,
    NucleoSideCommand,
    NucleoStatus,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from hand_nucleo.protocol import (
    ProtocolError,
    format_finger,
    format_grip,
    format_hold,
    format_status,
    format_stop,
)
from hand_nucleo.serial_client import SerialTransportError, VcpSerialClient


class HandNucleoBridge(Node):
    """Expose typed ROS services and forward commands to the Nucleo."""

    def __init__(self) -> None:
        super().__init__("hand_nucleo_bridge")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 460800)
        self.declare_parameter("response_timeout", 1.0)
        self.declare_parameter("reconnect_interval", 1.0)
        self.declare_parameter("retry_delay", 0.05)

        self._client = VcpSerialClient(
            port=str(self.get_parameter("serial_port").value),
            baudrate=int(self.get_parameter("baudrate").value),
            response_timeout=float(
                self.get_parameter("response_timeout").value
            ),
            retry_delay=float(self.get_parameter("retry_delay").value),
        )
        self._reported_connection: bool | None = None
        callback_group = ReentrantCallbackGroup()

        self.create_service(
            NucleoGrip,
            "grip",
            self._grip_callback,
            callback_group=callback_group,
        )
        self.create_service(
            NucleoFinger,
            "finger",
            self._finger_callback,
            callback_group=callback_group,
        )
        self.create_service(
            NucleoSideCommand,
            "stop",
            self._stop_callback,
            callback_group=callback_group,
        )
        self.create_service(
            NucleoSideCommand,
            "hold",
            self._hold_callback,
            callback_group=callback_group,
        )
        self.create_service(
            NucleoStatus,
            "status",
            self._status_callback,
            callback_group=callback_group,
        )

        self.create_timer(
            float(self.get_parameter("reconnect_interval").value),
            self._reconnect_callback,
            callback_group=callback_group,
        )
        self.get_logger().info(
            "Hand Nucleo bridge started on "
            f"{self._client.port} at {self._client.baudrate} baud"
        )

    def _grip_callback(self, request, response):
        try:
            command = format_grip(
                side=request.side,
                grip_id=request.grip_id,
                speed_mode=request.speed_mode,
                speed_percent=request.speed_percent,
                axis_speeds=request.axis_speeds,
            )
        except ProtocolError as exc:
            return self._set_command_failure(response, str(exc))
        return self._send_command(command, response)

    def _finger_callback(self, request, response):
        try:
            command = format_finger(
                side=request.side,
                finger=request.finger,
                position=request.position,
                use_speed=request.use_speed,
                speed=request.speed,
            )
        except ProtocolError as exc:
            return self._set_command_failure(response, str(exc))
        return self._send_command(command, response)

    def _stop_callback(self, request, response):
        try:
            command = format_stop(request.side)
        except ProtocolError as exc:
            return self._set_command_failure(response, str(exc))
        return self._send_command(command, response)

    def _hold_callback(self, request, response):
        try:
            command = format_hold(request.side)
        except ProtocolError as exc:
            return self._set_command_failure(response, str(exc))
        return self._send_command(command, response)

    def _status_callback(self, _request, response):
        try:
            firmware_response = self._client.transact(format_status())
        except SerialTransportError as exc:
            self._record_connection(False)
            response.successful = False
            response.message = str(exc)
            response.status_lines = []
            response.raw_response = ""
            return response

        self._record_connection(True)
        response.successful = firmware_response.successful
        response.message = firmware_response.terminal
        response.status_lines = list(firmware_response.status_lines)
        response.raw_response = firmware_response.raw_response
        return response

    def _send_command(self, command: str, response):
        try:
            firmware_response = self._client.transact(command)
        except SerialTransportError as exc:
            self._record_connection(False)
            return self._set_command_failure(response, str(exc))

        self._record_connection(True)
        response.successful = firmware_response.successful
        response.message = firmware_response.terminal
        response.raw_response = firmware_response.raw_response
        return response

    @staticmethod
    def _set_command_failure(response, message: str):
        response.successful = False
        response.message = message
        response.raw_response = ""
        return response

    def _reconnect_callback(self) -> None:
        if self._client.connected:
            self._record_connection(True)
            return
        self._record_connection(self._client.try_connect())

    def _record_connection(self, connected: bool) -> None:
        if self._reported_connection == connected:
            return
        self._reported_connection = connected
        if connected:
            self.get_logger().info("Nucleo serial connection established")
        else:
            self.get_logger().warn(
                f"Nucleo serial connection unavailable: {self._client.port}"
            )

    def destroy_node(self):
        self._client.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandNucleoBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
