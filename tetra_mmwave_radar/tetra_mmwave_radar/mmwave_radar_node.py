"""ROS 2 node for the DFRobot SEN0395 24GHz mmWave presence radar.

Topics (relative to the node namespace, default /mmwave_radar):
  presence         std_msgs/Bool   every frame from the sensor
  presence_changed std_msgs/Bool   only when the value flips (latched)
  diagnostics      diagnostic_msgs/DiagnosticArray  1 Hz link health
Services:
  apply_config     std_srvs/Trigger  write detection_ranges + latencies to flash
  factory_reset    std_srvs/Trigger  restore factory settings
  start / stop     std_srvs/Trigger  sensorStart / sensorStop (not persisted)
"""

import threading
import time
from typing import List, Optional, Tuple

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from tetra_mmwave_radar import protocol

# The vendor measured that the sensor drops commands sent closer than 1 s apart.
COMMAND_INTERVAL_S = 1.0
RECONNECT_INTERVAL_S = 2.0
# Named so the value survives CLI / YAML parsing (raw "\r\n" does not).
COMMAND_TERMINATORS = {'none': '', 'crlf': '\r\n', 'lf': '\n'}


class MmWaveRadarNode(Node):
    """Reads presence frames over UART and exposes sensor configuration."""

    def __init__(self) -> None:
        super().__init__('mmwave_radar')
        self._port = self.declare_parameter('port', '/dev/ttyUSB0').value
        self._baudrate = self.declare_parameter('baudrate', 115200).value
        self._frame_id = self.declare_parameter('frame_id', 'mmwave_radar_link').value
        self._stale_timeout_s = self.declare_parameter('stale_timeout_s', 3.0).value
        # Vendor lib sends no terminator; kept configurable because some
        # firmware revisions only accept newline-terminated commands.
        terminator_name = self.declare_parameter('command_terminator', 'none').value
        if terminator_name not in COMMAND_TERMINATORS:
            raise ValueError(f'command_terminator must be one of {list(COMMAND_TERMINATORS)}')
        self._terminator = COMMAND_TERMINATORS[terminator_name]
        self.declare_parameter('detection_ranges', [0.0, 9.0])
        self.declare_parameter('detect_latency_s', 0.0)
        self.declare_parameter('clear_latency_s', 0.0)
        apply_on_start = self.declare_parameter('apply_config_on_start', False).value
        start_on_open = self.declare_parameter('send_start_on_connect', True).value

        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._presence_pub = self.create_publisher(Bool, '~/presence', 10)
        self._changed_pub = self.create_publisher(Bool, '~/presence_changed', latched_qos)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.create_service(Trigger, '~/apply_config', self._on_apply_config)
        self.create_service(Trigger, '~/factory_reset', self._on_factory_reset)
        self.create_service(Trigger, '~/start', self._on_start)
        self.create_service(Trigger, '~/stop', self._on_stop)
        self.create_timer(1.0, self._publish_diagnostics)

        self._serial: Optional[serial.Serial] = None
        self._write_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._last_presence: Optional[bool] = None
        self._last_frame_time = 0.0
        self._frame_count = 0
        self._last_reply = ''
        self._pending_start = start_on_open
        self._pending_config = apply_on_start
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self.get_logger().info(f'mmWave radar on {self._port} @ {self._baudrate}')

    # ---------------------------------------------------------------- serial
    def _read_loop(self) -> None:
        """Own the port: (re)open it, parse frames, run deferred commands."""
        parser = protocol.FrameParser()
        while self._running and rclpy.ok():
            if self._serial is None and not self._open_port():
                time.sleep(RECONNECT_INTERVAL_S)
                continue
            try:
                chunk = self._serial.read(self._serial.in_waiting or 1)
            except (serial.SerialException, OSError) as error:
                self.get_logger().error(f'serial read failed: {error}; reconnecting')
                self._close_port()
                continue
            parsed = parser.feed(chunk)
            for line in parsed.text_lines:
                self._last_reply = line
                self.get_logger().info(f'sensor: {line}')
            for presence in parsed.presence_values:
                self._handle_presence(presence)

    def _open_port(self) -> bool:
        """Open the port and queue the start / config commands for it."""
        try:
            self._serial = serial.Serial(self._port, self._baudrate, timeout=0.2)
        except (serial.SerialException, OSError) as error:
            self.get_logger().warn(f'cannot open {self._port}: {error}',
                                   throttle_duration_sec=30.0)
            return False
        self.get_logger().info(f'opened {self._port}')
        # Commands block ~4 s, so they run off the reader thread to keep
        # frames flowing (and the reply lines visible in the log).
        threading.Thread(target=self._run_connect_commands, daemon=True).start()
        return True

    def _run_connect_commands(self) -> None:
        """Send queued commands once per connection."""
        if self._pending_config:
            self._pending_config = False
            ok, message = self._apply_config()
            log = self.get_logger().info if ok else self.get_logger().error
            log(f'apply_config_on_start: {message}')
        elif self._pending_start:
            # A sensor left in sensorStop state is silent until started.
            self._send_commands([protocol.CMD_START])

    def _close_port(self) -> None:
        with self._write_lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except (serial.SerialException, OSError):
                    pass
            self._serial = None

    def _send_commands(self, commands: List[str]) -> bool:
        """Write commands spaced by COMMAND_INTERVAL_S; False if port is gone."""
        with self._write_lock:
            for command in commands:
                if self._serial is None:
                    return False
                self.get_logger().info(f'-> {command}')
                try:
                    self._serial.write((command + self._terminator).encode('ascii'))
                except (serial.SerialException, OSError) as error:
                    self.get_logger().error(f'serial write failed: {error}')
                    return False
                time.sleep(COMMAND_INTERVAL_S)
        return True

    # -------------------------------------------------------------- presence
    def _handle_presence(self, presence: bool) -> None:
        with self._state_lock:
            changed = presence != self._last_presence
            self._last_presence = presence
            self._last_frame_time = time.monotonic()
            self._frame_count += 1
        self._presence_pub.publish(Bool(data=presence))
        if changed:
            self._changed_pub.publish(Bool(data=presence))
            self.get_logger().info(f'presence: {"DETECTED" if presence else "clear"}')

    def _publish_diagnostics(self) -> None:
        with self._state_lock:
            age_s = time.monotonic() - self._last_frame_time
            frame_count = self._frame_count
            presence = self._last_presence
        status = DiagnosticStatus(name='mmwave_radar', hardware_id=self._frame_id)
        if self._serial is None:
            status.level, status.message = DiagnosticStatus.ERROR, 'port not open'
        elif frame_count == 0 or age_s > self._stale_timeout_s:
            status.level, status.message = DiagnosticStatus.WARN, 'no frames'
        else:
            status.level, status.message = DiagnosticStatus.OK, 'ok'
        status.values = [
            KeyValue(key='port', value=str(self._port)),
            KeyValue(key='frames', value=str(frame_count)),
            KeyValue(key='presence', value=str(presence)),
            KeyValue(key='last_reply', value=self._last_reply),
        ]
        array = DiagnosticArray(status=[status])
        array.header.stamp = self.get_clock().now().to_msg()
        self._diag_pub.publish(array)

    # -------------------------------------------------------------- services
    def _apply_config(self) -> Tuple[bool, str]:
        """Validate parameters, then write range and latency to the sensor."""
        try:
            range_command = protocol.build_range_command(
                list(self.get_parameter('detection_ranges').value))
            latency_command = protocol.build_latency_command(
                float(self.get_parameter('detect_latency_s').value),
                float(self.get_parameter('clear_latency_s').value))
        except ValueError as error:
            return False, f'invalid parameter: {error}'
        commands = protocol.wrap_config_commands([range_command, latency_command])
        if not self._send_commands(commands):
            return False, 'serial port not available'
        return True, f'sent "{range_command}" and "{latency_command}"'

    def _on_apply_config(self, _request: Trigger.Request,
                         response: Trigger.Response) -> Trigger.Response:
        response.success, response.message = self._apply_config()
        return response

    def _on_factory_reset(self, _request: Trigger.Request,
                          response: Trigger.Response) -> Trigger.Response:
        response.success = self._send_commands(
            protocol.wrap_config_commands([protocol.CMD_FACTORY_RESET]))
        response.message = 'factory reset sent' if response.success else 'port not available'
        return response

    def _on_start(self, _request: Trigger.Request,
                  response: Trigger.Response) -> Trigger.Response:
        response.success = self._send_commands([protocol.CMD_START])
        response.message = 'sensorStart sent' if response.success else 'port not available'
        return response

    def _on_stop(self, _request: Trigger.Request,
                 response: Trigger.Response) -> Trigger.Response:
        response.success = self._send_commands([protocol.CMD_STOP])
        response.message = 'sensorStop sent' if response.success else 'port not available'
        return response

    def destroy_node(self) -> None:
        self._running = False
        self._reader.join(timeout=1.0)
        self._close_port()
        super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MmWaveRadarNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
