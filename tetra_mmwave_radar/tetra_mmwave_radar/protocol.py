"""DFRobot SEN0395 24GHz mmWave presence radar UART protocol.

Ported from the vendor Arduino library (DFRobot_mmWave_Radar, V1.0) so it
can run on a PC over a USB-UART adapter. Kept free of rclpy so it can be unit
tested without a ROS environment.

Sensor output: a stream of 15-byte ASCII frames ``$JYBSS,<0|1>, , , *``,
possibly interleaved with command echoes / ``Done`` / ``Error`` replies.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

FRAME_HEADER = b'$JYBSS,'
FRAME_LENGTH = 15
PRESENCE_INDEX = 7

# The sensor refuses to reconfigure while running, so every config change is
# wrapped in stop -> change -> save -> start (same order as the vendor lib).
CMD_STOP = 'sensorStop'
CMD_START = 'sensorStart'
CMD_SAVE = 'saveCfg 0x45670123 0xCDEF89AB 0x956128C6 0xDF54AC89'
CMD_FACTORY_RESET = 'factoryReset 0x45670123 0xCDEF89AB 0x956128C6 0xDF54AC89'

RANGE_UNIT_M = 0.15
LATENCY_UNIT_S = 0.025
MAX_RANGE_SEGMENTS = 4
MAX_RANGE_M = 9.0
MAX_LATENCY_S = 1638.375
MAX_PENDING_TEXT = 256


@dataclass
class ParseResult:
    """Presence values and non-frame text lines extracted from one feed."""

    presence_values: List[bool] = field(default_factory=list)
    text_lines: List[str] = field(default_factory=list)


class FrameParser:
    """Incremental parser that tolerates partial frames and stray bytes."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, chunk: bytes) -> ParseResult:
        """Append raw serial bytes and return everything decodable so far.

        @param chunk raw bytes read from the serial port
        @returns presence values in arrival order plus reply text lines
        """
        self._buffer.extend(chunk)
        result = ParseResult()
        while True:
            header_index = self._buffer.find(FRAME_HEADER)
            if header_index < 0:
                self._drain_complete_lines(result)
                return result
            # Whatever precedes a frame header is a finished reply.
            self._emit_text(header_index, result)
            if len(self._buffer) < FRAME_LENGTH:
                return result
            presence_char = self._buffer[PRESENCE_INDEX:PRESENCE_INDEX + 1]
            if presence_char in (b'0', b'1'):
                result.presence_values.append(presence_char == b'1')
                del self._buffer[:FRAME_LENGTH]
            else:
                # Corrupted frame: drop only the '$' so a real header that
                # starts inside it is not lost.
                del self._buffer[:1]

    def _drain_complete_lines(self, result: ParseResult) -> None:
        """Emit newline-terminated text, keeping a possible partial header.

        A partial line longer than MAX_PENDING_TEXT is flushed anyway so a
        wrong device on the port cannot grow the buffer without bound.
        """
        last_newline = self._buffer.rfind(b'\n')
        if last_newline >= 0:
            self._emit_text(last_newline + 1, result)
        if len(self._buffer) > MAX_PENDING_TEXT:
            self._emit_text(len(self._buffer) - len(FRAME_HEADER), result)

    def _emit_text(self, end: int, result: ParseResult) -> None:
        """Move buffer[:end] out as stripped, non-empty text lines."""
        if end <= 0:
            return
        for raw_line in bytes(self._buffer[:end]).splitlines():
            line = raw_line.decode('ascii', errors='replace').strip()
            if line:
                result.text_lines.append(line)
        del self._buffer[:end]


def validate_ranges(range_bounds_m: Sequence[float]) -> List[Tuple[float, float]]:
    """Turn a flat [start, end, start, end, ...] list into checked segments.

    @param range_bounds_m flat list of segment bounds in metres
    @returns list of (start, end) pairs
    @throws ValueError if the list breaks the sensor's segment rules
    """
    if len(range_bounds_m) == 0 or len(range_bounds_m) % 2 != 0:
        raise ValueError('detection_ranges needs an even, non-zero count')
    segments = [(float(range_bounds_m[i]), float(range_bounds_m[i + 1]))
                for i in range(0, len(range_bounds_m), 2)]
    if len(segments) > MAX_RANGE_SEGMENTS:
        raise ValueError(f'at most {MAX_RANGE_SEGMENTS} segments are allowed')
    previous_end: Optional[float] = None
    for start, end in segments:
        if start < 0.0 or end > MAX_RANGE_M:
            raise ValueError(f'segment {start}-{end} m outside 0-{MAX_RANGE_M} m')
        if end <= start:
            raise ValueError(f'segment {start}-{end} m: end must exceed start')
        if previous_end is not None and start <= previous_end:
            raise ValueError('segments must be ascending and non-overlapping')
        previous_end = end
    return segments


def build_range_command(range_bounds_m: Sequence[float]) -> str:
    """Build ``detRangeCfg -1 ...`` in 0.15 m units (truncated like the vendor lib).

    @param range_bounds_m flat list of segment bounds in metres
    @returns the command string without terminator
    """
    units: List[int] = []
    for start, end in validate_ranges(range_bounds_m):
        units.extend([int(start / RANGE_UNIT_M), int(end / RANGE_UNIT_M)])
    return 'detRangeCfg -1 ' + ' '.join(str(unit) for unit in units)


def build_latency_command(detect_delay_s: float, clear_delay_s: float) -> str:
    """Build ``outputLatency -1 ...`` in 25 ms units.

    @param detect_delay_s delay before reporting a newly detected target
    @param clear_delay_s delay before reporting that the target left
    @returns the command string without terminator
    @throws ValueError if either delay is outside 0..1638.375 s
    """
    for delay_s in (detect_delay_s, clear_delay_s):
        if not 0.0 <= delay_s <= MAX_LATENCY_S:
            raise ValueError(f'latency {delay_s} s outside 0-{MAX_LATENCY_S} s')
    detect_units = int(detect_delay_s / LATENCY_UNIT_S)
    clear_units = int(clear_delay_s / LATENCY_UNIT_S)
    return f'outputLatency -1 {detect_units} {clear_units}'


def wrap_config_commands(config_commands: Sequence[str]) -> List[str]:
    """Return stop -> config... -> save -> start for one or more settings.

    The sensor ignores config while running, so every setting must sit
    between a single stop and the final start.

    @param config_commands detRangeCfg / outputLatency / factoryReset commands
    @returns ordered commands to send, one per ~1 s
    """
    return [CMD_STOP, *config_commands, CMD_SAVE, CMD_START]
