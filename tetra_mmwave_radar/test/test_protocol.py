import pytest

from tetra_mmwave_radar import protocol


def test_parses_frames_split_across_reads() -> None:
    parser = protocol.FrameParser()
    first = parser.feed(b'$JYBSS,1, , ')
    second = parser.feed(b', *\r\n$JYBSS,0, , , *\r\n')
    assert first.presence_values == []
    assert second.presence_values == [True, False]


def test_reply_text_is_separated_from_frames() -> None:
    parser = protocol.FrameParser()
    result = parser.feed(b'sensorStop\r\nDone\r\nleapMMW:/>$JYBSS,1, , , *')
    assert result.presence_values == [True]
    assert result.text_lines == ['sensorStop', 'Done', 'leapMMW:/>']


def test_corrupted_frame_does_not_hide_next_frame() -> None:
    parser = protocol.FrameParser()
    result = parser.feed(b'$JYBSS,x$JYBSS,0, , , *')
    assert result.presence_values == [False]


def test_garbage_without_newline_is_bounded() -> None:
    parser = protocol.FrameParser()
    parser.feed(b'\xaa' * 1000)
    assert len(parser._buffer) < 300


def test_range_command_matches_vendor_units() -> None:
    assert protocol.build_range_command([0, 9]) == 'detRangeCfg -1 0 60'
    assert protocol.build_range_command([0, 1.5, 3, 4.5]) == 'detRangeCfg -1 0 10 20 30'


@pytest.mark.parametrize('bounds', [[], [1.0], [2, 1], [0, 1, 0.5, 2], [0, 10], [0, 1] * 5])
def test_invalid_ranges_rejected(bounds: list) -> None:
    with pytest.raises(ValueError):
        protocol.build_range_command(bounds)


def test_latency_command() -> None:
    assert protocol.build_latency_command(0, 0) == 'outputLatency -1 0 0'
    assert protocol.build_latency_command(1.0, 0.5) == 'outputLatency -1 40 20'
    with pytest.raises(ValueError):
        protocol.build_latency_command(-1, 0)


def test_all_settings_sent_while_stopped() -> None:
    commands = protocol.wrap_config_commands(['detRangeCfg -1 0 60', 'outputLatency -1 0 0'])
    assert commands[0] == protocol.CMD_STOP
    assert commands[-2:] == [protocol.CMD_SAVE, protocol.CMD_START]
    assert commands.count(protocol.CMD_START) == 1
