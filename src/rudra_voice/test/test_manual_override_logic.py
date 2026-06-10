from rudra_voice.command_guard_node import DEFAULT_MOTION_DURATION_SEC, has_meaningful_manual_input
from rudra_voice.intent_parser import default_reply_for_skill
from rudra_voice.tts import make_tts_backend
from rudra_voice.voice_node import MOTION_COMMANDS


def test_espeak_backend_exposes_async_speech() -> None:
    backend = make_tts_backend('espeak')

    assert hasattr(backend, 'speak_async')


def test_small_joystick_noise_does_not_look_like_manual_override() -> None:
    assert not has_meaningful_manual_input([0.08, 0.0, 0.0, 0.0], [], 0.05)


def test_clear_joystick_motion_still_counts_as_manual_override() -> None:
    assert has_meaningful_manual_input([0.35, 0.0, 0.0, 0.0], [], 0.05)


def test_buttons_always_count_as_manual_override() -> None:
    assert has_meaningful_manual_input([0.0, 0.0, 0.0, 0.0], [1, 0, 0, 0], 0.05)


def test_default_motion_duration_falls_back_to_five_seconds() -> None:
    assert DEFAULT_MOTION_DURATION_SEC == 5.0


def test_motion_reply_uses_five_second_duration() -> None:
    assert 'five seconds' in default_reply_for_skill('move_forward_slow').lower()


def test_voice_motion_commands_use_high_speed_values() -> None:
    forward_linear, forward_angular = MOTION_COMMANDS['move_forward_slow']
    backward_linear, _ = MOTION_COMMANDS['move_backward_slow']
    turn_left_linear, turn_left_angular = MOTION_COMMANDS['turn_left_slow']

    assert abs(forward_linear) >= 0.8
    assert abs(backward_linear) >= 0.8
    assert abs(turn_left_angular) >= 0.8
    assert forward_angular == 0.0
    assert turn_left_linear == 0.0
