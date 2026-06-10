"""Deterministic intent parser and safety policy for RUDRA voice skills."""

from __future__ import annotations

from dataclasses import dataclass
import re
from typing import Any, Iterable


APPROVED_NON_MOTION_SKILLS = {
    'status',
    'run_self_test',
    'check_lidar',
    'check_teensy',
    'check_controller',
    'bringup_level_1',
    'bringup_level_2',
    'shutdown',
    'chat_only',
}

APPROVED_MOTION_SKILLS = {
    'move_forward_slow',
    'move_backward_slow',
    'turn_left_slow',
    'turn_right_slow',
    'stop',
    'emergency_stop',
}

APPROVED_SKILLS = APPROVED_NON_MOTION_SKILLS | APPROVED_MOTION_SKILLS

DISALLOWED_SKILLS = {
    'continuous_forward',
    'continuous_backward',
    'faster',
    'maximum_speed',
    'autonomous_drive',
    'follow_me',
    'go_to_location',
}

UNSAFE_PHRASE_PATTERNS = (
    r'\bforever\b',
    r'\bcontinuously\b',
    r'\bcontinuous\b',
    r'\bkeep going\b',
    r'\bdon[\' ]?t stop\b',
    r'\bdo not stop\b',
    r'\bwithout stopping\b',
    r'\bmaximum speed\b',
    r'\bmax speed\b',
    r'\bfaster\b',
    r'\bfull speed\b',
    r'\bfollow me\b',
    r'\bgo to\b',
    r'\bdrive to\b',
    r'\bnavigate to\b',
)


@dataclass(frozen=True)
class IntentResult:
    """Validated voice intent."""

    skill: str
    reply: str
    source: str
    requires_confirmation: bool = False

    @property
    def is_motion(self) -> bool:
        return self.skill in APPROVED_MOTION_SKILLS

    def to_json_dict(self) -> dict[str, Any]:
        return {
            'skill': self.skill,
            'reply': self.reply,
            'requires_confirmation': self.requires_confirmation,
            'source': self.source,
        }


COMMAND_PATTERNS: tuple[tuple[tuple[str, ...], str, str], ...] = (
    (('emergency stop',), 'emergency_stop', 'Emergency stop. Motors stay locked out.'),
    (('stop',), 'stop', 'Stopping now.'),
    (('move forward', 'go forward', 'forward'), 'move_forward_slow', 'Moving forward for one second.'),
    (('move backward', 'go backward', 'reverse', 'back'), 'move_backward_slow', 'Backing up for one second.'),
    (('turn left', 'left'), 'turn_left_slow', 'Turning left for one second.'),
    (('turn right', 'right'), 'turn_right_slow', 'Turning right for one second.'),
    (('run self test',), 'run_self_test', 'Starting a quick self check.'),
    (('check lidar',), 'check_lidar', 'Checking lidar now.'),
    (('check teensy',), 'check_teensy', 'Checking the Teensy link.'),
    (('check controller', 'check ps2'), 'check_controller', 'Checking manual controller activity.'),
    (('bring up level one', 'bringup level one', 'bring up level 1'), 'bringup_level_1', 'Bringup level one requested.'),
    (('bring up level two', 'bringup level two', 'bring up level 2'), 'bringup_level_2', 'Bringup level two requested.'),
    (('shut down', 'shutdown'), 'shutdown', 'Shutdown requested. Please confirm from the operator terminal.'),
    (('status',), 'status', 'I am online.'),
)

DEFAULT_WAKE_ALIASES = (
    'hey rudra',
    'rudra',
    'hey robot',
    'hello robot',
    'robot',
    # Common Vosk small-model misrecognitions seen on the P610.
    'hey deidre',
    'hey redra',
    'he redraw',
    'here to draw',
    'read dre',
    'he bought',
)

UNSUPPORTED_VOSK_GRAMMAR_TOKENS = {'1', '2', 'bringup', 'ps2', 'redra'}


def normalize_text(text: str) -> str:
    """Normalize speech text for wake phrase and command matching."""
    lowered = text.lower().strip()
    lowered = re.sub(r'[^a-z0-9\s]', ' ', lowered)
    return re.sub(r'\s+', ' ', lowered).strip()


def strip_wake_phrase(
    text: str,
    wake_phrase: str,
    wake_aliases: Iterable[str] = (),
) -> tuple[bool, str]:
    """Return whether the wake phrase is present and the command after it."""
    normalized = normalize_text(text)
    normalized_wakes = [
        normalize_text(candidate)
        for candidate in (wake_phrase, *wake_aliases)
        if normalize_text(candidate)
    ]
    if not normalized_wakes:
        return True, normalized
    for normalized_wake in sorted(normalized_wakes, key=len, reverse=True):
        if normalized == normalized_wake:
            return True, ''
        if normalized.startswith(normalized_wake + ' '):
            return True, normalized[len(normalized_wake) + 1 :].strip()
        wake_with_comma = normalized_wake.replace(' ', r'\s+')
        match = re.search(rf'\b{wake_with_comma}\b\s*(.*)$', normalized)
        if match:
            return True, match.group(1).strip()
    return False, normalized


def is_unsafe_open_motion_request(text: str) -> bool:
    """Detect movement requests that v0.5 must not execute."""
    normalized = normalize_text(text)
    return any(re.search(pattern, normalized) for pattern in UNSAFE_PHRASE_PATTERNS)


def parse_deterministic_intent(command_text: str) -> IntentResult | None:
    """Map known Alexa-style commands without calling the LLM."""
    normalized = normalize_text(command_text)
    if not normalized:
        return None
    if is_unsafe_open_motion_request(normalized):
        return IntentResult(
            skill='chat_only',
            reply='I can only do short, slow motion pulses right now.',
            source='deterministic',
        )
    for phrases, skill, reply in COMMAND_PATTERNS:
        for phrase in phrases:
            pattern = rf'(^|\b){re.escape(phrase)}(\b|$)'
            if re.search(pattern, normalized):
                return IntentResult(skill=skill, reply=reply, source='deterministic')
    return None


def deterministic_command_phrases() -> list[str]:
    """Return command phrases useful for constrained speech recognition."""
    phrases: list[str] = []
    for command_phrases, _, _ in COMMAND_PATTERNS:
        phrases.extend(command_phrases)
    return sorted(set(phrases))


def build_vosk_grammar_phrases(
    wake_phrase: str,
    wake_aliases: Iterable[str],
) -> list[str]:
    """Build a small phrase list that biases Vosk toward RUDRA commands."""
    wakes = [
        normalize_text(candidate)
        for candidate in (wake_phrase, *wake_aliases)
        if normalize_text(candidate)
    ]
    commands = deterministic_command_phrases()
    grammar = set(wakes)
    grammar.update(commands)
    for wake in wakes:
        for command in commands:
            grammar.add(f'{wake} {command}')
    grammar.add('[unk]')
    return sorted(
        phrase
        for phrase in grammar
        if not any(
            token in UNSUPPORTED_VOSK_GRAMMAR_TOKENS
            for token in phrase.split()
        )
    )


def validate_intent_payload(payload: dict[str, Any], source: str) -> IntentResult:
    """Validate an LLM or external intent payload before acting on it."""
    skill = str(payload.get('skill', '')).strip()
    reply = str(payload.get('reply', '')).strip()
    requires_confirmation = bool(payload.get('requires_confirmation', False))

    if skill in DISALLOWED_SKILLS or skill not in APPROVED_SKILLS:
        return IntentResult(
            skill='chat_only',
            reply='That request is outside my safe command set.',
            source=source,
        )
    if skill in APPROVED_MOTION_SKILLS and requires_confirmation:
        return IntentResult(
            skill='chat_only',
            reply='I cannot use voice confirmations for motion yet.',
            source=source,
        )
    if not reply:
        reply = default_reply_for_skill(skill)
    return IntentResult(
        skill=skill,
        reply=reply,
        source=source,
        requires_confirmation=requires_confirmation,
    )


def default_reply_for_skill(skill: str) -> str:
    """Return a concise robot speech reply for an approved skill."""
    replies = {
        'move_forward_slow': 'Moving forward for one second.',
        'move_backward_slow': 'Backing up for one second.',
        'turn_left_slow': 'Turning left for one second.',
        'turn_right_slow': 'Turning right for one second.',
        'stop': 'Stopping now.',
        'emergency_stop': 'Emergency stop. Motors stay locked out.',
        'status': 'I am online.',
        'run_self_test': 'Starting a quick self check.',
        'check_lidar': 'Checking lidar now.',
        'check_teensy': 'Checking the Teensy link.',
        'check_controller': 'Checking manual controller activity.',
        'bringup_level_1': 'Bringup level one requested.',
        'bringup_level_2': 'Bringup level two requested.',
        'shutdown': 'Shutdown requested. Please confirm from the operator terminal.',
        'chat_only': 'I can help with safe robot commands.',
    }
    return replies.get(skill, 'Command received.')
