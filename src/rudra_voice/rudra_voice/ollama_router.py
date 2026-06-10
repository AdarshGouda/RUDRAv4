"""Ollama-backed natural-language intent router for RUDRA voice commands."""

from __future__ import annotations

import json
from typing import Any

import requests

from .intent_parser import IntentResult, validate_intent_payload


SYSTEM_PROMPT = """You are RUDRA, a local ROS2 rover voice assistant.
Respond only as valid JSON using this exact schema:
{
  "skill": "one approved skill",
  "reply": "short spoken robot reply",
  "requires_confirmation": false
}

Approved non-motion skills:
status, run_self_test, check_lidar, check_teensy, check_controller,
bringup_level_1, bringup_level_2, shutdown, chat_only

Approved motion skills:
move_forward_slow, move_backward_slow, turn_left_slow, turn_right_slow,
stop, emergency_stop

Rules:
- You may only select approved skills.
- You must never directly drive motors.
- You must never publish /cmd_vel.
- Physical emergency stop overrides everything.
- PS2 manual control has priority over voice.
- Voice movement is allowed only for approved slow, short-duration motion skills.
- If the user asks for normal movement, map it to one of:
  move_forward_slow, move_backward_slow, turn_left_slow, turn_right_slow,
  stop, emergency_stop.
- If the user asks for unsafe or continuous motion, choose chat_only and explain
  that v0.5 only supports short safe motion pulses.
- Keep replies short and suitable for robot speech.
"""


class OllamaRouter:
    """Route uncertain speech commands to a local Ollama model."""

    def __init__(
        self,
        url: str,
        model: str,
        timeout_sec: float = 15.0,
    ) -> None:
        self.url = url
        self.model = model
        self.timeout_sec = timeout_sec

    def route(self, command_text: str) -> tuple[IntentResult, str, str | None]:
        """Return a validated intent, raw model text, and optional error."""
        request_body: dict[str, Any] = {
            'model': self.model,
            'stream': False,
            'format': 'json',
            'messages': [
                {'role': 'system', 'content': SYSTEM_PROMPT},
                {'role': 'user', 'content': command_text},
            ],
            'options': {'temperature': 0.0},
        }
        try:
            response = requests.post(
                self.url,
                json=request_body,
                timeout=self.timeout_sec,
            )
            response.raise_for_status()
        except requests.RequestException as exc:
            intent = IntentResult(
                skill='chat_only',
                reply='The local language model is offline. I can still use direct commands.',
                source='ollama_error',
            )
            return intent, '', str(exc)

        raw_content = self._extract_content(response.json())
        try:
            payload = json.loads(raw_content)
        except json.JSONDecodeError as exc:
            intent = IntentResult(
                skill='chat_only',
                reply='I could not understand the local language model response.',
                source='ollama_invalid_json',
            )
            return intent, raw_content, str(exc)

        return validate_intent_payload(payload, source='ollama'), raw_content, None

    @staticmethod
    def _extract_content(response_payload: dict[str, Any]) -> str:
        message = response_payload.get('message')
        if isinstance(message, dict):
            content = message.get('content', '')
            return str(content).strip()
        response = response_payload.get('response', '')
        return str(response).strip()
