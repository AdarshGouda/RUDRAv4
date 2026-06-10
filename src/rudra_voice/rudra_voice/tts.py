"""Text-to-speech helpers for RUDRA voice replies."""

from __future__ import annotations

from pathlib import Path
import subprocess
import threading
from typing import Callable
import tempfile


class TtsBackend:
    """Common asynchronous TTS interface."""

    def speak(self, text: str) -> None:
        raise NotImplementedError

    def speak_async(self, text: str) -> None:
        """Speak on a daemon thread so ROS callbacks stay responsive."""
        thread = threading.Thread(target=self.speak, args=(text,), daemon=True)
        thread.start()


class EspeakTts(TtsBackend):
    """Speak short robot replies through espeak-ng."""

    def __init__(
        self,
        output_device_name: str | None = None,
        speaking_callback: Callable[[bool], None] | None = None,
        voice: str = 'en-us+f3',
        rate_wpm: int = 155,
        pitch: int = 45,
        amplitude: int = 180,
    ) -> None:
        self.output_device_name = output_device_name
        self.speaking_callback = speaking_callback
        self.voice = voice
        self.rate_wpm = rate_wpm
        self.pitch = pitch
        self.amplitude = amplitude
        self._lock = threading.Lock()

    def speak(self, text: str) -> None:
        """Speak synchronously and mark listening suppression while active."""
        stripped = text.strip()
        if not stripped:
            return
        with self._lock:
            if self.speaking_callback is not None:
                self.speaking_callback(True)
            try:
                subprocess.run(
                    [
                        'espeak-ng',
                        '-v',
                        self.voice,
                        '-s',
                        str(self.rate_wpm),
                        '-p',
                        str(self.pitch),
                        '-a',
                        str(self.amplitude),
                        stripped,
                    ],
                    check=False,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            finally:
                if self.speaking_callback is not None:
                    self.speaking_callback(False)


class PiperTts(TtsBackend):
    """Speak through Piper when a local voice model is installed."""

    def __init__(
        self,
        model_path: str,
        executable: str = 'piper',
        speaking_callback: Callable[[bool], None] | None = None,
        fallback: TtsBackend | None = None,
    ) -> None:
        self.model_path = str(Path(model_path).expanduser())
        self.executable = str(Path(executable).expanduser()) if executable else 'piper'
        self.speaking_callback = speaking_callback
        self.fallback = fallback
        self._lock = threading.Lock()

    def speak(self, text: str) -> None:
        stripped = text.strip()
        if not stripped:
            return
        if not Path(self.model_path).exists():
            if self.fallback is not None:
                self.fallback.speak(stripped)
            return
        with self._lock:
            if self.speaking_callback is not None:
                self.speaking_callback(True)
            try:
                with tempfile.NamedTemporaryFile(suffix='.wav') as wav_file:
                    piper = subprocess.run(
                        [
                            self.executable,
                            '--model',
                            self.model_path,
                            '--output_file',
                            wav_file.name,
                        ],
                        input=stripped.encode('utf-8'),
                        check=False,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                    if piper.returncode == 0:
                        subprocess.run(
                            ['aplay', wav_file.name],
                            check=False,
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                        )
                    elif self.fallback is not None:
                        self.fallback.speak(stripped)
            finally:
                if self.speaking_callback is not None:
                    self.speaking_callback(False)


def make_tts_backend(
    backend: str,
    speaking_callback: Callable[[bool], None] | None = None,
    output_device_name: str | None = None,
    espeak_voice: str = 'en-us+f3',
    espeak_rate_wpm: int = 155,
    espeak_pitch: int = 45,
    espeak_amplitude: int = 180,
    piper_model_path: str = '',
    piper_executable: str = 'piper',
) -> TtsBackend:
    """Create the configured TTS backend with eSpeak as the safe fallback."""
    fallback = EspeakTts(
        output_device_name=output_device_name,
        speaking_callback=speaking_callback,
        voice=espeak_voice,
        rate_wpm=espeak_rate_wpm,
        pitch=espeak_pitch,
        amplitude=espeak_amplitude,
    )
    if backend.strip().lower() == 'piper':
        return PiperTts(
            model_path=piper_model_path,
            executable=piper_executable,
            speaking_callback=speaking_callback,
            fallback=fallback,
        )
    return fallback
