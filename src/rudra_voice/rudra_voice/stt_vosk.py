"""Vosk speech-to-text capture using sounddevice."""

from __future__ import annotations

import json
from pathlib import Path
import queue
from typing import Iterator


class VoskSpeechToText:
    """Stream microphone audio into Vosk and yield final transcripts."""

    def __init__(
        self,
        model_path: str,
        sample_rate: int,
        input_device: int | None = None,
        blocksize: int = 8000,
        grammar_phrases: list[str] | None = None,
    ) -> None:
        import sounddevice  # type: ignore[import-not-found]
        import vosk  # type: ignore[import-not-found]

        resolved_model_path = Path(model_path).expanduser()
        if not resolved_model_path.exists():
            raise FileNotFoundError(f'Vosk model not found: {resolved_model_path}')

        vosk.SetLogLevel(-1)
        self.sounddevice = sounddevice
        self.sample_rate = sample_rate
        self.input_device = input_device
        self.blocksize = blocksize
        self.audio_queue: queue.Queue[bytes] = queue.Queue()
        self.model = vosk.Model(str(resolved_model_path))
        if grammar_phrases:
            self.recognizer = vosk.KaldiRecognizer(
                self.model,
                sample_rate,
                json.dumps(grammar_phrases),
            )
        else:
            self.recognizer = vosk.KaldiRecognizer(self.model, sample_rate)

    def _audio_callback(self, indata: bytes, frames: int, time_info: object, status: object) -> None:
        if status:
            # Status is surfaced by the owning node through transcript failures.
            pass
        self.audio_queue.put(bytes(indata))

    def transcripts(self) -> Iterator[str]:
        """Yield final recognized text from the microphone stream."""
        with self.sounddevice.RawInputStream(
            samplerate=self.sample_rate,
            blocksize=self.blocksize,
            device=self.input_device,
            dtype='int16',
            channels=1,
            callback=self._audio_callback,
        ):
            while True:
                data = self.audio_queue.get()
                if self.recognizer.AcceptWaveform(data):
                    payload = json.loads(self.recognizer.Result())
                    text = str(payload.get('text', '')).strip()
                    if text:
                        yield text
