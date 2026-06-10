"""Audio-device discovery helpers for the Plantronics/Poly Calisto P610."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable


@dataclass(frozen=True)
class AudioDeviceSelection:
    """Selected input/output device indexes for sounddevice."""

    input_device: int | None
    output_device: int | None
    input_name: str
    output_name: str
    found_preferred: bool


def find_preferred_audio_devices(
    keywords: Iterable[str],
    device_query: Any | None = None,
) -> AudioDeviceSelection:
    """Find preferred USB speakerphone devices, falling back to defaults."""
    sounddevice = device_query
    if sounddevice is None:
        import sounddevice  # type: ignore[import-not-found]

    keyword_list = [keyword.lower() for keyword in keywords if keyword]
    devices = list(sounddevice.query_devices())
    input_device = None
    output_device = None
    input_name = 'default input'
    output_name = 'default output'

    for index, device in enumerate(devices):
        name = str(device.get('name', ''))
        lowered = name.lower()
        if not any(keyword in lowered for keyword in keyword_list):
            continue
        if input_device is None and int(device.get('max_input_channels', 0)) > 0:
            input_device = index
            input_name = name
        if output_device is None and int(device.get('max_output_channels', 0)) > 0:
            output_device = index
            output_name = name
        if input_device is not None and output_device is not None:
            break

    return AudioDeviceSelection(
        input_device=input_device,
        output_device=output_device,
        input_name=input_name,
        output_name=output_name,
        found_preferred=input_device is not None or output_device is not None,
    )
