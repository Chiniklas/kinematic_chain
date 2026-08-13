"""Minimal dependency-free TensorBoard scalar event writer.

TensorBoard event files are TFRecord streams containing Event protobuf messages. This
module writes the small subset required for scalar optimization telemetry so logging
does not depend on TensorFlow, PyTorch, or tensorboard at training time.
"""

from __future__ import annotations

import os
import socket
import struct
import time
from pathlib import Path
from typing import Mapping


def _varint(value: int) -> bytes:
    if value < 0:
        raise ValueError("protobuf varints in this writer must be non-negative")
    encoded = bytearray()
    while value >= 0x80:
        encoded.append((value & 0x7F) | 0x80)
        value >>= 7
    encoded.append(value)
    return bytes(encoded)


def _length_delimited(field: int, payload: bytes) -> bytes:
    return _varint((field << 3) | 2) + _varint(len(payload)) + payload


def _crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82F63B78 if crc & 1 else 0)
    return crc ^ 0xFFFFFFFF


def _masked_crc32c(data: bytes) -> bytes:
    crc = _crc32c(data)
    masked = (((crc >> 15) | (crc << 17)) + 0xA282EAD8) & 0xFFFFFFFF
    return struct.pack("<I", masked)


def _summary_value(tag: str, value: float) -> bytes:
    return (
        _length_delimited(1, tag.encode("utf-8"))
        + _varint((2 << 3) | 5)
        + struct.pack("<f", float(value))
    )


def _event(step: int, scalars: Mapping[str, float] | None = None) -> bytes:
    message = _varint((1 << 3) | 1) + struct.pack("<d", time.time())
    message += _varint(2 << 3) + _varint(step)
    if scalars is None:
        message += _length_delimited(3, b"brain.Event:2")
    else:
        summary = b"".join(
            _length_delimited(1, _summary_value(tag, value))
            for tag, value in scalars.items()
        )
        message += _length_delimited(5, summary)
    return message


class TensorBoardLogger:
    """Write multiple scalar summaries per optimization step."""

    def __init__(self, log_dir: Path):
        log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = int(time.time())
        filename = (
            f"events.out.tfevents.{timestamp}.{socket.gethostname()}.{os.getpid()}"
        )
        self.path = log_dir / filename
        self._stream = self.path.open("wb")
        self._write_record(_event(0))

    def _write_record(self, payload: bytes) -> None:
        length = struct.pack("<Q", len(payload))
        self._stream.write(length)
        self._stream.write(_masked_crc32c(length))
        self._stream.write(payload)
        self._stream.write(_masked_crc32c(payload))

    def add_scalars(self, scalars: Mapping[str, float], step: int) -> None:
        self._write_record(_event(step, scalars))

    def flush(self) -> None:
        self._stream.flush()

    def close(self) -> None:
        if not self._stream.closed:
            self.flush()
            self._stream.close()

    def __enter__(self) -> TensorBoardLogger:
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()
