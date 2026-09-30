"""Record both boards' serial output to log files, unchanged.

Only complete lines are written, each prefixed with milliseconds since the
capture started, so every log ends with a newline. A line still arriving when
the capture ends is dropped rather than written half-finished.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from pathlib import Path


class _LineSplitter:
    def __init__(self) -> None:
        self._pending = b""

    def feed(self, data: bytes) -> list[str]:
        self._pending += data
        *complete, self._pending = self._pending.split(b"\n")
        return [line.rstrip(b"\r").decode("utf-8", errors="replace") for line in complete]


def capture(
    ports: dict[str, object],
    out_dir: Path,
    seconds: float,
    clock: Callable[[], float] = time.monotonic,
) -> dict[str, int]:
    """Reads every port until `seconds` have passed and writes out_dir/<name>.log
    per port. Returns the number of lines written per port."""
    out_dir.mkdir(parents=True, exist_ok=True)
    files = {
        name: open(out_dir / f"{name}.log", "w", encoding="utf-8", newline="\n")
        for name in ports
    }
    splitters = {name: _LineSplitter() for name in ports}
    counts = {name: 0 for name in ports}
    start = clock()
    try:
        while clock() - start < seconds:
            for name, port in ports.items():
                data = port.read(port.in_waiting or 1)
                if not data:
                    continue
                t_ms = int((clock() - start) * 1000)
                for line in splitters[name].feed(data):
                    files[name].write(f"{t_ms} {line}\n")
                    counts[name] += 1
    finally:
        for f in files.values():
            f.close()
    return counts
