"""Record both boards' serial output to log files, unchanged.

Only complete lines are written, each prefixed with milliseconds since the
capture started, so every log ends with a newline. A line still arriving when
the capture ends is dropped rather than written half-finished.

A Teensy that resets (a watchdog reset included) drops off USB and comes back.
When a port fails, its partial line is discarded, and the port is reopened
until it answers or the capture ends.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from pathlib import Path

REOPEN_INTERVAL_S = 0.5


class _LineSplitter:
    def __init__(self) -> None:
        self._pending = b""

    def feed(self, data: bytes) -> list[str]:
        self._pending += data
        *complete, self._pending = self._pending.split(b"\n")
        return [line.rstrip(b"\r").decode("utf-8", errors="replace") for line in complete]

    def discard_partial(self) -> None:
        self._pending = b""


def capture(
    ports: dict[str, object],
    out_dir: Path,
    seconds: float,
    clock: Callable[[], float] = time.monotonic,
    reopen: dict[str, Callable[[], object]] | None = None,
) -> dict[str, int]:
    """Reads every port until `seconds` have passed and writes out_dir/<name>.log
    per port. `reopen` maps a port name to a callable that opens it again after
    it fails; without one, a failed port stays closed. Returns, per port, the
    lines written (`<name>`) and the times it was reopened (`<name>_reconnects`)."""
    out_dir.mkdir(parents=True, exist_ok=True)
    live: dict[str, object | None] = dict(ports)
    files = {
        name: open(out_dir / f"{name}.log", "w", encoding="utf-8", newline="\n")
        for name in ports
    }
    splitters = {name: _LineSplitter() for name in ports}
    counts = {name: 0 for name in ports}
    reconnects = {name: 0 for name in ports}
    next_try = {name: 0.0 for name in ports}
    start = clock()
    try:
        while clock() - start < seconds:
            for name in ports:
                port = live[name]
                if port is None:
                    if reopen is None or name not in reopen or clock() < next_try[name]:
                        continue
                    try:
                        live[name] = reopen[name]()
                        reconnects[name] += 1
                    except OSError:
                        next_try[name] = clock() + REOPEN_INTERVAL_S
                    continue
                try:
                    data = port.read(port.in_waiting or 1)
                except OSError:
                    # pyserial's SerialException is an OSError. The device went
                    # away mid-line, so that line can never be completed.
                    live[name] = None
                    splitters[name].discard_partial()
                    next_try[name] = clock() + REOPEN_INTERVAL_S
                    continue
                if not data:
                    continue
                t_ms = int((clock() - start) * 1000)
                for line in splitters[name].feed(data):
                    files[name].write(f"{t_ms} {line}\n")
                    counts[name] += 1
    finally:
        for f in files.values():
            f.close()
    result = dict(counts)
    result.update({f"{name}_reconnects": n for name, n in reconnects.items()})
    return result
