"""Parse FloodNet node and gateway serial logs.

Each log line is "<t_ms> <line>": milliseconds since the capture started (the
simulation clock for simulated transcripts), then the line as the board
printed it.
"""

from __future__ import annotations

from dataclasses import dataclass, field

TX_FIELDS = 6
REC_FIELDS = 19


@dataclass(frozen=True)
class TxLine:
    t_ms: int
    node_id: int
    boot_count: int
    seq: int
    millis: int
    completed: bool

    @property
    def key(self) -> tuple[int, int, int]:
        return (self.node_id, self.boot_count, self.seq)


@dataclass(frozen=True)
class RecLine:
    t_ms: int
    node_id: int
    seq: int
    boot_count: int
    rssi: int

    @property
    def key(self) -> tuple[int, int, int]:
        return (self.node_id, self.boot_count, self.seq)


@dataclass
class ParsedLog:
    tx: list[TxLine] = field(default_factory=list)
    rec: list[RecLine] = field(default_factory=list)
    decode_errors: int = 0
    short_frames: int = 0
    malformed: int = 0


def _parse_line(line: str, out: ParsedLog) -> None:
    stamp, sep, payload = line.partition(" ")
    try:
        if not sep:
            raise ValueError("no timestamp")
        t_ms = int(stamp)
        fields = payload.split(",")
        kind = fields[0]
        if kind == "TX":
            if len(fields) != TX_FIELDS or fields[5] not in ("ok", "timeout"):
                raise ValueError("bad TX line")
            node_id, boot_count, seq, millis = (int(f) for f in fields[1:5])
            out.tx.append(TxLine(t_ms, node_id, boot_count, seq, millis, fields[5] == "ok"))
        elif kind == "REC":
            if len(fields) != REC_FIELDS:
                raise ValueError("bad REC line")
            values = [int(f) for f in fields[1:]]
            out.rec.append(
                RecLine(t_ms, node_id=values[0], seq=values[1], boot_count=values[15],
                        rssi=values[12])
            )
        elif kind == "ERR":
            if len(fields) >= 2 and fields[1] == "decode":
                out.decode_errors += 1
            elif len(fields) >= 2 and fields[1] == "short":
                out.short_frames += 1
            else:
                raise ValueError("unknown ERR line")
        # Anything else (a boot banner, "radio: init failed") is not data.
    except ValueError:
        out.malformed += 1


def parse_log(text: str) -> ParsedLog:
    """Parses a whole log. A final line with no newline was cut off mid-write,
    so it is counted as malformed rather than parsed."""
    out = ParsedLog()
    if not text:
        return out
    lines = text.split("\n")
    for line in lines[:-1]:
        line = line.rstrip("\r")
        if line:
            _parse_line(line, out)
    if lines[-1]:
        out.malformed += 1
    return out
