"""Turn a node log and a gateway log into one HIL summary line."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

from floodnet_hil.parse import ParsedLog, RecLine, parse_log


@dataclass(frozen=True)
class Summary:
    label: str
    tx_ok: int
    tx_timeout: int
    accepted: int
    delivery: float | None
    decode_errors: int
    short_frames: int
    unmatched_rec: int
    seq_gaps: int
    rssi_min: int | None
    rssi_median: int | None
    rssi_max: int | None
    malformed: int

    def line(self) -> str:
        def opt(value: int | None) -> str:
            return "-" if value is None else str(value)

        delivery = "-" if self.delivery is None else f"{self.delivery:.6f}"
        return (
            f"HIL,{self.label},{self.tx_ok},{self.tx_timeout},{self.accepted},{delivery},"
            f"{self.decode_errors},{self.short_frames},{self.unmatched_rec},{self.seq_gaps},"
            f"{opt(self.rssi_min)},{opt(self.rssi_median)},{opt(self.rssi_max)},{self.malformed}"
        )


def _seq_gaps(keys: set[tuple[int, int, int]]) -> int:
    """Sequence numbers missing between the lowest and highest accepted,
    per (node_id, boot_count)."""
    seqs: dict[tuple[int, int], set[int]] = defaultdict(set)
    for node_id, boot_count, seq in keys:
        seqs[(node_id, boot_count)].add(seq)
    return sum(max(s) - min(s) + 1 - len(s) for s in seqs.values())


def analyze(node: ParsedLog, gateway: ParsedLog, label: str) -> Summary:
    ok_keys = {t.key for t in node.tx if t.completed}
    tx_ok = sum(1 for t in node.tx if t.completed)
    tx_timeout = sum(1 for t in node.tx if not t.completed)

    first_rec: dict[tuple[int, int, int], RecLine] = {}
    for r in gateway.rec:
        first_rec.setdefault(r.key, r)
    matched = [r for key, r in first_rec.items() if key in ok_keys]
    rssi = sorted(r.rssi for r in matched)

    return Summary(
        label=label,
        tx_ok=tx_ok,
        tx_timeout=tx_timeout,
        accepted=len(matched),
        delivery=len(matched) / tx_ok if tx_ok else None,
        decode_errors=gateway.decode_errors,
        short_frames=gateway.short_frames,
        unmatched_rec=len(first_rec) - len(matched),
        seq_gaps=_seq_gaps(set(first_rec)),
        rssi_min=rssi[0] if rssi else None,
        rssi_median=rssi[(len(rssi) - 1) // 2] if rssi else None,
        rssi_max=rssi[-1] if rssi else None,
        malformed=node.malformed + gateway.malformed,
    )


def analyze_dir(run_dir: Path) -> Summary:
    """Analyzes run_dir/node.log and run_dir/gateway.log; the label is the
    directory's name."""
    node = parse_log((run_dir / "node.log").read_text(encoding="utf-8", errors="replace"))
    gateway = parse_log((run_dir / "gateway.log").read_text(encoding="utf-8", errors="replace"))
    return analyze(node, gateway, run_dir.name)
