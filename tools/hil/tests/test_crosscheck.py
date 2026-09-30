"""The host tool against the simulation's own counts, for every frozen scenario.

The transcripts are written by `pio test -e native -f test_hil_transcripts`.
A difference means the tool or the firmware hook is wrong; no count is ever
adjusted to make them agree.
"""

import json
from pathlib import Path

import pytest

from floodnet_hil.analyze import analyze_dir
from floodnet_hil.parse import parse_log

REPO = Path(__file__).resolve().parents[3]
TRANSCRIPTS = REPO / "build" / "hil-transcripts"
SCENARIOS = [
    "interrupt_p0",
    "interrupt_p1e-3",
    "interrupt_p1e-2",
    "duty_cycled_p0",
    "duty_cycled_p1e-3",
    "duty_cycled_p1e-2",
    "duty_cycled_lost_completion",
    "duty_cycled_hang",
]


def load(name):
    run_dir = TRANSCRIPTS / name
    expected_path = run_dir / "expected.json"
    if not expected_path.exists():
        pytest.fail(
            f"missing {expected_path}: run `pio test -e native -f test_hil_transcripts` first"
        )
    return run_dir, json.loads(expected_path.read_text())


@pytest.mark.parametrize("name", SCENARIOS)
def test_analyze_matches_the_simulation(name):
    run_dir, expected = load(name)
    s = analyze_dir(run_dir)
    assert s.tx_ok == expected["tx_ok"]
    assert s.tx_timeout == expected["tx_timeout"]
    assert s.unmatched_rec == expected["unmatched_rec"]
    assert s.decode_errors == expected["decode_errors"]
    assert s.accepted == expected["rec_lines"] - expected["unmatched_rec"]
    assert s.malformed == 0


def test_lost_completion_scenario_produces_a_timeout():
    _, expected = load("duty_cycled_lost_completion")
    assert expected["tx_timeout"] >= 1


def test_hang_scenario_spans_two_boots():
    run_dir, _ = load("duty_cycled_hang")
    node = parse_log((run_dir / "node.log").read_text())
    assert len({t.boot_count for t in node.tx}) == 2


def test_noisy_scenarios_produce_decode_errors():
    _, expected = load("duty_cycled_p1e-2")
    assert expected["decode_errors"] > 0
