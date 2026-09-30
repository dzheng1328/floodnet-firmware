from pathlib import Path

from floodnet_hil.analyze import analyze, analyze_dir
from floodnet_hil.parse import parse_log


def rec(t, node, seq, boot, rssi=-90):
    return f"{t} REC,{node},{seq},0,0,0,0,0,0,0,0,0,0,{rssi},1,0,{boot},0,0"


def tx(t, node, boot, seq, outcome="ok"):
    return f"{t} TX,{node},{boot},{seq},{t},{outcome}"


def run(node_lines, gateway_lines, label="run"):
    node = parse_log("".join(line + "\n" for line in node_lines))
    gateway = parse_log("".join(line + "\n" for line in gateway_lines))
    return analyze(node, gateway, label)


def test_matching_and_delivery():
    s = run(
        [tx(1, 66, 1, 0), tx(2, 66, 1, 1), tx(3, 66, 1, 2)],
        [rec(1, 66, 0, 1), rec(3, 66, 2, 1)],
    )
    assert (s.tx_ok, s.accepted, s.unmatched_rec) == (3, 2, 0)
    assert s.delivery == 2 / 3


def test_matching_across_a_reboot():
    s = run(
        [tx(1, 66, 1, 0), tx(2, 66, 2, 0)],
        [rec(1, 66, 0, 1), rec(2, 66, 0, 2)],
    )
    assert (s.accepted, s.unmatched_rec, s.seq_gaps) == (2, 0, 0)


def test_duplicate_rec_counts_once():
    s = run([tx(1, 66, 1, 0)], [rec(1, 66, 0, 1), rec(2, 66, 0, 1)])
    assert (s.accepted, s.unmatched_rec) == (1, 0)


def test_timeout_never_matches_and_is_counted_apart():
    s = run([tx(1, 66, 1, 0, "timeout"), tx(2, 66, 1, 1)], [rec(1, 66, 0, 1), rec(2, 66, 1, 1)])
    assert (s.tx_ok, s.tx_timeout, s.accepted, s.unmatched_rec) == (1, 1, 1, 1)


def test_unmatched_rec():
    s = run([tx(1, 66, 1, 0)], [rec(1, 66, 0, 1), rec(2, 66, 9, 1)])
    assert (s.accepted, s.unmatched_rec) == (1, 1)


def test_seq_gaps_per_boot():
    s = run(
        [],
        [rec(1, 66, 0, 1), rec(2, 66, 3, 1), rec(3, 66, 10, 2), rec(4, 66, 11, 2)],
    )
    assert s.seq_gaps == 2


def test_seq_gaps_ignore_rec_keys_no_tx_matches():
    # A REC whose key corruption altered (seq 2**31) must not count ~2**31 gaps.
    s = run(
        [tx(1, 66, 1, 0), tx(2, 66, 1, 1)],
        [rec(1, 66, 0, 1), rec(2, 66, 1, 1), rec(3, 66, 2**31, 1)],
    )
    assert (s.unmatched_rec, s.seq_gaps) == (1, 0)


def test_seq_gaps_without_a_node_log_use_every_rec():
    s = run([], [rec(1, 66, 0, 1), rec(2, 66, 3, 1)])
    assert s.seq_gaps == 2


def test_rssi_with_no_accepted_frames():
    s = run([tx(1, 66, 1, 0)], [])
    assert (s.rssi_min, s.rssi_median, s.rssi_max) == (None, None, None)


def test_rssi_with_one_accepted_frame():
    s = run([tx(1, 66, 1, 0)], [rec(1, 66, 0, 1, -80)])
    assert (s.rssi_min, s.rssi_median, s.rssi_max) == (-80, -80, -80)


def test_rssi_median_of_two_is_the_lower():
    s = run([tx(1, 66, 1, 0), tx(2, 66, 1, 1)], [rec(1, 66, 0, 1, -80), rec(2, 66, 1, 1, -60)])
    assert (s.rssi_min, s.rssi_median, s.rssi_max) == (-80, -80, -60)


def test_zero_tx_ok_prints_a_dash():
    s = run([], [], label="empty")
    assert s.delivery is None
    assert s.line() == "HIL,empty,0,0,0,-,0,0,0,0,-,-,-,0"


def test_summary_line():
    s = run(
        [tx(1, 66, 1, 0), tx(2, 66, 1, 1)],
        [rec(1, 66, 0, 1, -80), "5 ERR,decode,1", "6 ERR,short,12,1", "garbage"],
        label="bench",
    )
    assert s.line() == "HIL,bench,2,0,1,0.500000,1,1,0,0,-80,-80,-80,1"


def test_analyze_dir_reads_both_logs(tmp_path: Path):
    run_dir = tmp_path / "attenuator_20db"
    run_dir.mkdir()
    (run_dir / "node.log").write_text(tx(1, 66, 1, 0) + "\n")
    (run_dir / "gateway.log").write_text(rec(1, 66, 0, 1) + "\n")
    s = analyze_dir(run_dir)
    assert s.label == "attenuator_20db"
    assert (s.tx_ok, s.accepted) == (1, 1)
