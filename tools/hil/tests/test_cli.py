from pathlib import Path

import serial

from floodnet_hil import cli


def test_analyze_prints_the_hil_line(tmp_path: Path, capsys):
    run_dir = tmp_path / "bench"
    run_dir.mkdir()
    (run_dir / "node.log").write_text("1 TX,66,1,0,1,ok\n")
    (run_dir / "gateway.log").write_text(
        "2 REC,66,0,0,0,0,0,0,0,0,0,0,0,-90,1,0,1,0,0\n"
    )
    assert cli.main(["analyze", str(run_dir)]) == 0
    assert capsys.readouterr().out == "HIL,bench,1,0,1,1.000000,0,0,0,0,-90,-90,-90,0\n"


def test_capture_reports_a_port_it_cannot_open(tmp_path: Path, monkeypatch, capsys):
    def refuse(port, *args, **kwargs):
        raise serial.SerialException(f"could not open port {port}")

    monkeypatch.setattr(serial, "Serial", refuse)
    code = cli.main(
        ["capture", "--node", "/dev/nope", "--gateway", "/dev/nada", "--label", "x",
         "--seconds", "1", "--out", str(tmp_path)]
    )
    assert code == 2
    assert "/dev/nope" in capsys.readouterr().err
