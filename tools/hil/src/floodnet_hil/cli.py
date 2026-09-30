"""floodnet-hil: capture both boards' serial logs, then analyze them."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from floodnet_hil.analyze import analyze_dir
from floodnet_hil.capture import capture


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="floodnet-hil")
    sub = parser.add_subparsers(dest="command", required=True)

    cap = sub.add_parser("capture", help="record node and gateway serial output")
    cap.add_argument("--node", required=True, help="the node's serial port")
    cap.add_argument("--gateway", required=True, help="the gateway's serial port")
    cap.add_argument("--label", required=True, help="scenario name, e.g. attenuator_20db")
    cap.add_argument("--seconds", required=True, type=float)
    cap.add_argument("--baud", type=int, default=115200)
    cap.add_argument("--out", type=Path, default=Path("hil-captures"))

    ana = sub.add_parser("analyze", help="print the HIL summary line for a capture")
    ana.add_argument("run_dir", type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.command == "analyze":
        print(analyze_dir(args.run_dir).line())
        return 0

    import serial

    ports = {}
    for name, device in (("node", args.node), ("gateway", args.gateway)):
        try:
            ports[name] = serial.Serial(device, args.baud, timeout=0.05)
        except serial.SerialException as err:
            print(f"floodnet-hil: cannot open {device}: {err}", file=sys.stderr)
            return 2
    run_dir = args.out / args.label
    counts = capture(ports, run_dir, args.seconds)
    print(
        f"captured {counts['node']} node lines and {counts['gateway']} gateway lines "
        f"into {run_dir}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
