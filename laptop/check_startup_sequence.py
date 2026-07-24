#!/usr/bin/env python3
"""Check ESP32 mission logs for CMD_START -> ToF pass -> OFFBOARD -> ARM sequence.

Usage examples:
  python laptop/check_startup_sequence.py --log-file esp32.log
  idf.py monitor | python laptop/check_startup_sequence.py
    python laptop/check_startup_sequence.py --serial-port COM7 --baud 115200
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from dataclasses import dataclass
from typing import Iterable


@dataclass
class Step:
    name: str
    pattern: re.Pattern[str]
    matched_line: str | None = None


def build_steps() -> list[Step]:
    # Accept either '-' or unicode dashes and allow variable whitespace.
    return [
        Step(
            name="CMD_START received",
            pattern=re.compile(r"\bCMD_START\s+received\b", re.IGNORECASE),
        ),
        Step(
            name="ToF sensors passed",
            pattern=re.compile(
                r"\bAll\s+\d+\s+ToF\s+sensors\s+OK\b.*\bproceeding\s+to\s+arm\b",
                re.IGNORECASE,
            ),
        ),
        Step(
            name="OFFBOARD confirmed",
            pattern=re.compile(r"\bOFFBOARD\s+mode\s+confirmed\b", re.IGNORECASE),
        ),
        Step(
            name="Arming confirmed",
            pattern=re.compile(r"\bArmed\s+confirmed\b", re.IGNORECASE),
        ),
    ]


TOF_FAIL_RE = re.compile(r"\bTOF\s+CHECK\s+FAILED\b", re.IGNORECASE)


def iter_lines(
    log_file: str | None,
    serial_port: str | None,
    baud: int,
    timeout_s: float,
) -> Iterable[str]:
    if log_file:
        with open(log_file, "r", encoding="utf-8", errors="replace") as fh:
            for line in fh:
                yield line.rstrip("\n")
        return

    if serial_port:
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise SystemExit(
                "pyserial is required for --serial-port mode. Install with: python -m pip install pyserial"
            ) from exc

        deadline = time.monotonic() + timeout_s if timeout_s > 0 else None
        try:
            with serial.Serial(serial_port, baudrate=baud, timeout=0.2) as ser:
                while True:
                    if deadline is not None and time.monotonic() > deadline:
                        break

                    raw = ser.readline()
                    if not raw:
                        continue

                    line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                    if line:
                        yield line
        except serial.SerialException as exc:  # type: ignore[attr-defined]
            raise SystemExit(
                "Could not open serial port "
                f"{serial_port}. It may be busy (for example, idf.py monitor is using it) "
                "or the port name is wrong.\n"
                "Close other serial monitors, then retry.\n"
                "Tip: list ports with: Get-CimInstance Win32_SerialPort | "
                "Select-Object DeviceID, Name\n"
                f"Original error: {exc}"
            ) from exc
        return

    for line in sys.stdin:
        yield line.rstrip("\n")


def check_sequence(lines: Iterable[str], verbose: bool = False) -> int:
    steps = build_steps()
    step_idx = 0
    tof_fail_lines: list[str] = []

    for raw_line in lines:
        line = raw_line.strip()

        if TOF_FAIL_RE.search(line):
            tof_fail_lines.append(line)

        if step_idx >= len(steps):
            continue

        step = steps[step_idx]
        if step.pattern.search(line):
            step.matched_line = line
            step_idx += 1
            if verbose:
                print(f"MATCH {step_idx}/{len(steps)} | {step.name} | {line}")

            if step_idx >= len(steps):
                break

    print("=== Startup Sequence Check ===")

    for i, step in enumerate(steps, start=1):
        status = "PASS" if step.matched_line else "MISSING"
        print(f"{i}. {step.name:<22} : {status}")
        if verbose and step.matched_line:
            print(f"   line: {step.matched_line}")

    if tof_fail_lines:
        print(f"\nDetected {len(tof_fail_lines)} ToF failure line(s):")
        for line in tof_fail_lines[-3:]:
            print(f"  - {line}")

    all_passed = all(step.matched_line for step in steps)

    if all_passed and not tof_fail_lines:
        print("\nRESULT: PASS - CMD_START -> ToF -> OFFBOARD -> ARM sequence confirmed.")
        return 0

    if all_passed and tof_fail_lines:
        print("\nRESULT: WARN - Sequence confirmed, but ToF failure lines were also seen.")
        return 2

    missing = [step.name for step in steps if not step.matched_line]
    print("\nRESULT: FAIL - Missing step(s): " + ", ".join(missing))
    return 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check ESP32 log for CMD_START, ToF pass, OFFBOARD confirmed, and arming confirmed.",
    )
    parser.add_argument(
        "--log-file",
        help="Path to an ESP32 log file. If omitted, input is read from stdin.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print matched lines for each step.",
    )
    parser.add_argument(
        "--serial-port",
        help="Read live ESP32 logs from a serial port (for example COM7).",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate for --serial-port mode (default: 115200).",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=45.0,
        help="Seconds to wait in --serial-port mode (0 means no timeout, default: 45).",
    )
    args = parser.parse_args()

    if args.log_file and args.serial_port:
        parser.error("Use only one input source: either --log-file or --serial-port.")

    return args


def main() -> int:
    args = parse_args()
    return check_sequence(
        iter_lines(args.log_file, args.serial_port, args.baud, args.timeout_s),
        verbose=args.verbose,
    )


if __name__ == "__main__":
    raise SystemExit(main())
