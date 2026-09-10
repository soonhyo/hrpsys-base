#!/usr/bin/env python3
"""Linux PTY regressions: no ROS, robot or hardware serial access."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def cases(names):
    return [[name] for name in names.split()]


SUITES = {
    "receive": ([], [["fragment", str(n)] for n in range(27)] + cases(
        "echo missing-return missing-echo id header count length address flags checksum"), []),
    "transfers": (["write", "tcflush"], cases(
        "concurrent get-off short-write zero-write write-error write-timeout "
        "partial-error flush-error rx-error-off"), cases("partial-off-limit")),
    "framing": ([], [["resync", str(n)] for n in range(27)] + cases(
        "coalesced flags late-other trickle noise-limit buffered maximum"), cases("same-id-limit")),
    "initialize": (["tcgetattr", "cfsetospeed", "cfsetispeed", "tcsetattr"],
                   cases("0 1 2 3 4 open"), []),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--header-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--suite", choices=list(SUITES), action="append")
    parser.add_argument("--sanitize", action="store_true")
    parser.add_argument("--debug-logging", action="store_true")
    parser.add_argument("--compiler", default="g++")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results, builds = [], []
    for suite in args.suite or SUITES:
        wrappers, tests, limitations = SUITES[suite]
        binary = args.output.resolve() / ("test-servo-" + suite)
        command = [args.compiler, "-std=gnu++98", "-pthread",
                   "-I" + str(args.header_dir.resolve()),
                   str(Path(__file__).with_name("test-servo-" + suite + ".cpp")),
                   "-lutil", "-o", str(binary)]
        command += ["-Wl,--wrap=" + name for name in wrappers]
        if args.debug_logging:
            command += ["-DSERVO_SERIAL_DEBUG"]
        if args.sanitize:
            command += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-no-pie"]
        with (args.output / ("compile-" + suite + ".log")).open("w") as log:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
        builds.append(command)
        for case in tests + limitations:
            path = args.output / (suite + "-" + "-".join(case) + ".log")
            with path.open("w") as log:
                try:
                    status = subprocess.run([str(binary)] + case, stdout=log,
                                            stderr=subprocess.STDOUT, timeout=3).returncode
                except subprocess.TimeoutExpired:
                    status = "timeout"
            matched = status == 0
            if case == ["partial-error"]:
                matched = matched and "2/9 bytes queued" in path.read_text()
            if suite == "framing" and case == ["coalesced"]:
                logged = "[ServoSerial] sending :" in path.read_text()
                matched = matched and logged == args.debug_logging
            results.append({"suite": suite, "case": case, "status": status,
                            "known_limit": case in limitations, "matched": matched})
    failures = [r for r in results if not r["matched"]]
    regular = [r for r in results if not r["known_limit"]]
    limits = [r for r in results if r["known_limit"]]
    summary = {
        "header_sha256": hashlib.sha256((args.header_dir / "ServoSerial.h").read_bytes()).hexdigest(),
        "sanitized": args.sanitize, "debug_logging": args.debug_logging,
        "builds": builds, "results": results,
        "regression_cases": len(regular), "known_limit_cases": len(limits),
        "unexpected": len(failures),
    }
    (args.output / "results.json").write_text(json.dumps(summary, indent=2) + "\n")
    print("%d regression cases, %d known-limit cases, %d unexpected" %
          (len(regular), len(limits), len(failures)), flush=True)
    for failure in failures:
        print(failure, flush=True)
    return bool(failures)


if __name__ == "__main__":
    raise SystemExit(main())
