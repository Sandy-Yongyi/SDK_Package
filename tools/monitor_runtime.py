"""Follow runtime logs and record memory for an existing application process tree."""

import argparse
import csv
from dataclasses import asdict, dataclass, fields
from datetime import datetime
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
MAX_LINE_BYTES = 64 * 1024
MIB = 1024 ** 2


class LogFollower:
    """Read new, complete lines from the current timestamp-named log."""

    def __init__(self, folder: Path, max_lines: int = 200):
        self.folder = folder
        self.max_lines = max_lines
        self.path = self._newest()
        self.offset = self.path.stat().st_size if self.path is not None else 0

    def _newest(self) -> Path | None:
        return max(self.folder.glob("????-??-??_??-??-??.log"), default=None)

    def read_lines(self):
        remaining = self.max_lines
        while remaining:
            if self.path is not None:
                lines = self._read_current(remaining)
                yield from lines
                remaining -= len(lines)
            next_log = self._newest()
            if remaining == 0 or next_log is None or (self.path is not None and next_log <= self.path):
                return
            self.path, self.offset = next_log, 0

    def _read_current(self, limit: int) -> list[str]:
        lines = []
        try:
            with self.path.open("rb") as stream:
                if stream.seek(0, 2) < self.offset:
                    self.offset = 0
                stream.seek(self.offset)
                for _ in range(limit):
                    chunk = stream.readline(MAX_LINE_BYTES)
                    if not chunk.endswith(b"\n") and len(chunk) < MAX_LINE_BYTES:
                        break
                    encoding = "utf-8-sig" if self.offset == 0 else "utf-8"
                    self.offset = stream.tell()
                    lines.append(chunk.decode(encoding, errors="replace").rstrip("\r\n"))
        except FileNotFoundError:
            pass
        return lines


@dataclass(frozen=True)
class ProcessReading:
    pid: int
    process_name: str
    created_at: float
    rss_bytes: int
    private_bytes: int | None


def sample_processes(root, unavailable):
    for process in (root, *root.children(recursive=True)):
        try:
            memory = process.memory_info()
            yield ProcessReading(process.pid, process.name(), process.create_time(), memory.rss, getattr(memory, "private", None))
        except unavailable:
            continue


def positive_number(value: str) -> float:
    number = float(value)
    if not 0 < number < float("inf"):
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return number


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pid", type=int, required=True, help="existing application main PID from its log")
    parser.add_argument("--control-type", type=int, choices=(1, 2), required=True, help="label for the mode already in use")
    parser.add_argument("--duration-minutes", type=positive_number, default=60)
    parser.add_argument("--interval-seconds", type=positive_number, default=5)
    parser.add_argument("--log-directory", type=Path, default=ROOT / "data" / "logs")
    parser.add_argument("--csv", type=Path, dest="csv_path", help="new output file; existing files are preserved")
    parser.add_argument("--no-tail", action="store_true", help="record memory without showing log lines")
    args = parser.parse_args(argv)

    try:
        import psutil
    except ImportError:
        parser.exit(2, "The monitor requires psutil. Install it with python -m pip install psutil.\n")
    unavailable = (psutil.NoSuchProcess, psutil.AccessDenied)
    try:
        root = psutil.Process(args.pid)
    except unavailable:
        parser.exit(2, f"Unable to access the application process {args.pid}.\n")

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    csv_path = args.csv_path or ROOT / "output" / "runtime-memory" / f"mode-{args.control_type}-{args.pid}-{stamp}.csv"
    try:
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        follower = None if args.no_tail else LogFollower(args.log_directory)
        with csv_path.open("x", newline="", encoding="utf-8") as output:
            print(f"Memory CSV: {csv_path}", flush=True)
            return monitor(root, args, output, follower, unavailable)
    except KeyboardInterrupt:
        print("\nMonitoring stopped. Samples already recorded are kept.")
        return 130
    except (OSError, *unavailable) as error:
        print(f"Monitoring ended: {error}", file=sys.stderr)
        return 1


def monitor(root, args, output, follower, unavailable) -> int:
    columns = ["timestamp", "control_type", *(field.name for field in fields(ProcessReading))]
    writer = csv.DictWriter(output, fieldnames=columns)
    writer.writeheader()
    next_sample = time.monotonic()
    deadline = next_sample + args.duration_minutes * 60
    while (now := time.monotonic()) < deadline:
        if now >= next_sample:
            readings = list(sample_processes(root, unavailable)) if root.is_running() else []
            if not readings or readings[0].pid != root.pid:
                print(f"Application process {root.pid} exited or became inaccessible.", file=sys.stderr)
                return 1
            timestamp = datetime.now().astimezone().isoformat(timespec="seconds")
            writer.writerows({"timestamp": timestamp, "control_type": args.control_type, **asdict(item)} for item in readings)
            output.flush()
            rss = sum(item.rss_bytes for item in readings) / MIB
            private = [item.private_bytes for item in readings if item.private_bytes is not None]
            private_summary = f", private={sum(private) / MIB:.1f} MiB" if len(private) == len(readings) else ""
            print(f"[memory] {timestamp} mode={args.control_type}, processes={len(readings)}, RSS={rss:.1f} MiB{private_summary}", flush=True)
            next_sample = now + args.interval_seconds
        if follower is not None:
            for line in follower.read_lines():
                print(line)
        time.sleep(min(0.25, max(0, deadline - time.monotonic()), max(0, next_sample - time.monotonic())))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
