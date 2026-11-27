from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

from tools.monitor_runtime import LogFollower, sample_processes


class RuntimeMemoryMonitorTests(unittest.TestCase):
    def test_log_follower_reads_only_new_complete_lines_and_follows_rotation(self):
        with TemporaryDirectory() as directory:
            folder = Path(directory)
            first = folder / "2026-11-27_12-00-01.log"
            first.write_text("history\n", encoding="utf-8")
            follower = LogFollower(folder)
            self.assertEqual([], list(follower.read_lines()))
            with first.open("ab") as log:
                log.write("实时".encode("utf-8"))
            self.assertEqual([], list(follower.read_lines()))
            with first.open("ab") as log:
                log.write(b"\r\n")
            self.assertEqual(["实时"], list(follower.read_lines()))
            with first.open("ab") as log:
                log.write(b"old file end\n")
            second = folder / "2026-11-27_12-30-01.log"
            second.write_text("new file start\n", encoding="utf-8-sig")
            self.assertEqual(["old file end", "new file start"], list(follower.read_lines()))
            self.assertEqual([], list(follower.read_lines()))

    def test_log_backlog_is_read_in_bounded_batches(self):
        with TemporaryDirectory() as directory:
            folder = Path(directory)
            log = folder / "2026-11-27_12-00-01.log"
            log.write_text("", encoding="utf-8")
            follower = LogFollower(folder, max_lines=2)
            with log.open("a", encoding="utf-8") as stream:
                stream.write("one\ntwo\nthree\n")
            self.assertEqual(["one", "two"], list(follower.read_lines()))
            self.assertEqual(["three"], list(follower.read_lines()))

    def test_process_sample_keeps_parent_and_child_memory_separate(self):
        class Process:
            def __init__(self, pid, private_bytes, children=()):
                self.pid = pid
                self._private = private_bytes
                self._children = children

            def children(self, recursive=False):
                self.recursive = recursive
                return self._children

            def name(self):
                return "python.exe"

            def create_time(self):
                return float(self.pid)

            def memory_info(self):
                return SimpleNamespace(rss=self._private + 10, private=self._private)

        root = Process(100, 500, [Process(101, 200)])
        readings = list(sample_processes(root, ()))
        self.assertEqual([(100, 500, 510), (101, 200, 210)], [(item.pid, item.private_bytes, item.rss_bytes) for item in readings])
        self.assertTrue(root.recursive)
