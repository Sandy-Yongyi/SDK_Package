import multiprocessing
import queue
import shutil
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch


def _enqueue_replay_frames(frames):
    for fifo in range(16):
        frames.put({"fifo": fifo})


class TxtReplaySequenceTests(unittest.TestCase):
    def setUp(self):
        from model.utils.TomlLoader import TomlLoader
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.root = Path(self.folder.name)
        self.config = self.root / "config"
        self.config.mkdir()
        source = Path(__file__).resolve().parents[1] / "model" / "tomls"
        for name in ("ReadDataConfig.toml", "ProcessConfig.toml", "SystemConfig.toml"):
            shutil.copyfile(source / name, self.config / name)
        TomlLoader.save({"translate_data_origin": 1, "z_threshold": 10}, str(self.config / "ReadDataConfig.toml"))

    def worker(self, data_name=None, strategy="frame_by_frame", same_origin=True):
        from control.DataProcessWorker import DataProcessWorker
        from model.utils.TomlLoader import TomlLoader
        TomlLoader.save({"translate_data_origin": 1 if same_origin else 2}, str(self.config / "ReadDataConfig.toml"))
        return DataProcessWorker(queue.Queue(), queue.Queue(), queue.Queue(), str(self.root), data_name=data_name, strategy_name=strategy, config_dir=str(self.config))

    def write(self, name, x):
        (self.root / name).write_text(f"{x} 100 0\n", encoding="utf-8")

    def test_replay_waits_for_a_real_baseline_and_rebases_after_disconnect(self):
        worker = self.worker()
        samples = queue.Queue()
        for pulse, fifo, generation in ((3250, 325, 0), (3250, 325, 0), (3260, 326, 0), (-999, 0, 1),
                                        (3270, 327, 0), (8000, 800, 1), (8010, 801, 1)):
            samples.put({"pulse": pulse, "fifo": fifo, "plc_generation": generation})
        sent = []

        def read_feedback(*args, **kwargs):
            if read_feedback.first:
                read_feedback.first = False
                raise queue.Empty
            if samples.empty():
                raise SystemExit
            return samples.get_nowait()

        read_feedback.first = True
        worker.max_frame_index = 10
        with patch.object(worker.pulse_queue, "get", side_effect=read_feedback), \
                patch.object(worker.pulse_queue, "get_nowait", side_effect=queue.Empty), \
                patch.object(worker, "_process_current_frame", side_effect=lambda real_fifo_key, repeat_count: sent.append(real_fifo_key)), \
                self.assertRaises(SystemExit):
            worker._process_frames()
        self.assertEqual([326, 801], sent)

    def test_buffered_first_sample_uses_zero_as_a_valid_baseline(self):
        worker = self.worker()
        for fifo in (0, 1, 3):
            worker.pulse_queue.put({"pulse": fifo * 10, "fifo": fifo})
        worker.max_frame_index = 10
        with patch.object(worker, "_heartbeat_touch", side_effect=[None, SystemExit]), patch.object(worker, "_process_current_frame") as send:
            with self.assertRaises(SystemExit):
                worker._process_frames()
        self.assertEqual([1, 2, 3], [entry.kwargs["real_fifo_key"] for entry in send.call_args_list])

    def test_empty_tail_is_combined_after_last_real_frame_across_wrap(self):
        worker = self.worker()
        worker.max_fifo = 20
        worker.max_frame_index = 0
        with patch.object(worker, "_process_current_frame") as send:
            self.assertEqual(5, worker._process_fifo_advance(18, 2))
        self.assertEqual([(19, 1), (2, 4)], [(call.kwargs["real_fifo_key"], call.kwargs["repeat_count"]) for call in send.call_args_list])

    def test_empty_tail_packet_carries_no_point_cloud_frames(self):
        worker = self.worker()
        worker.plc_generation = 2
        worker.current_frame_index = 1
        worker.max_frame_index = 0
        worker._process_current_frame(real_fifo_key=338, repeat_count=13)
        packet = worker.machine_data_queue.get_nowait()
        self.assertEqual({"fifo": 338, "repeat_count": 13, "plc_generation": 2, "lidar_status": 0, "empty_frames": True}, packet)

    def test_bounded_queue_preserves_newest_frames(self):
        from model.utils.QueueUtil import DropOldestQueue

        frames = DropOldestQueue(2)
        try:
            self.assertIsNone(frames.put({"fifo": 325}))
            self.assertIsNone(frames.put({"fifo": 326}))
            self.assertEqual(325, frames.put({"fifo": 327})["fifo"])
            received = []
            deadline = time.monotonic() + 2
            while len(received) < 2 and time.monotonic() < deadline:
                try:
                    received.append(frames.get_nowait()["fifo"])
                except queue.Empty:
                    time.sleep(0.001)
            self.assertEqual([326, 327], received)
        finally:
            frames.close()

    def test_spawned_producer_keeps_only_newest_frames(self):
        from model.utils.QueueUtil import DropOldestQueue

        frames = DropOldestQueue(2)
        producer = multiprocessing.Process(target=_enqueue_replay_frames, args=(frames,))
        try:
            producer.start()
            producer.join(timeout=10)
            self.assertEqual(0, producer.exitcode)
            self.assertEqual([14, 15], [frames.get_nowait()["fifo"], frames.get_nowait()["fifo"]])
        finally:
            if producer.pid is not None and producer.is_alive():
                producer.terminate()
                producer.join(timeout=2)
            frames.close()

    def test_mode_one_sustained_replay_stays_bounded_with_a_paused_consumer(self):
        from model.utils.QueueUtil import QueueLimits

        self.write("sample.txt", 100)
        worker = self.worker("sample.txt")
        worker._load_all_frames()
        worker.max_fifo = 99
        limits = QueueLimits({"queue_limits": {"txt_frames": 4}})
        frames = limits.create("txt_frames", drop_oldest=True)
        self.addCleanup(frames.close)
        worker.machine_data_queue = frames
        worker.plc_generation = 1
        with patch("control.DataProcessWorker.logger.debug", new=lambda *args: None):
            for step in range(1, 1001):
                self.assertEqual(1, worker._process_fifo_advance((step - 1) % 100, step % 100))
        packets = [frames.get(timeout=2) for _ in range(4)]
        self.assertEqual([97, 98, 99, 0], [packet["fifo"] for packet in packets])
        self.assertTrue(all(packet["empty_frames"] for packet in packets))
        with self.assertRaises(queue.Empty):
            frames.get_nowait()

    def test_plc_places_a_frame_after_dropped_positions_and_ignores_late_packets(self):
        from control.PlcCommunicationProcess import PlcCommunicationProcess

        plc = PlcCommunicationProcess(queue.Queue(), queue.Queue(), queue.Queue())
        plc.max_fifo = 1000
        plc.txt_replay_applied_fifo = 325
        plc.plc_connection_generation = 3
        self.assertEqual((1, 1), plc._align_txt_replay_packet({"fifo": 328, "plc_generation": 3}, 1, current_fifo=329))
        self.assertIsNone(plc._align_txt_replay_packet({"fifo": 327, "plc_generation": 3}, 1, current_fifo=329))
        self.assertIsNone(plc._align_txt_replay_packet({"fifo": 329, "plc_generation": 2}, 1, current_fifo=329))
        self.assertEqual((0, 1), plc._align_txt_replay_packet({"fifo": 329, "plc_generation": 3}, 2, current_fifo=329))
        self.assertIn("缺少 2 格", plc.txt_replay_fault)

    def test_plc_does_not_shift_its_stack_again_when_replay_frame_arrives_late(self):
        from acquisition_sdk._internal.formats.frame_by_frame.AxisFrameDataFormat import AxisFrameData
        from control.PlcCommunicationProcess import PlcCommunicationProcess
        pending = queue.Queue()
        plc = PlcCommunicationProcess(pending, queue.Queue(), queue.Queue())
        plc.txt_frame_replay = True
        plc.max_fifo = 1000
        plc.plc_data = SimpleNamespace(ChainCountCM=329)
        earlier_frame = AxisFrameData([])
        replay_frame = AxisFrameData([])
        stack = [None, None, None, None, earlier_frame]
        plc.frame_queue_manager = SimpleNamespace(active_directions=("left",), frame_stack={"left": stack})
        plc.last_synced_chain_fifo = 329
        plc.txt_replay_applied_fifo = 325
        pending.put({"fifo": 328, "repeat_count": 1, "plc_generation": 0, "left": replay_frame})
        with patch.object(plc, "_should_skip_raw_packet", return_value=False), patch.object(plc, "_handle_frame_packet_received"):
            plc._process_frame_data()
        self.assertEqual([None, replay_frame, None, None, earlier_frame], stack)
        self.assertIs(replay_frame, stack[1])

    def test_watchdog_replay_floor_only_moves_when_the_stack_is_cleared(self):
        from control.PlcCommunicationProcess import PlcCommunicationProcess

        plc = PlcCommunicationProcess(queue.Queue(), queue.Queue(), queue.Queue())
        plc.txt_frame_replay = True
        plc.txt_replay_applied_fifo = 325
        plc.plc_data = SimpleNamespace(ChainCountCM=327)
        with patch.object(plc, "_update_raw_data_watchdog", side_effect=lambda: setattr(plc, "raw_data_timeout_active", True)):
            plc._sync_frame_queue_with_chain_fifo()
            plc.plc_data.ChainCountCM = 328
            plc._sync_frame_queue_with_chain_fifo()
        self.assertEqual(327, plc.txt_replay_applied_fifo)

    def test_configured_fifo_step_aligns_live_and_replay_frames(self):
        import numpy as np
        from acquisition_sdk import LidarAcquisitionProcess
        from control.PlcCommunicationProcess import PlcCommunicationProcess
        from model.utils.TomlLoader import TomlLoader

        TomlLoader.save({"frame_fifo_step_mm": 5, "max_pulse": 100, "pulse_to_mm": 1},
                        str(self.config / "ReadDataConfig.toml"))
        worker = self.worker()
        plc = PlcCommunicationProcess(queue.Queue(), queue.Queue(), queue.Queue())
        plc.read_data_config = worker.read_data_config
        plc.max_pulse = 100
        self.assertEqual(3, plc._pulse_to_chaincountcm(15))
        self.assertEqual(20, plc._resolve_max_fifo())
        self.assertEqual(20, worker.max_fifo)
        self.assertEqual(20, LidarAcquisitionProcess._get_max_chaincountcm(worker))
        groups = worker._split_points_into_frames(np.array([[1, 100, 0], [2, 100, 4], [3, 100, 5]]))
        self.assertEqual([2, 1], [len(group) for group in groups])

    def test_frame_replay_advances_through_all_datasets_without_resetting_plc_fifo(self):
        self.write("combined_10.txt", 10)
        self.write("combined_2.txt", 2)
        worker = self.worker()
        worker._load_all_frames()
        self.assertEqual(2, worker.all_frames["left"][0][0, 0])
        sent = []
        original = worker._process_current_frame

        def record(real_fifo_key, repeat_count):
            sent.append((real_fifo_key, worker.all_frames["left"][worker.current_frame_index][0, 0]))
            original(real_fifo_key, repeat_count)
        with patch.object(worker, "_process_current_frame", side_effect=record):
            self.assertEqual(2, worker._process_fifo_advance(20, 22))
        self.assertEqual([(21, 2), (22, 10)], sent)
        self.assertEqual([21, 22], [worker.machine_data_queue.get_nowait()["fifo"] for _ in range(2)])

    def test_explicit_name_processes_only_that_dataset(self):
        self.write("combined_2.txt", 2)
        self.write("combined_10.txt", 10)
        worker = self.worker("combined_10.txt")
        self.assertEqual(["10.txt"], [item["name"] for item in worker._discover_point_cloud_datasets()])
        worker._load_all_frames()
        self.assertEqual(10, worker.all_frames["left"][0][0, 0])

    def test_independent_origin_groups_directions_and_replays_groups_in_order(self):
        for suffix, x in (("10", 10), ("2", 2)):
            self.write(f"left_{suffix}.txt", x)
            self.write(f"right_{suffix}.txt", x + 100)
        worker = self.worker(same_origin=False)
        self.assertEqual(["2.txt", "10.txt"], [item["name"] for item in worker._discover_point_cloud_datasets()])
        worker._load_all_frames()
        worker._process_fifo_advance(0, 2)
        self.assertEqual(10, worker.all_frames["left"][0][0, 0])
        self.assertEqual(110, worker.all_frames["right"][0][0, 0])

    def test_bottom_txt_import_and_replay_use_the_configured_direction(self):
        from acquisition_sdk._internal.pointcloud.PointCloudFileProcess import resolve_txt_files

        self.write("left_2.txt", 20)
        (self.root / "down_2.txt").write_text("-21 2000 0\n21 2000 0\n", encoding="utf-8")
        (self.root / "right_down_2.txt").write_text("33 2000 0\n", encoding="utf-8")
        files = resolve_txt_files(self.root, "right_down_2.txt")
        self.assertEqual({"left_2.txt", "down_2.txt", "right_down_2.txt"}, {Path(path).name for path in files})
        worker = self.worker("down_2.txt", same_origin=False)
        worker._load_all_frames()
        self.assertEqual(-21, worker.all_frames["left_down"][0][0, 0])
        self.assertEqual(33, worker.all_frames["right_down"][0][0, 0])

    def test_same_origin_directory_accepts_plain_txt_without_counting_direction_files_twice(self):
        self.write("2.txt", 2)
        self.write("10.txt", 10)
        self.write("left_2.txt", 200)
        worker = self.worker()
        self.assertEqual(["2.txt", "10.txt"], [item["name"] for item in worker._discover_point_cloud_datasets()])
        named = self.worker("10.txt")
        named._load_all_frames()
        self.assertEqual(10, named.all_frames["left"][0][0, 0])

    def test_complete_workpiece_replay_sends_all_files_in_natural_order(self):
        self.write("combined_10.txt", 10)
        self.write("combined_2.txt", 2)
        worker = self.worker(strategy="complete_workpiece")
        with patch.object(worker, "_heartbeat_sleep"):
            worker._process_complete_workpieces()
        packets = [worker.machine_data_queue.get_nowait() for _ in range(2)]
        self.assertEqual([2, 10], [packet["all_data"][0, 0] for packet in packets])
        self.assertTrue(worker.machine_data_queue.empty())
