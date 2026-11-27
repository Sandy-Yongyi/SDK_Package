import multiprocessing as mp
import queue
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from acquisition_sdk._internal.lidar.LidarCommon import LidarSick120
from acquisition_sdk._internal.lidar.LidarAcquisitionProcess import LidarAcquisitionProcess
from acquisition_sdk._internal.dataprocess.DataSplitting import DataSplitting
from acquisition_sdk._internal.pointcloud.PointCloudSaveBuffer import PointCloudSaveBuffer, SharedPointCloudPool, read_blocks


class LidarOcclusionTests(unittest.TestCase):
    def test_scan_returns_cropped_occlusion_count_and_separate_fault(self):
        from unittest.mock import Mock

        lidar = LidarSick120.__new__(LidarSick120)
        lidar.id = '9'
        lidar.lidar_type = '120'
        lidar.start_angle, lidar.stop_angle = 0, 90
        lidar.radius_threshold = 100
        lidar._scan_faulted = False
        lidar._read_data = Mock(return_value=b'scan')
        lidar._parse_data = Mock(return_value=(np.array([10, 0, 5, 100, 50]), np.array([-1, 0, 45, 90, 91])))
        lidar._to_cartesian_same_origin = lidar._to_cartesian_diff_origin = Mock(return_value=np.empty((0, 2)))

        data, count, failed = lidar.scan()
        np.testing.assert_array_equal(data.radius, [0, 5, 100])
        self.assertEqual(1, count)
        self.assertFalse(failed)

        lidar._read_data.return_value = b''
        _, count, failed = lidar.scan()
        self.assertEqual(0, count)
        self.assertTrue(failed)

        lidar._read_data.return_value = b'scan'
        _, count, failed = lidar.scan()
        self.assertEqual(0, count)
        self.assertTrue(failed)

    def test_bottom_lidar_count_reaches_pc_plc_process_through_workpiece_processing(self):
        from unittest.mock import Mock, patch

        from acquisition_sdk._internal.dataprocess.DataFilter import DataFilter
        from acquisition_sdk._internal.lidar.LidarCommon import LidarData
        from control.DataProcessingProcess import DataProcessingProcess
        from control.PlcCommunicationProcess import PlcCommunicationProcess

        raw, machine = queue.Queue(), queue.Queue()
        from model.utils.TomlLoader import TomlLoader

        original_load = TomlLoader.load

        def load_with_shared_origin(path):
            config = original_load(path)
            if path.endswith("ReadDataConfig.toml"):
                config["translate_data_origin"] = 1
            return config

        with patch.object(TomlLoader, "load", side_effect=load_with_shared_origin):
            acq = LidarAcquisitionProcess(queue.Queue(), raw, queue.Queue(), {"down": ["9"]}, "model\\tomls", strategy_name="complete_workpiece")
        lidar = Mock()
        empty = np.empty((0, 2))
        data = LidarData("9", np.array([200]), np.array([0]), np.array([[1500, 2000]]), empty)
        empty_data = LidarData("9", np.array([]), np.array([]), empty, empty)
        lidar.scan.side_effect = [(data, 101, False), (empty_data, 0, True), (empty_data, 0, False)]
        acq.lidar_manager = Mock()
        acq.lidar_manager.get_lidar.return_value = lidar
        acq.data_filter = DataFilter()
        acq.data_split = DataSplitting()
        acq.strategy = Mock()
        processor = DataProcessingProcess.__new__(DataProcessingProcess)
        processor.machine_data_queue = machine
        plc = PlcCommunicationProcess(raw, queue.Queue(), queue.Queue(), machine_data_queue=machine,
                                      strategy_name="complete_workpiece")
        plc.read_data_config["radius_num"] = 100
        plc.frame_queue_manager = Mock()

        for sample, status, count in ((acq._capture_frame, 1, 101), (acq._poll_lidar_status_if_due, 2, 101),
                                      (lambda: acq._poll_lidar_status_if_due(force=True), 0, 0)):
            sample()
            processor._process_data(raw.get_nowait())
            plc._process_workpiece_data()
            self.assertEqual(status, plc.lidar_status)
            self.assertEqual([0] * 8 + [count], plc.lidar_occlusion_counts)

        self.assertEqual([("9",)] * 3, [call.args for call in acq.lidar_manager.get_lidar.call_args_list])
        self.assertEqual(["down"], acq.active_directions)
        np.testing.assert_array_equal(acq.direction_states["down"].last_same_origin_filtered[:, :2], [[1500, 2000]])
        self.assertEqual(0, acq.direction_states["down"].last_diff_origin_filtered.size)
        acq.direction_states["down"].scanning_started = True
        acq.save_buffer = Mock(free_blocks=10, descriptor={"block_count": 10})
        acq._save_valid_data_core(40)
        direction, saved = acq.save_buffer.append.call_args_list[0].args
        self.assertEqual("down", direction)
        np.testing.assert_array_equal(saved, [[1500, 2000, 40]])

    def test_bottom_lidar_uses_x_layers_for_live_frames_and_fifo_fill(self):
        from unittest.mock import patch

        from acquisition_sdk import FrameQueueManager
        from acquisition_sdk._internal.dataprocess.frame_by_frame.BuildSideFrame import build_side_frame
        from control.PlcCommunicationProcess import PlcCommunicationProcess
        from model.utils.LidarDirectionUtil import SIDE_DIRECTIONS, get_active_directions

        system = {"left_lidar_ids": ["1"], "left_upper_lidar_ids": ["7"], "down_lidar_ids": ["9"]}
        self.assertEqual(["left", "left_upper", "down"], get_active_directions(system))
        self.assertEqual(["left"], get_active_directions(system, SIDE_DIRECTIONS))
        with patch("acquisition_sdk._internal.formats.FrameQueueManager.TomlLoader.load", return_value=system):
            manager = FrameQueueManager("SystemConfig.toml", stack_size=2, x_min=10, x_max=30, x_threshold=10,
                                        y_min=100, y_max=130, y_threshold=10)

        self.assertEqual(2, len(manager.frame_stack["down"][0].FrameData))
        points = np.array([[11, 120], [19, 150]])
        config = {"combined_x_min": 10, "combined_x_max": 30, "x_threshold": 10}
        frame = build_side_frame({"down": [points]}, "down", config)
        self.assertEqual([(10, 150, 120), (0, 0, 0)],
                         [(row.H_Axis, row.V_Axis_Max, row.V_Axis_Min) for row in frame.FrameData])
        manager.push_frame("down", frame)
        plc = PlcCommunicationProcess(queue.Queue(), queue.Queue(), queue.Queue())
        plc.frame_queue_manager = manager
        plc._push_empty_frames(1)
        self.assertEqual(2, len(manager.frame_stack["down"][0].FrameData))
        self.assertIs(frame, manager.frame_stack["down"][1])
        self.assertEqual(3, len(manager.frame_stack["left"][0].FrameData))
        manager.clear("down")
        self.assertEqual([2, 2], [len(frame.FrameData) for frame in manager.frame_stack["down"]])

    def test_independent_origin_splits_one_bottom_scan_and_keeps_shared_frame_grid(self):
        from unittest.mock import Mock, patch

        from acquisition_sdk import DataFilter, FrameQueueManager
        from acquisition_sdk._internal.lidar.LidarCommon import LidarData
        from acquisition_sdk._internal.lidar.LidarAcquisitionStrategies import AcquisitionStrategyFactory
        from control.PlcCommunicationProcess import PlcCommunicationProcess
        from model.utils.TomlLoader import TomlLoader

        config = TomlLoader.load("model/tomls/ReadDataConfig.toml")
        config.update(translate_data_origin=2, x_threshold=5, left_down_x_min=-20, left_down_x_max=0,
                      right_down_x_min=-10, right_down_x_max=10, left_down_y_min=1100,
                      left_down_y_max=2300, right_down_y_min=1800, right_down_y_max=2600)
        original_load = TomlLoader.load
        with patch.object(TomlLoader, "load", side_effect=lambda path: config if path.endswith("ReadDataConfig.toml") else original_load(path)):
            raw = queue.Queue()
            acq = LidarAcquisitionProcess(queue.Queue(), raw, queue.Queue(), {"down": ["9"]}, "model\\tomls")
        local = np.array([[-20, 1200], [-10, 2200], [10, 2400], [0, 1500]])
        common = local + [1500, 0]
        acq.data_filter = DataFilter()
        acq.strategy = Mock()
        acq.lidar_manager = Mock()
        scanner = acq.lidar_manager.get_lidar.return_value
        scanner.scan.return_value = (LidarData("9", np.ones(4), np.zeros(4), common, local), 7, False)

        acq._capture_frame()
        scanner.scan.assert_called_once_with()
        self.assertEqual(["left_down", "right_down"], acq.active_directions)
        np.testing.assert_array_equal(acq.direction_states["left_down"].last_diff_origin_filtered, local[[0, 1, 3]])
        np.testing.assert_array_equal(acq.direction_states["right_down"].last_diff_origin_filtered, local[[1, 2]])
        self.assertEqual([0] * 8 + [7], raw.get_nowait()["lidar_occlusion_counts"])

        with patch.object(TomlLoader, "load", return_value={"down_lidar_ids": ["9"]}):
            manager = FrameQueueManager("SystemConfig.toml", stack_size=2, read_data_config=config)
        acq.cm_accum[2] = {direction: [state.last_diff_origin_filtered] for direction, state in acq.direction_states.items()}
        acq._send_single_fifo(2, 2)
        packet = raw.get_nowait()
        plc = PlcCommunicationProcess(queue.Queue(), queue.Queue(), queue.Queue())
        plc.frame_queue_manager = manager
        for direction in manager.active_directions:
            manager.push_frame(direction, packet[direction])
        plc._push_empty_frames(1)
        self.assertEqual([7, 7], [len(frame.FrameData) for frame in manager.frame_stack["right_down"]])
        self.assertEqual(2400, manager.frame_stack["right_down"][1].FrameData[-1].V_Axis_Max)
        manager.clear("left_down")
        self.assertEqual([7, 7], [len(frame.FrameData) for frame in manager.frame_stack["left_down"]])

        acq.data_split = DataSplitting()
        acq.save_buffer = Mock(free_blocks=10, descriptor={"block_count": 10})
        for state in acq.direction_states.values():
            state.scanning_started = True
        acq._save_valid_data_core(40)
        self.assertEqual(["left_down", "right_down", "combined"], [call.args[0] for call in acq.save_buffer.append.call_args_list])
        for name in ("complete_workpiece", "continuous_bidirectional"):
            for state in acq.direction_states.values():
                state.xyz_data = np.column_stack((state.last_diff_origin_filtered, np.zeros(len(state.last_diff_origin_filtered))))
            AcquisitionStrategyFactory.create_strategy(name, config).send_raw_data(acq)
            packet = raw.get_nowait()
            self.assertEqual(3, len(packet["left_down_data"]))
            self.assertEqual(2, len(packet["right_down_data"]))


class AcquisitionSavingTests(unittest.TestCase):
    def test_recovery_rearms_at_plc_pulse_and_waits_for_configured_empty_scans(self):
        from unittest.mock import Mock
        from acquisition_sdk._internal.lidar.LidarAcquisitionStrategies import FrameByFrameStrategy

        proc = self.process
        proc.direction_states = {}
        proc.cm_accum = {1: {}}
        proc.current_pulse = 12
        proc.current_status = "moving_forward"
        proc.lidar_status = 1
        proc._lidar_recovery_request = (1, 10)
        proc._waiting_for_lidar_recovery = True
        proc.read_data_config.update(start_skip_mm=20, start_skip_empty_frames=3)
        proc._push_lidar_status_update = Mock()
        proc.send_data_to_queue = Mock()
        strategy = FrameByFrameStrategy(proc.read_data_config)
        empty = {"left": np.empty((0, 3)), "right": np.empty((0, 3))}
        occupied = {**empty, "right": np.ones((1, 3))}

        proc._apply_lidar_recovery_request()
        self.assertTrue(proc._waiting_for_lidar_recovery)
        proc.lidar_status = 0
        proc._apply_lidar_recovery_request()
        self.assertFalse(proc._waiting_for_lidar_recovery)
        self.assertEqual(10, proc.start_motion_pulse)
        self.assertEqual({}, proc.cm_accum)
        self.assertEqual(1, proc.lidar_recovery_epoch)

        strategy.send_direction_data(proc, empty)
        proc.send_data_to_queue.assert_called_once_with(empty)
        proc.send_data_to_queue.reset_mock()
        strategy.send_direction_data(proc, occupied)
        for scan in (empty, empty, occupied, empty, empty):
            strategy.send_direction_data(proc, scan)
        self.assertTrue(proc.fbf_skipping_workpiece)
        proc.send_data_to_queue.assert_not_called()

        strategy.send_direction_data(proc, empty)
        self.assertTrue(proc.fbf_skip_complete)
        strategy.send_direction_data(proc, empty)
        proc.send_data_to_queue.assert_not_called()
        strategy.send_direction_data(proc, occupied)
        proc.send_data_to_queue.assert_called_once_with(occupied)

        proc._lidar_recovery_request = (2, 20)
        proc._apply_lidar_recovery_request()
        proc.current_pulse = 41
        strategy.send_direction_data(proc, empty)
        self.assertTrue(proc.software_first_start)
        strategy.send_direction_data(proc, occupied)
        self.assertFalse(proc.software_first_start)
        self.assertEqual(3, proc.send_data_to_queue.call_count)

    def test_missing_buffer_is_reported_before_capturing(self):
        self.process.save_buffer = None
        with self.assertRaisesRegex(RuntimeError, "save buffer is not initialized"):
            self.process._save_valid_data_core(0)

    def test_missing_task_queue_does_not_seal_or_lose_buffer(self):
        self.buffer.append("combined", np.ones((2, 3)))
        self.process.save_task_queue = None
        with self.assertRaisesRegex(RuntimeError, "task queue is not initialized"):
            self.process._save_workpiece_data()
        self.assertTrue(self.buffer.has_data)
        self.assertEqual(0, self.buffer.pending_jobs)

    def setUp(self):
        self.pool = SharedPointCloudPool.create(2400, 12)
        self.acks = mp.get_context("spawn").Queue()
        self.buffer = PointCloudSaveBuffer(self.pool.descriptor, self.acks)
        self.process = LidarAcquisitionProcess.__new__(LidarAcquisitionProcess)
        self.process.save_buffer = self.buffer
        self.process.save_task_queue = queue.Queue()
        self.process.save_status_queue = queue.Queue()
        self.process.stop_event = mp.get_context("spawn").Event()
        self.process.shutdown_flag = False
        self.process.plc_connection_generation = 7
        self.process.lidar_occlusion_counts = [0] * 9
        self.process.strategy_name = "frame_by_frame"
        self.process.read_data_config = {"translate_data_origin": 0, "y_threshold": 10,
                                         "max_pulse": 100, "pulse_to_mm": 1}
        self.process.data_split = DataSplitting()
        self.process.all_xyz_data = np.empty((0, 3))
        self.process.viz_queue = queue.Queue()
        self.process._save_job_counter = 0
        self.process.draw_type = 2
        self.process._processing_chunks = {}
        self.process.direction_states = {"left": SimpleNamespace(
            scanning_started=True, has_scanned=True, xyz_data=np.empty((0, 3)),
            last_same_origin_filtered=np.array([[10., 20.], [30., 40.]]),
            last_diff_origin_filtered=np.array([[100., 200.], [300., 400.]]))}

    def tearDown(self):
        self.buffer.close()
        self.pool.close()
        self.acks.close()
        self.acks.join_thread()

    def test_frame_by_frame_saves_all_points_without_whole_workpiece_arrays(self):
        self.process._save_valid_data_core(95)
        self.process._save_valid_data_core(2)
        self.assertEqual(len(self.process.direction_states["left"].xyz_data), 0)
        self.assertEqual(len(self.process.all_xyz_data), 0)
        self.process._save_workpiece_data()
        job = self.process.save_task_queue.get_nowait()
        left = np.concatenate(list(read_blocks(self.pool.descriptor, job["channels"]["left"])))
        combined = np.concatenate(list(read_blocks(self.pool.descriptor, job["channels"]["combined"])))
        self.assertEqual(len(left), 4)
        np.testing.assert_array_equal(left[:, :2], [[100, 200], [300, 400]] * 2)
        np.testing.assert_array_equal(combined[:, :2], [[10, 20], [30, 40]] * 2)
        self.assertTrue(self.process.viz_queue.empty())
        self.assertEqual(job["z_reset_threshold"], 100)

    def test_second_workpiece_can_submit_while_first_waits_for_saving(self):
        self.process._save_valid_data_core(0)
        self.process._save_workpiece_data()
        first = self.process.save_task_queue.get_nowait()
        self.process._save_valid_data_core(10)
        self.process._save_workpiece_data()
        second = self.process.save_task_queue.get_nowait()
        self.assertNotEqual(first["job_id"], second["job_id"])
        self.assertEqual(self.buffer.pending_jobs, 2)

    def test_stop_event_is_observed_in_inner_monitoring_loops(self):
        self.assertTrue(hasattr(self.process, "_stop_requested"), "stop event is not integrated")
        self.process.stop_event.set()
        self.assertTrue(self.process._stop_requested())

    def test_legacy_processing_strategy_keeps_accumulated_direction_data(self):
        self.process.strategy_name = "complete_workpiece"
        self.process._save_valid_data_core(95)
        self.process._save_valid_data_core(2)
        self.process._prepare_processing_data()
        self.assertEqual(len(self.process.direction_states["left"].xyz_data), 4)

    def test_three_modes_keep_their_own_coordinates_and_delivery_interfaces(self):
        from acquisition_sdk._internal.lidar.LidarAcquisitionStrategies import AcquisitionStrategyFactory
        for strategy in ("frame_by_frame", "complete_workpiece", "continuous_bidirectional"):
            for origin in (1, 2):
                with self.subTest(strategy=strategy, origin=origin):
                    self.buffer.release_unsaved()
                    self.process._processing_chunks = {}
                    self.process.strategy_name = strategy
                    self.process.read_data_config["translate_data_origin"] = origin
                    self.process.capture_direction = "reverse"
                    self.process.current_pulse = 2
                    self.process.lidar_status = 0
                    self.process.lidar_config = {"left": ["lidar1"]}
                    self.process.raw_data_queue = queue.Queue()
                    self.process.direction_states["left"].xyz_data = np.empty((0, 3))
                    self.process.direction_states["left"].stop_pulse = 2
                    self.process.continuous_scan_invalid = False
                    self.process._save_valid_data_core(95)
                    self.process._save_valid_data_core(2)
                    self.process._save_workpiece_data()
                    job = self.process.save_task_queue.get_nowait()
                    self.assertEqual(job["normalization"], "absolute_sorted" if strategy == "continuous_bidirectional" else "relative_z")
                    self.assertEqual(job["preview_enabled"], strategy != "complete_workpiece")
                    if strategy != "frame_by_frame":
                        self.process._prepare_processing_data()
                        data = self.process.direction_states["left"].xyz_data
                        self.assertEqual(len(data), 4)
                        self.assertEqual(set(data[:, 2]), {2., 95.} if strategy == "continuous_bidirectional" else {0., 7.})
                        AcquisitionStrategyFactory.create_strategy(strategy, self.process.read_data_config).send_raw_data(self.process)
                        packet = self.process.raw_data_queue.get_nowait()
                        self.assertEqual(packet["translate_data_origin"], origin)
                        self.assertEqual(packet["left_stop_pulse"], 2)
                        if origin == 1:
                            np.testing.assert_array_equal(packet["all_data"], self.process.all_xyz_data)
                        else:
                            np.testing.assert_array_equal(packet["left_data"], data)
                    # Release the test job; no real writer is running in this fixture.
                    self.buffer.acknowledge({"job_id": job["job_id"], "blocks": [b for v in job["channels"].values() for b in v]})

    def test_plc_frame_queue_continues_while_save_job_is_pending(self):
        from acquisition_sdk._internal.dataprocess.frame_by_frame.BuildSideFrame import build_side_frame
        self.process._save_valid_data_core(0)
        self.process._save_workpiece_data()
        self.process.raw_data_queue = queue.Queue()
        self.process.cm_accum = {}
        self.process.max_fifo = 160
        self.process.last_sent_fifo = -1
        self.process.active_directions = ["left"]
        self.process.build_side_frame = build_side_frame
        self.process.lidar_status = 0
        self.process.lidar_occlusion_counts[8] = 42
        self.process.read_data_config.update(combined_y_min=0, combined_y_max=500)
        for fifo in (10, 11, 12):
            self.process.current_fifo = fifo
            self.process.send_data_to_queue({"left": np.array([[100, 200], [300, 400]])})
        self.assertEqual(self.buffer.pending_jobs, 1)
        for fifo in (10, 11, 12):
            message = self.process.raw_data_queue.get_nowait()
            self.assertEqual(message["fifo"], fifo)
            self.assertEqual(message["repeat_count"], 1)
            self.assertEqual(message["lidar_status"], 0)
            self.assertEqual(message["lidar_occlusion_counts"], [0] * 8 + [42])
            self.assertEqual(message["plc_generation"], 7)
            self.assertEqual(len(message["left"].FrameData), 50)

    def test_repeated_fifo_samples_leave_no_old_point_cloud_buckets(self):
        from acquisition_sdk._internal.dataprocess.frame_by_frame.BuildSideFrame import build_side_frame

        self.process.raw_data_queue = queue.Queue()
        self.process.cm_accum = {}
        self.process.build_side_frame = build_side_frame
        self.process.last_sent_fifo = -1
        self.process.max_fifo = 400
        self.process.active_directions = ["left"]
        self.process.lidar_status = 0
        for fifo in (325, 325, 326, 326, 338, 338, 339):
            self.process.current_fifo = fifo
            self.process.send_data_to_queue({"left": np.array([[100, 200], [300, 400]])})
        self.assertEqual({}, self.process.cm_accum)
        packets = [self.process.raw_data_queue.get_nowait() for _ in range(4)]
        self.assertEqual([(325, 1), (326, 1), (338, 12), (339, 1)], [(packet["fifo"], packet["repeat_count"]) for packet in packets])
        self.assertTrue(self.process.raw_data_queue.empty())

    def test_mode_two_sustained_realtime_traffic_releases_old_cache_and_latches_backpressure(self):
        from acquisition_sdk import build_side_frame
        from model.utils.QueueUtil import QueueLimits

        process = self.process
        overflow = mp.Event()
        limits = QueueLimits({"queue_limits": {"realtime_frames": 2}})
        frames = limits.create("realtime_frames", overflow=overflow)
        self.addCleanup(frames.close)
        process.raw_data_queue = frames
        process.cm_accum = {}
        process.last_sent_fifo = -1
        process.max_fifo = 99
        process.active_directions = ["left"]
        process.build_side_frame = build_side_frame
        process.lidar_status = 0
        process.read_data_config.update(combined_y_min=0, combined_y_max=400, y_threshold=100)
        points = {"left": np.array([[100, 200]])}
        for step in range(1000):
            process.current_fifo = step % 100
            process.send_data_to_queue(points)
            process.send_data_to_queue(points)
            self.assertEqual(process.current_fifo, frames.get(timeout=2)["fifo"])
        self.assertFalse(overflow.is_set())
        self.assertEqual({}, process.cm_accum)
        for fifo in (0, 1):
            process.current_fifo = fifo
            process.send_data_to_queue(points)
        process.current_fifo = 2
        with self.assertRaises(queue.Full):
            process.send_data_to_queue(points)
        self.assertTrue(overflow.is_set())
        self.assertEqual([0, 1], [frames.get(timeout=2)["fifo"] for _ in range(2)])
        self.assertEqual({}, process.cm_accum)

    def test_realtime_capture_accepts_each_forward_position_once(self):
        self.process.last_frame_capture = None
        self.process.max_fifo = 339
        observed = (337, 337, 336, 338, 338, 339, 0, 0, 1)
        captured = [fifo for fifo in observed if self.process._should_capture_fifo(fifo)]
        self.assertEqual([337, 338, 339, 0, 1], captured)
        self.process.plc_connection_generation += 1
        self.assertTrue(self.process._should_capture_fifo(1))

    def test_realtime_capture_uses_one_plc_sample_for_scanning_and_saved_z(self):
        from acquisition_sdk._internal.lidar.LidarAcquisitionStrategies import FrameByFrameStrategy

        self.process.strategy_name = "frame_by_frame"
        self.process.current_pulse = 3260
        self.process.current_fifo = 326
        self.process.current_status = "moving_forward"
        state = Mock(scanning_started=False, has_scanned=False, start_pulse=None)
        state.should_start_scanning.return_value = False
        strategy = FrameByFrameStrategy({})
        with patch.object(self.process, "_update_pulse_data") as update, patch.object(self.process, "_save_valid_data_core"), \
                patch.object(self.process, "_get_current_scan_z_value") as to_z:
            strategy.update_scanning_state(self.process, "left", state, None, None)
            strategy.save_valid_data(self.process)
        update.assert_not_called()
        to_z.assert_called_once_with(3260)

    def test_realtime_capture_rebases_after_recovering_from_a_fifo_reverse_fault(self):
        self.process.strategy_name = "frame_by_frame"
        self.process.cm_accum = {}
        self.process.last_sent_fifo = 100
        self.process.current_status = "moving_forward"
        self.process.current_pulse = 700
        self.process.current_fifo = 70
        self.process.last_frame_capture = (self.process.plc_connection_generation, 100)
        self.process.max_fifo = 200
        with patch.object(self.process, "_invalidate_current_workpiece") as invalidate:
            self.assertFalse(self.process._should_capture_fifo(70))
            self.assertFalse(self.process._should_capture_fifo(71))
        invalidate.assert_called_once()
        self.process._rearm_startup_guard_after_lidar_recovery()
        self.assertFalse(self.process._should_capture_fifo(70))
        self.assertTrue(self.process._should_capture_fifo(71))

    def test_reset_discards_active_blocks_without_releasing_pending_save(self):
        from acquisition_sdk._internal.lidar.LidarDirectionState import LidarDirectionState
        self.process._save_valid_data_core(0)
        self.process._save_workpiece_data()
        job = self.process.save_task_queue.get_nowait()
        self.process._save_valid_data_core(10)
        config = {"left_scan_points_threshold": 10}
        self.process.direction_states = {"left": LidarDirectionState("left", config)}
        self.process._reset_base()
        self.assertFalse(self.buffer.has_data)
        self.assertEqual(self.buffer.pending_jobs, 1)
        points = np.concatenate(list(read_blocks(self.pool.descriptor, job["channels"]["left"])))
        self.assertEqual(len(points), 2)

    def test_continuous_interrupted_pass_releases_blocks_and_is_not_saved(self):
        from acquisition_sdk._internal.lidar.LidarAcquisitionStrategies import ContinuousBidirectionalStrategy
        from acquisition_sdk._internal.lidar.LidarDirectionState import LidarDirectionState
        from model.utils.ProcessHeartbeat import ProcessHeartbeat
        self.process.strategy_name = "continuous_bidirectional"
        self.process._save_valid_data_core(95)
        self.process.direction_states = {"left": LidarDirectionState("left", {"left_scan_points_threshold": 10})}
        self.process._heartbeat = ProcessHeartbeat()
        self.process.scan_position_tolerance = 1
        self.process.read_data_config["max_swing_pulse"] = 100
        self.process.plc_disconnected = True
        self.process.raw_data_queue = queue.Queue()
        self.process.lidar_status = 3
        self.process._update_pulse_data = lambda: (10, 0, "moving_forward")
        ContinuousBidirectionalStrategy(self.process.read_data_config).monitor_movement(self.process)
        self.assertTrue(self.process.continuous_scan_invalid)
        self.assertTrue(self.process.continuous_wait_for_idle)
        self.assertFalse(self.buffer.has_data)
        self.assertEqual(self.process._processing_chunks, {})
        self.process._save_workpiece_data()
        self.assertTrue(self.process.save_task_queue.empty())


if __name__ == "__main__":
    unittest.main()
