import multiprocessing
import queue
import unittest
from unittest.mock import patch

from model.utils.QueueUtil import QueueLimits


class QueueLimitTests(unittest.TestCase):
    def test_realtime_overflow_is_latched_without_dropping_queued_data(self):
        limits = QueueLimits({"queue_limits": {"realtime_frames": 1}})
        overflow = multiprocessing.Event()
        channel = limits.create("realtime_frames", overflow=overflow)
        try:
            channel.put({"fifo": 325})
            with self.assertRaises(queue.Full):
                channel.put({"fifo": 326})
            self.assertTrue(overflow.is_set())
            self.assertEqual({"fifo": 325}, channel.get(timeout=2))
        finally:
            channel.close()

    def test_configured_queue_rejects_a_non_positive_limit(self):
        with self.assertRaisesRegex(ValueError, "pulse"):
            QueueLimits({"queue_limits": {"pulse": 0}})

    def test_plc_latches_a_realtime_queue_fault_without_processing_stale_data(self):
        from control.PlcCommunicationProcess import PlcCommunicationProcess

        plc = PlcCommunicationProcess(queue.Queue(), queue.Queue(), queue.Queue())
        plc.realtime_overflow = multiprocessing.Event()
        with patch.object(plc, "_update_lidar_status") as fault:
            self.assertFalse(plc._handle_realtime_queue_overflow())
            plc.realtime_overflow.set()
            self.assertTrue(plc._handle_realtime_queue_overflow())
            self.assertTrue(plc._handle_realtime_queue_overflow())
        fault.assert_called_once_with(2)
        self.assertIn("实时数据队列已满", plc.realtime_queue_fault)

    def test_full_control_queue_preserves_the_previous_enable_request_id(self):
        from control.PipelineRuntime import PipelineRuntime

        runtime = PipelineRuntime.__new__(PipelineRuntime)
        runtime.control_type = 2
        runtime.control_queue = queue.Queue(maxsize=1)
        runtime._enable_request_id = 0
        runtime.send_control({"device_enable_mask": 3})
        with self.assertRaisesRegex(RuntimeError, "命令队列已满"):
            runtime.send_control({"device_enable_mask": 0})
        self.assertEqual(1, runtime._enable_request_id)
        self.assertEqual(3, runtime._requested_device_enable_mask)
