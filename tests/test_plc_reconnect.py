import unittest
from queue import Queue
from unittest.mock import Mock, call, patch

from acquisition_sdk._internal.plc.PlcCommon import BasePlc
from control.PlcCommunicationProcess import PlcCommunicationProcess
from model.MovingFrameData import END, START, ReadCommon, ReceiveMovingFrameData, SendMovingFrameData


class PlcReconnectTests(unittest.TestCase):
    def setUp(self):
        self.process = PlcCommunicationProcess(Queue(), Queue(), Queue())
        self.process.connection_type = "tcp"
        self.process.read_data_config["plc_connect_max_retries"] = 2
        self.process._heartbeat.sleep = Mock()
        self.process.plc = BasePlc("1", {"tcp_ip": "127.0.0.1", "tcp_port": 20001, "udp_ip": "127.0.0.1", "udp_port": 6002})
        self.initial_frame = START + bytes(SendMovingFrameData.FRAME_SIZE) + END

    def test_reconnect_sends_initial_frame_before_reading_feedback(self):
        disconnected = Mock()
        disconnected.recv.side_effect = ConnectionResetError("connection lost")
        self.process.plc._tcp_socket = disconnected
        self.process.plc_connected = True
        self.process._update_chain_status = Mock()
        reconnected = Mock()
        sent = []
        reconnected.sendall.side_effect = sent.append

        def receive_feedback(_size):
            if sent != [self.initial_frame, ReadCommon]:
                raise TimeoutError("PLC is waiting for the initial frame")
            return START + ReceiveMovingFrameData(ChainPulse=42).to_bytes() + END

        reconnected.recv.side_effect = receive_feedback
        with patch("acquisition_sdk._internal.plc.PlcCommon.socket.socket", return_value=reconnected):
            self.assertFalse(self.process._update_plc_data())
            self.assertTrue(self.process.plc_connected)
            self.assertTrue(self.process._update_plc_data())

        disconnected.close.assert_called_once_with()
        self.assertEqual(42, self.process.plc_data.ChainPulse)

    def test_reconnect_retries_when_sending_initial_frame_fails(self):
        failed = Mock()
        failed.sendall.side_effect = BrokenPipeError("could not send initial frame")
        reconnected = Mock()
        with patch("acquisition_sdk._internal.plc.PlcCommon.socket.socket", side_effect=[failed, reconnected]) as sockets:
            self.assertTrue(self.process._handle_connection_error(self.process.plc))

        self.assertEqual(2, sockets.call_count)
        self.assertIn(call.sendall(self.initial_frame), failed.mock_calls)
        failed.close.assert_called_once_with()
        reconnected.sendall.assert_called_once_with(self.initial_frame)
        self.assertTrue(self.process.plc_connected)
