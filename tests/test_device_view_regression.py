import os
import unittest
from types import SimpleNamespace
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

class DeviceViewRegression(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def test_camera_x_points_towards_viewer(self):
        from view.qt.point_cloud_view import QtPointCloudView
        self.assertGreater(QtPointCloudView().camera_matrix()[2, 0], 0)

    def test_first_two_strategies_have_three_named_devices_and_sn_bits(self):
        from view.qt.pages import DeviceControlPage, DeviceConfigurationPage
        from model.telemetry.UiState import UiState, DeviceState
        for strategy in ('frame_by_frame', 'complete_workpiece'):
            sent = []
            controller = SimpleNamespace(strategy_name=strategy, set_device_enable_mask=sent.append)
            page = DeviceControlPage(controller)
            config = DeviceConfigurationPage(controller)
            self.assertEqual(['左侧外仿形', '左侧云雀', '右侧云雀'], [b.text() for b in page.device_buttons])
            self.assertEqual([b.text() for b in page.device_buttons], [b.text() for b in config.device_buttons.values()])
            self.assertTrue(config.fields)
            page.apply_state(UiState(strategy_name=strategy, running=True, pc_auto_mode=True, all_devices_safe=True, devices=(DeviceState('PLC', True),)))
            page.enable_button.click()
            page.device_buttons[2].click()
            self.assertEqual(9, sent[-1])

    def test_strategy_switch_reloads_devices(self):
        from view.qt.pages import DeviceControlPage
        from model.telemetry.UiState import UiState
        page = DeviceControlPage(SimpleNamespace(strategy_name='frame_by_frame'))
        page.apply_state(UiState.empty('continuous_bidirectional'))
        self.assertEqual(4, len(page.device_buttons))
