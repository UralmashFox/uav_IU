import unittest
from unittest.mock import patch, MagicMock
from uav_control import UAVControl


class TestUAVControl(unittest.TestCase):
    @patch('mavutil.mavlink_connection')
    def test_init(self, mock_mavlink_connection):
        """Тестирование инициализации соединения"""
        mock_master = MagicMock()
        mock_mavlink_connection.return_value = mock_master

        connection_string = 'tcp:127.0.0.1:5760'
        control = UAVControl(connection_string)

        mock_mavlink_connection.assert_called_once_with(connection_string)
        mock_master.wait_heartbeat.assert_called_once()
        self.assertEqual(control.master, mock_master)
        self.assertEqual(control.seq, 0)

    @patch('uav_control.UAVControl.master')
    def test_arm(self, mock_master):
        """Тестирование взведения (arm) БПЛА"""
        control = UAVControl('dummy_connection_string')

        control.arm()

        mock_master.arducopter_arm.assert_called_once()
        mock_master.motors_armed_wait.assert_called_once()

    @patch('uav_control.UAVControl.master')
    def test_disarm(self, mock_master):
        """Тестирование разоружения (disarm) БПЛА"""
        control = UAVControl('dummy_connection_string')

        control.disarm()

        mock_master.arducopter_disarm.assert_called_once()
        mock_master.motors_disarmed_wait.assert_called_once()

    @patch('uav_control.UAVControl.master')
    def test_takeoff(self, mock_master):
        """Тестирование взлета БПЛА"""
        control = UAVControl('dummy_connection_string')
        mock_master.recv_match.return_value = MagicMock(lat=123456700, lon=987654300)

        control.takeoff(10)

        mock_master.set_mode.assert_called_once_with('GUIDED')
        mock_master.recv_match.assert_called_once_with(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        mock_master.mav.command_long_send.assert_called_once_with(
            mock_master.target_system,
            mock_master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            12.34567,  # param5: Широта взлёта
            98.76543,  # param6: Долгота взлёта
            10          # param7: Высота взлёта
        )

    @patch('uav_control.UAVControl.master')
    def test_set_mode(self, mock_master):
        """Тестирование установки режима полета"""
        control = UAVControl('dummy_connection_string')
        mock_master.mode_mapping.return_value = {'GUIDED': 16}

        control.set_mode('GUIDED')

        mock_master.mode_mapping.assert_called_once()
        mock_master.set_mode.assert_called_once_with(16)

    @patch('uav_control.UAVControl.master')
    def test_get_telemetry(self, mock_master):
        """Тестирование получения телеметрии"""
        control = UAVControl('dummy_connection_string')
        mock_msg = MagicMock(lat=123456700, lon=987654300, alt=50000)
        mock_master.recv_match.return_value = mock_msg

        telemetry = control.get_telemetry()

        mock_master.recv_match.assert_called_once_with(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        self.assertIsNotNone(telemetry)
        self.assertAlmostEqual(telemetry['lat'], 12.34567, places=5)
        self.assertAlmostEqual(telemetry['lon'], 98.76543, places=5)
        self.assertAlmostEqual(telemetry['alt'], 50.0, places=5)

if __name__ == '__main__':
    unittest.main()