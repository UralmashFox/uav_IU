import unittest
from unittest.mock import patch, MagicMock
from mission_planner import MissionPlanner


class TestMissionPlanner(unittest.TestCase):
    @patch('mission_planner.UAVControl')
    def test_execute_mission_success(self, mock_uavcontrol):
        """Тест успешного выполнения миссии."""
        mock_uav = MagicMock()
        mock_uavcontrol.return_value = mock_uav

        waypoints = [(55.7558, 37.6173, 50), (55.7517, 37.6184, 60)]

        planner = MissionPlanner('test_connection_string')
        with self.assertRaises(Exception) as context:
            planner.execute_mission(waypoints)
        self.assertTrue('Не удалось достичь точки 1' in str(context.exception))

    @patch('mission_planner.UAVControl')
    def test_execute_mission_failure(self, mock_uavcontrol):
        """Тест неудачного выполнения миссии при недостижении одной из точек."""
        mock_uav = MagicMock()
        mock_uavcontrol.return_value = mock_uav

        waypoints = [(55.7558, 37.6173, 50), (55.7517, 37.6184, 60)]

        mock_uav.get_telemetry.side_effect = [
            {'lat': 55.7558, 'lon': 37.6173, 'alt': 49},  # Недолет по высоте
            {'lat': 55.7517, 'lon': 37.6184, 'alt': 61}
        ]

        with self.assertRaises(Exception) as context:
            planner = MissionPlanner('test_connection_string')
            planner.execute_mission(waypoints)
        print(context.exception)
        self.assertFalse('Не удалось достичь точки 1' in str(context.exception))


if __name__ == '__main__':
    unittest.main()