from pymavlink import mavutil  # type: ignore
import time
import math
from typing import List, Tuple
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class UAVControl:
    """
    Класс для управления БПЛА через MAVLink.
    """

    def __init__(self, connection_string: str):
        """
        Инициализация подключения к БПЛА.

        Args:
            connection_string (str): Строка подключения MAVLink.
        """
        try:
            self.master = mavutil.mavlink_connection(connection_string)
            self.master.wait_heartbeat()
            logger.info("Соединение установлено")
            self.seq = 0  # Инициализация последовательного номера миссии
        except Exception as e:
            logger.error(f"Ошибка подключения: {e}")
            raise

    def arm(self) -> None:
        """
        Взведение (Arm) БПЛА для начала работы двигателей.
        """
        try:
            self.master.arducopter_arm()
            self.master.motors_armed_wait()
            logger.info("БПЛА взведён")
        except Exception as e:
            logger.error(f"Ошибка взведения БПЛА: {e}")
            raise

    def disarm(self) -> None:
        """
        Разоружение (Disarm) БПЛА для остановки двигателей.
        """
        try:
            self.master.arducopter_disarm()
            self.master.motors_disarmed_wait()
            logger.info("БПЛА разоружён")
        except Exception as e:
            logger.error(f"Ошибка разоружения БПЛА: {e}")
            raise

    def takeoff(self, altitude: float) -> None:
        """
        Команда на взлёт до заданной высоты.

        Args:
            altitude (float): Целевая высота взлёта в метрах.
        """
        if altitude <= 0:
            raise ValueError("Высота должна быть положительной")
        try:
            self.set_mode('GUIDED')
            # Получение текущих координат
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
            if msg:
                current_lat = msg.lat / 1e7
                current_lon = msg.lon / 1e7
            else:
                raise Exception("Не удалось получить текущие координаты для взлёта")

            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0, 0, 0, 0,
                current_lat,  # param5: Широта взлёта
                current_lon,  # param6: Долгота взлёта
                altitude  # param7: Высота взлёта
            )
            if not self.wait_command_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF):
                raise Exception("Команда взлёта не подтверждена")
            logger.info(f"Взлёт на высоту {altitude} метров")
        except Exception as e:
            logger.error(f"Ошибка взлёта: {e}")
            raise

    def set_mode(self, mode: str) -> None:
        """
        Установка режима полёта БПЛА.

        Args:
            mode (str): Название режима (например, 'GUIDED', 'LAND').
        """
        mode_mapping = self.master.mode_mapping()
        if not isinstance(mode_mapping, dict):
            logger.error("Ошибка: mode_mapping() не вернул словарь")
            raise Exception("Не удалось получить список режимов полёта")

        mode_id = mode_mapping.get(mode)
        if mode_id is None:
            raise ValueError(f"Неизвестный режим: {mode}")

        try:
            self.master.set_mode(mode_id)
            logger.info(f"Режим установлен: {mode}")
        except Exception as e:
            logger.error(f"Ошибка установки режима {mode}: {e}")
            raise

    def get_telemetry(self) -> Optional[Dict[str, float]]:
        """
        Получение телеметрических данных от БПЛА.

        Returns:
            Optional[Dict[str, float]]: Словарь с телеметрическими данными или None.
        """
        telemetry = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if telemetry:
            return {
                'lat': telemetry.get('lat', 0.0),
                'lon': telemetry.get('lon', 0.0),
                'alt': telemetry.get('alt', 0.0)
            }
        else:
            logger.error("Телеметрия недоступна")
            return None

    def wait_command_ack(self, command: int) -> bool:
        """Ожидание подтверждения команды отправки"""
        result = self.master.command(command, blocking=True)
        return result.status == mavutil.mavutil.mavlink.MAVLinkCommand.ACK

    def execute_mission(self, waypoints: List[Tuple[float, float, float]]) -> None:
        if len(waypoints) == 0:
            logger.warning("Список точек пуст.")
            return
        for idx, waypoint in enumerate(waypoints):
            logger.info(f"Переходим к точке {idx + 1}: {waypoint}")
            self.uav.goto(*waypoint)
            reached = False
            for _ in range(5):
                telemetry = self.uav.get_telemetry()
                if telemetry:
                    lat_diff = abs(telemetry.get('lat', 0.0) - waypoint[0])
                    lon_diff = abs(telemetry.get('lon', 0.0) - waypoint[1])
                    alt_diff = abs(telemetry.get('alt', 0.0) - waypoint[2])
                    if lat_diff < 0.0001 and lon_diff < 0.0001 and alt_diff < 1.0:
                        reached = True
                        logger.info(f"Достигнута точка {idx + 1}")
                        break
                    time.sleep(1)
            if not reached:
                logger.error(f"Не удалось достичь точки {idx + 1}: ({waypoint})")
                raise Exception(f"Не удалось достичь точки {idx + 1}.")

        # Возвращение и посадка
        self.uav.set_mode('RTL')
        logger.info("Возвращение домой и посадка")
        time.sleep(5)
        self.disarm()