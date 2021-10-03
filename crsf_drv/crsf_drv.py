from dataclasses import dataclass
from threading import Lock, Thread
from typing import List
from construct import Container
from time import sleep, time
from sensor_msgs.msg import Joy
from .joy_publisher import JoyPublisher
from serial import Serial
from crsf_parser import CRSFParser, PacketsTypes, PacketValidationStatus
import rospy


@dataclass
class CRSFConfiguration:
    """
    CRSF driver configuration
    """

    axis_map: List[int]
    buttons_map: List[int]
    serial_port: str
    serial_baudrate: int
    joy_message_rate: float
    failsafe_timeout: float
    failsafe_axis: List[float]
    failsafe_buttons: List[float]
    deadband: float

    def __repr__(self) -> str:
        ret = f"""configuration:
        axis_map:{self.axis_map}
        buttons_map:{self.buttons_map}
        serial: {self.serial_port} @ {self.serial_baudrate}
        joy message rate: {self.joy_message_rate}
        failsafe: timeout {self.failsafe_timeout}, axis[{self.failsafe_axis}] , buttons [{self.failsafe_buttons}]
        deadband: {self.deadband}
        """
        return ret


class CRSFDrv:
    """
    CRSF Joy Driver implementaton
    """

    def __init__(self, config: CRSFConfiguration, publisher: rospy.Publisher) -> None:

        self._config = config
        self._message_pub = publisher
        self._joy_publisher = JoyPublisher(
            self._config.axis_map, self._config.buttons_map, self._message_pub
        )

        self._values_lock = Lock()
        self._last_values = None
        self._last_update_time: float = 0
        self._is_running = True

        self._crsf_parser = CRSFParser(self.publish)

        self._publishing_thread = Thread(target=self._publishing_worker)

    def _set_failsafe(self) -> None:
        self._joy_publisher.publish(
            self._config.failsafe_axis, self._config.failsafe_buttons
        )

    def _publishing_worker(self) -> None:
        try:
            previous_update = 0
            while self._is_running:
                with self._values_lock:
                    time_since_last_update = time() - self._last_update_time
                    if time_since_last_update > self._config.failsafe_timeout:
                        self._set_failsafe()
                    else:
                        if previous_update != self._last_update_time:
                            previous_update = self._last_update_time
                            self._joy_publisher.remap_and_publish(self._last_values)
                sleep(1.0 / self._config.joy_message_rate)
        finally:
            self._set_failsafe()
            self._is_running = False

    def publish(self, packet: Container, status: PacketValidationStatus) -> None:
        if status == PacketValidationStatus.VALID:
            if packet.header.type == PacketsTypes.RC_CHANNELS_PACKED:
                with self._values_lock:
                    # derived from CRSF spec Rev7, TICKS_TO_US(x) = ((x - 992) * 5 / 8 + 1500)
                    channels = [
                        ((x - 992) * 10 / 8000) for x in packet.payload.channels
                    ]
                    channels = [
                        x if abs(x) > self._config.deadband else 0 for x in channels
                    ]
                    # Inversion is a temporary workaround as the parser return them reversed
                    self._last_values = channels[::-1]
                    self._last_update_time = time()
        else:
            rospy.logwarn_throttle(
                5, f"received invalid data with status {status}, {packet}"
            )

    def run(self) -> None:
        with Serial(
            self._config.serial_port, self._config.serial_baudrate, timeout=2
        ) as ser:
            input_data = bytearray()
            self._is_running = True
            self._publishing_thread.start()
            while not rospy.is_shutdown():
                values = ser.read(100)
                input_data.extend(values)
                self._crsf_parser.parse_stream(input_data)

            self._is_running = False
            self._publishing_thread.join()
