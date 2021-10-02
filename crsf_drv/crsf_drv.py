from threading import Lock, Thread
from typing import List, final
from pytest import param
from time import sleep, time
from sensor_msgs.msg import Joy
from .joy_publisher import JoyPublisher
from serial import Serial
from crsf_parser import CRSFParser
import rospy


class CRSFDrv:
    def __init__(self) -> None:

        self._message_pub = rospy.Publisher("~/joy", Joy, queue_size=10)
        axis_map = rospy.get_param("~mapping/axis", default=[])
        buttons_map = rospy.get_param("~mapping/buttons", default=[])
        self._joy_publisher = JoyPublisher(axis_map, buttons_map, self._message_pub)

        self._port = rospy.get_param("~serial/port")
        self._baudrate = rospy.get_param("~serial/baudrate", 425000)

        self._messages_period = 1.0 / rospy.get_param("~message_rate", 50)

        self._timeout = rospy.get_param("~failsafe/timeout", 0.5)
        self._failsafe_axis = rospy.get_param("~failsafe/axis", [0, 0, 0, 0])
        self._failsafe_buttons = rospy.get_param("~failsafe/buttons", [0, 0, 0, 0])

        self._values_lock = Lock()
        self._last_values = []
        self._last_update_time: float = 0
        self._is_running = True

        self._crsf_parser = CRSFParser(self._joy_publisher.publish)

        self._publishing_thread = Thread(target=self._publishing_worker)

    def _set_failsafe(self) -> None:
        self._joy_publisher.publish(self._failsafe_axis, self._failsafe_buttons)

    def _publishing_worker(self) -> None:
        try:
            previous_update_time: float = time()
            while self._is_running:
                time_since_last_update = previous_update_time - self._last_update_time
                if time_since_last_update > self._timeout:
                    self._set_failsafe()

                self._joy_publisher.remap_and_publish(self._last_values)
                sleep(self._messages_period)
        finally:
            self._set_failsafe()
            self._is_running = False

    def publish(self, values: List[float]) -> None:
        with self._values_lock:
            self._last_values = values

    def run(self) -> None:
        with Serial(self._port, self._baudrate, timeout=2) as ser:
            input = bytearray()
            while not rospy.is_shutdown():
                values = ser.read(100)
                input.extend(values)
                self._crsf_parser.parse_stream(input)
