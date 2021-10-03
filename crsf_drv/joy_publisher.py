from typing import List
from rospy import Publisher
from sensor_msgs.msg import Joy


class JoyPublisher:
    def __init__(
        self, axis_map: List[int], buttons_map: List[int], publisher: Publisher
    ) -> None:
        """
        Set up the publisher
        axis_map: list of indexes for each element in the Joy axis message
        button_map: list of indexes for each element in the Joy button message
        """
        self._axis_map = axis_map
        self._buttons_map = buttons_map
        self._joy_pub = publisher

    def _remap(self, source: List[float], map: List[int]) -> List[float]:
        destination = [source[x] if x >= 0 else 0 for x in map]
        return destination

    def remap_and_publish(self, values: List[float]) -> None:
        """
        Publish values to the Joy topic
        values: values to be published, mormalized between -1.0:1.0
        """
        axes = self._remap(values, self._axis_map)
        buttons_float = self._remap(values, self._buttons_map)
        buttons = [1 if x > 0 else 0 for x in buttons_float]
        self.publish(axes, buttons)

    def publish(self, axes: List[float], buttons: List[float]) -> None:
        msg = Joy()
        msg.axes = axes
        msg.buttons = buttons
        self._joy_pub.publish(msg)
