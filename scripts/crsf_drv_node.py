from crsf_drv.crsf_drv import CRSFConfiguration, CRSFDrv
from crsf_parser.handling import crsf_build_frame
from crsf_parser.payloads import PacketsTypes

from sensor_msgs.msg import Joy, BatteryState
import rospy

crsf_drv: CRSFDrv = None

def battery_callback(msg: BatteryState) -> None:
    if crsf_drv:
        frame = crsf_build_frame(
            PacketsTypes.BATTERY_SENSOR,
            {
                "voltage": int(msg.voltage * 10),
                "current": abs(int(msg.current * 10)),
                "capacity": 0, "remaining": 0
            },
        )
        crsf_drv.publish_telemetry(frame)

def main():
    """
    CRSF Driver node wrapper
    """
    global crsf_drv
    rospy.init_node("crsf_joy_emulation_driver")
    rospy.loginfo("CRSF joy emulation driver starting")
    battery_subscriber = rospy.Subscriber("/battery", BatteryState, battery_callback)
    
    publisher = rospy.Publisher("joy", Joy, queue_size=10)
    config = CRSFConfiguration(
        rospy.get_param("~mapping/axis", default=[]),
        rospy.get_param("~mapping/buttons", default=[]),
        rospy.get_param("~serial/port"),
        rospy.get_param("~serial/baudrate", 420000),
        rospy.get_param("~joy_message_rate", 50),
        rospy.get_param("~failsafe/timeout", 0.5),
        rospy.get_param("~failsafe/axis", [0, 0, 0, 0]),
        rospy.get_param("~failsafe/buttons", [0, 0, 0, 0]),
        rospy.get_param("~deadband", 0),
    )

    rospy.loginfo(config)
    crsf_drv = CRSFDrv(config, publisher)
    
    rospy.loginfo("CRSF joy emulation driver running")
    crsf_drv.run()


main()
