from crsf_drv.crsf_drv import CRSFConfiguration, CRSFDrv
from sensor_msgs.msg import Joy
import rospy


def main():
    """
    CRSF Driver node wrapper
    """
    rospy.init_node("CRSF joy emulation driver")
    rospy.loginfo("CRSF joy emulation driver starting")
    publisher = rospy.Publisher("~/joy", Joy, queue_size=10)
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
