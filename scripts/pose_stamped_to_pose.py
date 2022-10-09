#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, PoseStamped


class PoseStampedToPose:
    def __init__(self) -> None:
        rospy.init_node("pose_stamped_to_pose")

        # pub
        self.curr_pose_pub = rospy.Publisher("curr_pose", Pose, queue_size=1)

        # sub
        rospy.Subscriber("pose", PoseStamped, self.pose_stamped_callback, queue_size=1)

    def pose_stamped_callback(self, msg: PoseStamped) -> None:
        self.curr_pose_pub.publish(msg.pose)


def main() -> None:
    try:
        PoseStampedToPose()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"{e}")


if __name__ == "__main__":
    main()
