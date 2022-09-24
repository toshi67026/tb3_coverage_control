#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener, TransformStamped


class TFToPose:
    def __init__(self) -> None:
        rospy.init_node("tf_to_pose")

        self.world_frame = str(rospy.get_param("world_frame", default="world"))
        self.agent_id = int(rospy.get_param("agent_id", default=0))
        timer_period = float(rospy.get_param("/timer_period", default=0.1))

        # tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # pub
        self.curr_pose_pub = rospy.Publisher("curr_pose", Pose, queue_size=1)

        # timer
        self.timer = rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

    def timer_callback(self, tiemr: rospy.Timer) -> None:
        trans: TransformStamped = self.tf_buffer.lookup_transform(
            self.world_frame,
            "tb3_" + str(self.agent_id),
            rospy.Time(0),
        )

        curr_pose = Pose(
            position=trans.transform.translation,
            orientation=trans.transform.rotation,
        )
        self.curr_pose_pub.publish(curr_pose)


def main() -> None:
    try:
        TFToPose()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"{e}")


if __name__ == "__main__":
    main()
