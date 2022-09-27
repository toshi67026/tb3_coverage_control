#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener, TransformStamped


class OdomToPose:
    def __init__(self) -> None:
        rospy.init_node("odom_to_pose")

        self.world_frame = str(rospy.get_param("world_frame", default="world"))

        # tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # pub
        self.curr_pose_pub = rospy.Publisher("curr_pose", Pose, queue_size=1)

        # sub
        rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=1)

    def odom_callback(self, msg: Odometry) -> None:
        # transform pose in odom frame to world frame
        pose_stamped_in_odom = PoseStamped(header=msg.header, pose=msg.pose.pose)
        trans: TransformStamped = self.tf_buffer.lookup_transform(
            self.world_frame, pose_stamped_in_odom.header.frame_id, rospy.Time(0)
        )
        pose_stamped_in_world = do_transform_pose(pose_stamped_in_odom, trans)
        self.curr_pose_pub.publish(pose_stamped_in_world.pose)


def main() -> None:
    try:
        OdomToPose()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"{e}")


if __name__ == "__main__":
    main()
