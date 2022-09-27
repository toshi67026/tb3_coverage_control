#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import Pose, Twist
from tf.transformations import euler_from_quaternion


class Controller:
    def __init__(self) -> None:
        rospy.init_node("controller")

        # p-gain
        self.kp = float(rospy.get_param("/kp", default=1))
        timer_period = float(rospy.get_param("/timer_period", default=0.1))

        self.ref_pose = Pose()
        self.curr_pose = Pose()
        self.cmd_vel_in_agent = Twist()

        # pub
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # sub
        rospy.Subscriber("ref_pose", Pose, self.ref_pose_callback, queue_size=1)
        rospy.Subscriber("curr_pose", Pose, self.curr_pose_callback, queue_size=1)

        # timer
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

    def curr_pose_callback(self, msg: Pose) -> None:
        self.curr_pose = msg

    def ref_pose_callback(self, msg: Pose) -> None:
        self.ref_pose = msg

    def timer_callback(self, timer: rospy.Timer) -> None:
        orientation = self.curr_pose.orientation
        _, _, yaw = euler_from_quaternion(quaternion=[orientation.x, orientation.y, orientation.z, orientation.w])

        # 偏差
        x_err = self.ref_pose.position.x - self.curr_pose.position.x
        y_err = self.ref_pose.position.y - self.curr_pose.position.y

        # エージェント座標系での偏差
        x_err_in_agent = x_err * np.cos(yaw) + y_err * np.sin(yaw)
        y_err_in_agent = -x_err * np.sin(yaw) + y_err * np.cos(yaw)

        # p-control
        self.cmd_vel_in_agent.linear.x = self.kp * x_err_in_agent
        self.cmd_vel_in_agent.angular.z = self.kp * np.arctan2(y_err_in_agent, x_err_in_agent)

        self.cmd_vel_pub.publish(self.cmd_vel_in_agent)


def main() -> None:
    try:
        Controller()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"{e}")


if __name__ == "__main__":
    main()
