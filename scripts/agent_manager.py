#!/usr/bin/env python3

from typing import Any, List, Tuple, Union

import numpy as np
import rospy
from coverage_control.parameters import GeometricPattern, color_list
from coverage_control.utils import get_color_rgba, get_random_color_rgba
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion, Vector3
from numpy.typing import NDArray
from sensor_msgs.msg import Joy
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from voronoi_tessellation.field import Field
from voronoi_tessellation.voronoi import Voronoi


class AgentManager:
    def __init__(self) -> None:
        rospy.init_node("agent_manager")

        self.world_frame = str(rospy.get_param("/world_frame", default="world"))

        self.agent_id = int(rospy.get_param("agent_id", default=0))
        self.agent_num = int(rospy.get_param("/agent_num", default=1))
        rospy.loginfo(f"agent_id: {self.agent_id}")

        # fieldを規定するパラメータを取得
        field_param = rospy.get_param("/field")
        self.dim = len(field_param["grid_accuracy"])
        grid_accuracy: NDArray = np.array(field_param["grid_accuracy"])
        limit: NDArray = np.array(field_param["limit"])

        rospy.loginfo(f"dim: {self.dim}")
        rospy.loginfo(f"grid_accuracy: {grid_accuracy}")
        rospy.loginfo(f"limit: {limit}")

        field = Field(grid_accuracy=grid_accuracy, limit=limit)
        self.voronoi = Voronoi(field)
        rospy.loginfo(f"grid_span: {field.grid_span}")

        self.geometric_pattern = GeometricPattern(str(rospy.get_param("/geometric_pattern", default="none")))
        rospy.loginfo(f"geometric_pattern: {self.geometric_pattern}")

        # Markerの重なりに応じて適切な透過度に設定
        alpha = 0.7 - 0.15 * self.dim
        # 台数が多い場合はrandomにカラーを選択
        color_rgba = (
            get_color_rgba(color_initial=color_list[self.agent_id], alpha=alpha)
            if self.agent_num <= len(color_list)
            else get_random_color_rgba(alpha=alpha)
        )

        self.voronoi_region_marker = Marker(
            header=Header(stamp=rospy.Time.now(), frame_id=self.world_frame),
            ns="voronoi_region",
            id=self.agent_id,
            action=Marker.ADD,
            type=Marker.CUBE_LIST,
            pose=Pose(orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
            scale=Vector3(*self.padding(original_array=self.voronoi.field_.grid_span, padding_value=0.05)),
            color=color_rgba,
        )
        self.geometric_radius = 0.5
        self.phi_cent_position = np.zeros(self.dim)

        # pub
        self.ref_pose_pub = rospy.Publisher("ref_pose", Pose, queue_size=1)
        self.voronoi_region_marker_pub = rospy.Publisher("voronoi_region_marker", Marker, queue_size=1)

        # sub
        rospy.Subscriber("/curr_pose_array", PoseArray, self.curr_pose_array_callback)
        if self.geometric_pattern != GeometricPattern.NONE:
            rospy.Subscriber("/joy", Joy, self.joy_callback)

    def curr_pose_array_callback(self, msg: PoseArray) -> None:
        grid_map = self.voronoi.field_.grid_map
        if self.geometric_pattern == GeometricPattern.LINE:
            self.voronoi.field_.phi = np.exp(
                -100 * (sum([grid_map[i] - self.phi_cent_position[i] for i in range(self.dim)])) ** 2
            )
        elif self.geometric_pattern == GeometricPattern.CIRCLE:
            self.voronoi.field_.phi = np.exp(
                -100
                * (
                    sum([(grid_map[i] - self.phi_cent_position[i]) ** 2 for i in range(self.dim)])
                    - self.geometric_radius**2
                )
                ** 2
            )
        elif self.geometric_pattern == GeometricPattern.DISK:
            self.voronoi.field_.phi = np.exp(
                -100
                * self.smooth_ramp(
                    sum([(grid_map[i] - self.phi_cent_position[i]) ** 2 for i in range(self.dim)])
                    - self.geometric_radius**2
                )
                ** 2
            )

        cent_position, voronoi_region_grid_map = self.calc_voronoi_tesselation(msg.poses)

        # 1~3次元に対応して使わない部分を0で埋めた上で，unpackしたものを渡す
        ref_pose = Pose(position=Point(*self.padding(cent_position)))

        self.ref_pose_pub.publish(ref_pose)

        self.voronoi_region_marker.header.stamp = rospy.Time.now()
        self.voronoi_region_marker.points.clear()
        for voronoi_region_point in zip(*voronoi_region_grid_map):
            self.voronoi_region_marker.points.append(Point(*self.padding(voronoi_region_point)))
        self.voronoi_region_marker_pub.publish(self.voronoi_region_marker)

    def calc_voronoi_tesselation(self, pose_list: List[Pose]) -> Tuple[NDArray, List[NDArray]]:
        all_agent_position_list: List[NDArray] = []
        for pose in pose_list:
            all_agent_position_list.append(np.array([pose.position.x, pose.position.y, pose.position.z]))

        agent_position = all_agent_position_list[self.agent_id]

        # 自分の位置のみ除外
        neighbor_agent_position_list = [
            neighbor_agent_position
            for agent_id, neighbor_agent_position in enumerate(all_agent_position_list)
            if agent_id != self.agent_id
        ]

        rospy.loginfo(f"neighbor_agent_position_list: {neighbor_agent_position_list}")

        # ボロノイ領域を計算し，重心と領域を示す離散点を取得
        cent_position, voronoi_region_grid_map = self.voronoi.calc_tesselation(
            agent_position=agent_position,
            neighbor_agent_position_list=neighbor_agent_position_list,
        )
        rospy.loginfo(f"agent_{self.agent_id} cent_position: {cent_position}")
        return cent_position, voronoi_region_grid_map

    @staticmethod
    def padding(
        original_array: Union[Tuple, List, NDArray], return_list_length: int = 3, padding_value: float = 0
    ) -> List[float]:
        """1~3次元それぞれへの対応として，後半部分を0 or 指定した値で埋める

        Args:
            original_array (Union[Tuple, List, NDArray]): 元の配列
            return_list_length (int): 埋めた後のリストのサイズ．Defaults to 3.
            padding_value (float): 埋める際に用いる値. Defaults to 0.

        Returns:
            List[float]: 埋めた結果のリスト

        Note:
            結果を渡す先にPoint, Vector3を想定しているのでreturn_list_lengthのデフォルト値を3に設定している．
        """
        original_array_length = len(original_array)
        return [original_array[i] if i < original_array_length else padding_value for i in range(return_list_length)]

    @staticmethod
    def smooth_ramp(x: Any) -> NDArray:
        ret: NDArray = x * (np.arctan(x) / np.pi + 1 / 2)
        return ret

    def joy_callback(self, msg: Joy) -> None:
        # 重要度分布の中心を操作
        self.phi_cent_position = np.array([-msg.axes[0], msg.axes[1]])
        # [0.25, 0.75]の範囲でgeometric_radiusを操作
        if self.geometric_pattern == GeometricPattern.CIRCLE or self.geometric_pattern == GeometricPattern.DISK:
            self.geometric_radius = (msg.axes[4] + 2) / 4


def main() -> None:
    try:
        AgentManager()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"{e}")


if __name__ == "__main__":
    main()
