import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

from std_msgs.msg import ColorRGBA

from visualization_msgs.msg import Marker

from arp_assignment.map_generator import GridMap
from arp_assignment.value_iteration import BackwardValueIteration
from arp_assignment.path_planner import PathPlanner


class BVINavigationNode(Node):

    def __init__(self):
        super().__init__("bvi_navigation")

        # =====================================================
        # Create map
        # =====================================================

        self.grid_map = GridMap()

        self.get_logger().info(
            f"Map created: {self.grid_map.width}x{self.grid_map.height}"
        )

        self.get_logger().info(
            f"Start: {self.grid_map.start}"
        )

        self.get_logger().info(
            f"Goal: {self.grid_map.goal}"
        )

        # =====================================================
        # Backward Value Iteration
        # =====================================================

        self.bvi = BackwardValueIteration(self.grid_map)

        self.values, self.policy = self.bvi.solve()

        # =====================================================
        # Optimal Path
        # =====================================================

        self.planner = PathPlanner(
            self.grid_map,
            self.policy
        )

        self.path = self.planner.get_path()

        self.get_logger().info(
            f"Optimal path length: {len(self.path) - 1}"
        )

        # =====================================================
        # Robot state
        # =====================================================

        self.current_index = 0

        # =====================================================
        # Publishers
        # =====================================================

        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            "/map",
            10
        )

        self.path_publisher = self.create_publisher(
            Path,
            "/optimal_path",
            10
        )

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            "/robot_pose",
            10
        )

        self.cost_publisher = self.create_publisher(
            Marker,
            "/cost_to_go",
            10
        )

        self.marker_publisher = self.create_publisher(
            Marker,
            "/robot_marker",
            10
        )

        self.start_goal_publisher = self.create_publisher(
            Marker,
            "/start_goal_markers",
            10
        )

        # =====================================================
        # Timer
        # Robot moves every 2 seconds
        # =====================================================

        self.timer = self.create_timer(
            2.0,
            self.update
        )

        self.get_logger().info(
            "BVI navigation node started."
        )

    # =========================================================
    # Main update loop
    # =========================================================

    def update(self):

        self.publish_map()

        self.publish_path()

        self.publish_robot()

        self.publish_robot_marker()

        self.publish_cost_to_go()

        self.publish_start_goal()

        # Move robot along optimal path

        if self.current_index < len(self.path) - 1:

            self.current_index += 1

    # =========================================================
    # Publish occupancy grid
    # =========================================================

    def publish_map(self):

        msg = OccupancyGrid()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = "map"

        msg.info.resolution = 1.0

        msg.info.width = self.grid_map.width

        msg.info.height = self.grid_map.height

        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0

        msg.info.origin.orientation.w = 1.0

        data = []

        for y in range(self.grid_map.height):

            for x in range(self.grid_map.width):

                if self.grid_map.is_obstacle((x, y)):

                    data.append(100)

                else:

                    data.append(0)

        msg.data = data

        self.map_publisher.publish(msg)

    # =========================================================
    # Publish optimal path
    # =========================================================

    def publish_path(self):

        msg = Path()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = "map"

        for x, y in self.path:

            pose = PoseStamped()

            pose.header = msg.header

            pose.pose.position.x = (
                float(x) + 0.5
            )

            pose.pose.position.y = (
                float(y) + 0.5
            )

            pose.pose.position.z = 0.05

            pose.pose.orientation.w = 1.0

            msg.poses.append(pose)

        self.path_publisher.publish(msg)

    # =========================================================
    # Publish robot pose
    # =========================================================

    def publish_robot(self):

        if not self.path:

            return

        x, y = self.path[
            self.current_index
        ]

        msg = PoseStamped()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = "map"

        msg.pose.position.x = (
            float(x) + 0.5
        )

        msg.pose.position.y = (
            float(y) + 0.5
        )

        msg.pose.position.z = 0.5

        msg.pose.orientation.w = 1.0

        self.pose_publisher.publish(msg)

    # =========================================================
    # Publish robot marker
    # =========================================================

    def publish_robot_marker(self):

        if not self.path:

            return

        x, y = self.path[
            self.current_index
        ]

        marker = Marker()

        marker.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        marker.header.frame_id = "map"

        marker.ns = "robot"

        marker.id = 0

        marker.type = Marker.CYLINDER

        marker.action = Marker.ADD

        # Robot position

        marker.pose.position.x = (
            float(x) + 0.5
        )

        marker.pose.position.y = (
            float(y) + 0.5
        )

        # Raise robot above the map

        marker.pose.position.z = 0.5

        marker.pose.orientation.w = 1.0

        # Bigger robot

        marker.scale.x = 0.9

        marker.scale.y = 0.9

        marker.scale.z = 0.8

        # Bright blue

        marker.color.r = 0.0

        marker.color.g = 0.0

        marker.color.b = 1.0

        marker.color.a = 1.0

        marker.lifetime.sec = 0

        self.marker_publisher.publish(
            marker
        )

    # =====================================================
    # Publish cost-to-go heatmap
    # =====================================================

    def publish_cost_to_go(self):

        marker = Marker()

        marker.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        marker.header.frame_id = "map"

        marker.ns = "cost_to_go"

        marker.id = 0

        marker.type = Marker.CUBE_LIST

        marker.action = Marker.ADD

        marker.scale.x = 0.9

        marker.scale.y = 0.9

        marker.scale.z = 0.02

        # Make heatmap slightly transparent

        marker.color.a = 0.25

        for y in range(
            self.grid_map.height
        ):

            for x in range(
                self.grid_map.width
            ):

                # Skip obstacles

                if self.grid_map.is_obstacle(
                    (x, y)
                ):

                    continue

                value = self.values[y, x]

                if not np.isfinite(value):

                    continue

                point = Point()

                point.x = (
                    float(x) + 0.5
                )

                point.y = (
                    float(y) + 0.5
                )

                point.z = 0.02

                marker.points.append(point)

                # Normalize cost

                max_value = np.max(
                    self.values[
                        np.isfinite(
                            self.values
                        )
                    ]
                )

                if max_value > 0:

                    normalized = (
                        value / max_value
                    )

                else:

                    normalized = 0.0

                color = ColorRGBA()

                # Blue -> Green -> Red

                color.r = float(
                    normalized
                )

                color.g = float(
                    1.0 - abs(
                        normalized - 0.5
                    ) * 2.0
                )

                color.b = float(
                    1.0 - normalized
                )

                color.a = 0.25

                marker.colors.append(
                    color
                )

        self.cost_publisher.publish(
            marker
        )

    # =====================================================
    # Publish start and goal markers
    # =====================================================

    def publish_start_goal(self):

        marker = Marker()

        marker.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        marker.header.frame_id = "map"

        marker.ns = "start_goal"

        marker.id = 0

        marker.type = Marker.SPHERE

        marker.action = Marker.ADD

        # Start

        sx, sy = self.grid_map.start

        marker.pose.position.x = (
            float(sx) + 0.5
        )

        marker.pose.position.y = (
            float(sy) + 0.5
        )

        marker.pose.position.z = 0.3

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.5

        marker.scale.y = 0.5

        marker.scale.z = 0.5

        marker.color.r = 0.0

        marker.color.g = 1.0

        marker.color.b = 0.0

        marker.color.a = 1.0

        self.start_goal_publisher.publish(
            marker
        )

        # =================================================
        # Goal marker
        # =================================================

        goal_marker = Marker()

        goal_marker.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        goal_marker.header.frame_id = "map"

        goal_marker.ns = "start_goal"

        goal_marker.id = 1

        goal_marker.type = Marker.SPHERE

        goal_marker.action = Marker.ADD

        gx, gy = self.grid_map.goal

        goal_marker.pose.position.x = (
            float(gx) + 0.5
        )

        goal_marker.pose.position.y = (
            float(gy) + 0.5
        )

        goal_marker.pose.position.z = 0.3

        goal_marker.pose.orientation.w = 1.0

        goal_marker.scale.x = 0.5

        goal_marker.scale.y = 0.5

        goal_marker.scale.z = 0.5

        goal_marker.color.r = 1.0

        goal_marker.color.g = 0.0

        goal_marker.color.b = 0.0

        goal_marker.color.a = 1.0

        self.start_goal_publisher.publish(
            goal_marker
        )


# ============================================================
# Main
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = BVINavigationNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()