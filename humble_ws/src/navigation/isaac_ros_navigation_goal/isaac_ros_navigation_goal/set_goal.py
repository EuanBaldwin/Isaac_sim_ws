import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Empty
from std_msgs.msg import Empty as EmptyMsg
from .obstacle_map import GridMap
from .goal_generators import RandomGoalGenerator, GoalReader
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
from std_msgs.msg import UInt32
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import BatteryState


class SetNavigationGoal(Node):
    def __init__(self):
        super().__init__("set_navigation_goal")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("iteration_count", 1),
                ("goal_generator_type", "RandomGoalGenerator"),
                ("action_server_name", "navigate_to_pose"),
                ("obstacle_search_distance_in_meters", 0.2),
                ("frame_id", "map"),
                ("map_yaml_path", rclpy.Parameter.Type.STRING),
                ("goal_text_file_path", rclpy.Parameter.Type.STRING),
                ("initial_pose", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("waypoint_topic", "/waypoint_reached"),
                ("battery_status_topic", "/battery_status"),
                ("battery_low_threshold", 0.2),
                ("dock_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
                ("dock_charged_topic", "/dock_charged"),
            ],
        )

        self.__goal_generator = self.__create_goal_generator()
        action_server_name = self.get_parameter("action_server_name").value
        self._action_client = ActionClient(self, NavigateToPose, action_server_name)

        waypoint_topic = self.get_parameter("waypoint_topic").value
        self._waypoint_pub = self.create_publisher(UInt32, waypoint_topic, 10)
        self._waypoint_index = 0

        self._reset_cli = self.create_client(Empty, "/reset_tag_id_log")
        self._reset_cli.wait_for_service()

        self.MAX_ITERATION_COUNT = self.get_parameter("iteration_count").value
        self.curr_iteration_count = 1

        self.__initial_goal_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 1
        )
        self.__initial_pose = self.get_parameter("initial_pose").value
        self.__is_initial_pose_sent = True if self.__initial_pose is None else False

        self._battery_low_threshold = self.get_parameter("battery_low_threshold").value
        self._dock_pose = self.get_parameter("dock_pose").value
        self._heading_to_dock = False
        self._dock_goal_id = None
        self._current_goal_handle = None
        
        dock_charged_topic = self.get_parameter("dock_charged_topic").value
        self._dock_charged_pub = self.create_publisher(EmptyMsg, dock_charged_topic, 1)

        battery_topic = self.get_parameter("battery_status_topic").value
        self.create_subscription(BatteryState, battery_topic, self.__battery_callback, 10)

    def __battery_callback(self, msg):
        percentage = getattr(msg, "percentage", None)
        if (
            percentage is not None
            and percentage < self._battery_low_threshold
            and not self._heading_to_dock
        ):
            self._heading_to_dock = True
            if self._current_goal_handle:
                self._current_goal_handle.cancel_goal_async().add_done_callback(
                    lambda _f: self.__send_dock_goal()
                )
            else:
                self.__send_dock_goal()

    def __send_dock_goal(self):
        self.get_logger().info("Battery low, sending dock goal")
        self._action_client.wait_for_server()
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = self.get_parameter("frame_id").value
        g.pose.header.stamp = self.get_clock().now().to_msg()
        p = self._dock_pose
        g.pose.pose.position.x = p[0]
        g.pose.pose.position.y = p[1]
        g.pose.pose.orientation.x = p[2]
        g.pose.pose.orientation.y = p[3]
        g.pose.pose.orientation.z = p[4]
        g.pose.pose.orientation.w = p[5]
        self._action_client.send_goal_async(
            g, feedback_callback=self.__feedback_callback
        ).add_done_callback(self.__goal_response_callback)

    def __reset_tag_logger_then(self, cont):
        self._reset_cli.call_async(Empty.Request()).add_done_callback(lambda _f: cont())

    def __shutdown(self):
        rclpy.shutdown()

    def __send_initial_pose(self):
        g = PoseWithCovarianceStamped()
        g.header.frame_id = self.get_parameter("frame_id").value
        g.header.stamp = self.get_clock().now().to_msg()
        p = self.__initial_pose
        g.pose.pose.position.x = p[0]
        g.pose.pose.position.y = p[1]
        g.pose.pose.position.z = p[2]
        g.pose.pose.orientation.x = p[3]
        g.pose.pose.orientation.y = p[4]
        g.pose.pose.orientation.z = p[5]
        g.pose.pose.orientation.w = p[6]
        self.__initial_goal_publisher.publish(g)

    def send_goal(self):
        if not self.__is_initial_pose_sent:
            self.get_logger().info("Sending initial pose")
            self.__send_initial_pose()
            self.__is_initial_pose_sent = True
            time.sleep(10)
            self.get_logger().info("Sending first goal")

        self._action_client.wait_for_server()
        g = self.__get_goal()
        if g is None:
            self.__shutdown()
            sys.exit(1)

        self._action_client.send_goal_async(
            g, feedback_callback=self.__feedback_callback
        ).add_done_callback(self.__goal_response_callback)

    def __goal_response_callback(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().info("Goal rejected :(")
            if not self._heading_to_dock:
                self.__shutdown()
            return

        self.get_logger().info("Goal accepted :)")
        self._current_goal_handle = gh
        if self._heading_to_dock and self._dock_goal_id is None:
            self._dock_goal_id = gh.goal_id
        gh.get_result_async().add_done_callback(
            lambda f, h=gh: self.__get_result_callback(f, h)
        )

    def __get_result_callback(self, future, goal_handle):
        status = future.result().status

        if self._heading_to_dock and self._dock_goal_id and goal_handle.goal_id != self._dock_goal_id:
            return
            
        if self._heading_to_dock and status == GoalStatus.STATUS_SUCCEEDED:
            self._dock_charged_pub.publish(EmptyMsg())
            self.__reset_tag_logger_then(self.__shutdown)
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            m = UInt32()
            m.data = self._waypoint_index
            self._waypoint_pub.publish(m)
            self.get_logger().info(
                f"Waypoint {self._waypoint_index} reached "
                f"(published on {self._waypoint_pub.topic_name})"
            )
        else:
            self.get_logger().warn(
                f"Goal finished with status {status}; waypoint not logged"
            )

        self._waypoint_index += 1

        if self.curr_iteration_count < self.MAX_ITERATION_COUNT:
            self.curr_iteration_count += 1
            self.send_goal()
        else:
            self.__reset_tag_logger_then(self.__shutdown)

    def __feedback_callback(self, _):
        pass

    def __get_goal(self):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = self.get_parameter("frame_id").value
        g.pose.header.stamp = self.get_clock().now().to_msg()
        p = self.__goal_generator.generate_goal()
        if p is None:
            self.get_logger().error("Could not generate next goal. Exiting.")
            return
        g.pose.pose.position.x = p[0]
        g.pose.pose.position.y = p[1]
        g.pose.pose.orientation.x = p[2]
        g.pose.pose.orientation.y = p[3]
        g.pose.pose.orientation.z = p[4]
        g.pose.pose.orientation.w = p[5]
        return g

    def __create_goal_generator(self):
        t = self.get_parameter("goal_generator_type").value
        if t == "RandomGoalGenerator":
            yaml_path = self.get_parameter("map_yaml_path").value
            grid_map = GridMap(yaml_path)
            d = self.get_parameter("obstacle_search_distance_in_meters").value
            return RandomGoalGenerator(grid_map, d)
        if t == "GoalReader":
            f = self.get_parameter("goal_text_file_path").value
            return GoalReader(f)
        self.get_logger().info("Invalid goal generator specified. Exiting.")
        sys.exit(1)


def main():
    rclpy.init()
    node = SetNavigationGoal()
    node.send_goal()
    rclpy.spin(node)


if __name__ == "__main__":
    main()

