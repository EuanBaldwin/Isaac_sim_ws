import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Empty
from .obstacle_map import GridMap
from .goal_generators import RandomGoalGenerator, GoalReader
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
from std_msgs.msg import UInt32
from action_msgs.msg import GoalStatus


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
            ],
        )

        self.__goal_generator = self.__create_goal_generator()
        action_server_name = self.get_parameter("action_server_name").value
        self._action_client = ActionClient(self, NavigateToPose, action_server_name)
        
        # publisher that announces when a waypoint is reached
        waypoint_topic = self.get_parameter("waypoint_topic").value
        self._waypoint_pub = self.create_publisher(UInt32, waypoint_topic, 10)
        self._waypoint_index = 0 

        # Tag‑ID‑logger reset service client (used only once at the end)
        self._reset_cli = self.create_client(Empty, '/reset_tag_id_log')
        self._reset_cli.wait_for_service()

        self.MAX_ITERATION_COUNT = self.get_parameter("iteration_count").value
        assert self.MAX_ITERATION_COUNT > 0
        self.curr_iteration_count = 1

        self.__initial_goal_publisher = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        self.__initial_pose = self.get_parameter("initial_pose").value
        self.__is_initial_pose_sent = True if self.__initial_pose is None else False

    def __reset_tag_logger_then(self, continuation_fn):
        """Call /reset_tag_id_log and, when it returns, run continuation_fn()."""
        future = self._reset_cli.call_async(Empty.Request())
        future.add_done_callback(lambda _f: continuation_fn())

    def __shutdown(self):
        rclpy.shutdown()

    def __send_initial_pose(self):
        goal = PoseWithCovarianceStamped()
        goal.header.frame_id = self.get_parameter("frame_id").value
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.__initial_pose[0]
        goal.pose.pose.position.y = self.__initial_pose[1]
        goal.pose.pose.position.z = self.__initial_pose[2]
        goal.pose.pose.orientation.x = self.__initial_pose[3]
        goal.pose.pose.orientation.y = self.__initial_pose[4]
        goal.pose.pose.orientation.z = self.__initial_pose[5]
        goal.pose.pose.orientation.w = self.__initial_pose[6]
        self.__initial_goal_publisher.publish(goal)


    def send_goal(self):
        if not self.__is_initial_pose_sent:
            self.get_logger().info("Sending initial pose")
            self.__send_initial_pose()
            self.__is_initial_pose_sent = True
            time.sleep(10)  # let AMCL converge
            self.get_logger().info("Sending first goal")

        self._action_client.wait_for_server()
        goal_msg = self.__get_goal()
        if goal_msg is None:
            rclpy.shutdown()
            sys.exit(1)

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.__feedback_callback
        )
        self._send_goal_future.add_done_callback(self.__goal_response_callback)

    def __goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted :)")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.__get_result_callback)

    def __get_result_callback(self, future):
        response = future.result()
        status_code = response.status
        
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            msg = UInt32()
            msg.data = self._waypoint_index
            self._waypoint_pub.publish(msg)
            self.get_logger().info(
                f"Waypoint {self._waypoint_index} reached "
                f"(published on {self._waypoint_pub.topic_name})"
            )
        else:
            self.get_logger().warn(
                f"Goal finished with status {status_code}; waypoint not logged"
            )

        self._waypoint_index += 1            # always advance label

        if self.curr_iteration_count < self.MAX_ITERATION_COUNT:
            self.curr_iteration_count += 1
            self.send_goal()
        else:
            self.__reset_tag_logger_then(self.__shutdown)

    def __feedback_callback(self, feedback_msg):
        pass

    def __get_goal(self):
        """Create and return the next NavigateToPose.Goal message."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.get_parameter("frame_id").value
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        pose = self.__goal_generator.generate_goal()

        if pose is None:
            self.get_logger().error("Could not generate next goal. Exiting.")
            return

        self.get_logger().info(f"Generated goal pose: {pose}")
        goal_msg.pose.pose.position.x = pose[0]
        goal_msg.pose.pose.position.y = pose[1]
        goal_msg.pose.pose.orientation.x = pose[2]
        goal_msg.pose.pose.orientation.y = pose[3]
        goal_msg.pose.pose.orientation.z = pose[4]
        goal_msg.pose.pose.orientation.w = pose[5]
        return goal_msg

    def __create_goal_generator(self):
        goal_generator_type = self.get_parameter("goal_generator_type").value

        if goal_generator_type == "RandomGoalGenerator":
            yaml_file_path = self.get_parameter("map_yaml_path").value
            if yaml_file_path is None:
                self.get_logger().info("Yaml file path is not given. Exiting.")
                sys.exit(1)

            grid_map = GridMap(yaml_file_path)
            obstacle_distance = self.get_parameter(
                "obstacle_search_distance_in_meters"
            ).value
            return RandomGoalGenerator(grid_map, obstacle_distance)

        elif goal_generator_type == "GoalReader":
            file_path = self.get_parameter("goal_text_file_path").value
            if file_path is None:
                self.get_logger().info("Goal text file path is not given. Exiting.")
                sys.exit(1)

            return GoalReader(file_path)

        else:
            self.get_logger().info("Invalid goal generator specified. Exiting.")
            sys.exit(1)


def main():
    rclpy.init()
    node = SetNavigationGoal()
    node.send_goal()
    rclpy.spin(node)


if __name__ == "__main__":
    main()

