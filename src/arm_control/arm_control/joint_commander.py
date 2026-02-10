import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Trigger


class ButtonCommander(Node):
    def __init__(self):
        super().__init__("button_commander")

        self._ac = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )

        self.srv_a = self.create_service(Trigger, "/go_pose_a", self.go_pose_a_cb)
        self.srv_home = self.create_service(Trigger, "/go_home", self.go_home_cb)

            # MUST match URDF + controller joints
        self.joints = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
        ]

        self.home = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.pose_a = [0.7, -2.2, 1.2, -1.5, -1.57, 0.3]


        self.busy = False

    def _send_goal(self, target, duration_sec=2.0):
        if len(target) != len(self.joints):
            raise ValueError("Target length must match joints length")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joints)

        pt = JointTrajectoryPoint()
        pt.positions = list(target)
        pt.time_from_start.sec = int(duration_sec)
        pt.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        goal.trajectory.points = [pt]

        if not self._ac.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("follow_joint_trajectory action server not available")
            self.busy = False
            return

        self.busy = True
        send_future = self._ac.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            self.busy = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        res = future.result()
        status = res.status
        error_code = res.result.error_code
        self.get_logger().info(f"Result status={status} error_code={error_code}")
        self.busy = False

    def go_pose_a_cb(self, request, response):
        if self.busy:
            response.success = False
            response.message = "Busy"
            return response
        self._send_goal(self.pose_a, 2.0)
        response.success = True
        response.message = "Sent pose A goal"
        return response

    def go_home_cb(self, request, response):
        if self.busy:
            response.success = False
            response.message = "Busy"
            return response
        self._send_goal(self.home, 2.0)
        response.success = True
        response.message = "Sent home goal"
        return response


def main():
    rclpy.init()
    node = ButtonCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
