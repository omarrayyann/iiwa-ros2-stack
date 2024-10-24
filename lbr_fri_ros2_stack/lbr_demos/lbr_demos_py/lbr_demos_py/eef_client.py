import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np
from spatialmath import SE3
from spatialmath.base import q2r
from roboticstoolbox import DHRobot, RevoluteDH
import threading
import time


class KukaIIWA(DHRobot):
    def __init__(self):
        pi = np.pi
        # Define the DH parameters based on the provided values
        d_bs = 0.340
        d_se = 0.400
        d_ew = 0.400
        d_wf = 0.152

        L = [
            RevoluteDH(a=0, alpha=-pi / 2, d=d_bs, offset=0, name="iiwa_joint_1"),
            RevoluteDH(a=0, alpha=pi / 2, d=0, offset=0, name="iiwa_joint_2"),
            RevoluteDH(a=0, alpha=pi / 2, d=d_se, offset=0, name="iiwa_joint_3"),
            RevoluteDH(a=0, alpha=-pi / 2, d=0, offset=0, name="iiwa_joint_4"),
            RevoluteDH(a=0, alpha=-pi / 2, d=d_ew, offset=0, name="iiwa_joint_5"),
            RevoluteDH(a=0, alpha=pi / 2, d=0, offset=0, name="iiwa_joint_6"),
            RevoluteDH(a=0, alpha=0, d=d_wf, offset=0, name="iiwa_joint_7"),
        ]
        super().__init__(L, name="KukaIIWA")


# Instantiate the robot model
robot = KukaIIWA()

class JointTrajectoryClient(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name=node_name)

        # Initialize current joint positions
        self.current_joint_positions = [0.0] * 7
        self.joint_state_received = False

        # Subscribe to joint states
        self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        # Action client
        self._joint_trajectory_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "joint_trajectory_controller/follow_joint_trajectory",
        )
        while not self._joint_trajectory_action_client.wait_for_server(1):
            self.get_logger().info("Waiting for action server to become available...")
        self.get_logger().info("Action server available.")

    def joint_state_callback(self, msg: JointState):
        # Update current joint positions based on received joint states
        joint_positions = {}
        for name, position in zip(msg.name, msg.position):
            joint_positions[name] = position

        # Ensure we have all joints
        joint_names = [f"iiwa_joint_{i+1}" for i in range(7)]
        if all(name in joint_positions for name in joint_names):
            self.current_joint_positions = [
                joint_positions[name] for name in joint_names
            ]
            self.joint_state_received = True

    def execute(self, positions: list, sec_from_start: int = 1):
        if len(positions) != 7:
            self.get_logger().error("Invalid number of joint positions.")
            self.joint_state_received = True

            return

        joint_trajectory_goal = FollowJointTrajectory.Goal()
        goal_sec_tolerance = 1
        joint_trajectory_goal.goal_time_tolerance.sec = goal_sec_tolerance

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.time_from_start.sec = sec_from_start

        for i in range(7):
            joint_trajectory_goal.trajectory.joint_names.append(f"A{i + 1}")

        joint_trajectory_goal.trajectory.points.append(point)

        # Send goal
        self.get_logger().info("Sending goal to action server.")
        send_goal_future = self._joint_trajectory_action_client.send_goal_async(
            joint_trajectory_goal
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by server.")
            return
        self.get_logger().info("Goal was accepted by server.")

        # Start a thread to print positions
        stop_thread = threading.Event()
        position_thread = threading.Thread(
            target=self.print_positions,
            args=(positions, stop_thread),
        )
        position_thread.start()

        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result

        # Stop the position printing thread
        stop_thread.set()
        position_thread.join()

        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error("Failed to execute joint trajectory.")
        else:
            self.get_logger().info("Joint trajectory execution successful.")

    def print_positions(self, goal_positions, stop_event):
        rate = self.create_rate(2)  # Print at 2 Hz
        while not stop_event.is_set():
            if self.joint_state_received:
                # Compute current end-effector position
                current_pose = robot.fkine(self.current_joint_positions)
                current_position = current_pose.t.tolist()
                # Compute goal end-effector position
                goal_pose = robot.fkine(goal_positions)
                goal_position = goal_pose.t.tolist()
                # Print positions
                self.get_logger().info(
                    f"Current Position: {current_position}, Goal Position: {goal_position}"
                )
            else:
                self.get_logger().info("Waiting for joint states...")
            rate.sleep()

    def move_to_pose(
        self, position: list, orientation: list = None, sec_from_start: int = 1
    ):
        """
        Move the robot to the specified end-effector position and orientation.

        Parameters:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
        - position: list of 3 floats [x, y, z]
        - orientation: Rotation matrix of sie (3,3)
        """
        # Compute the desired end-effector pose as SE3
        if orientation is None:
            # No orientation provided, use identity rotation
            T_desired = SE3.Trans(position)
        else:
            T_desired = SE3.Rt(orientation, position)

        # Compute inverse kinematics
        ik_solution = robot.ikine_LM(
            T_desired, q0=self.current_joint_positions, ilimit=1000
        )
        if not ik_solution.success:
            self.get_logger().error("Inverse kinematics failed to find a solution.")
            return

        joint_positions = ik_solution.q.tolist()
        self.get_logger().info(f"Computed joint positions: {joint_positions}")

        # Proceed to send the joint positions to the action server
        self.execute(joint_positions, sec_from_start)


def main(args: list = None) -> None:
    rclpy.init(args=args)
    joint_trajectory_client = JointTrajectoryClient("joint_trajectory_client")

    # Wait until initial joint states are received
    # while not joint_trajectory_client.joint_state_received:
    #     rclpy.spin_once(joint_trajectory_client)

    # Define the desired end-effector position and orientation
    position = [0.562, -0.095, 0.126]  # x, y, z in meters
    orientation = np.array([
        [1,0,0],
        [0,-1,0],
        [0,0,-1],
    ])
    # orientation = [-0/13, -1.0, 0.0, 0.0]  # Quaternion [qx, qy, qz, qw]

    # Move to specified end-effector pose
    joint_trajectory_client.get_logger().info("Moving to specified end-effector pose.")
    joint_trajectory_client.move_to_pose(position, orientation)

    rclpy.shutdown()

