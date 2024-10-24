import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
import sys

class KukaIIWA(DHRobot):
    def __init__(self):
        # Link parameters based on the provided DH table
        links = [
            RevoluteDH(d=0.360, a=0, alpha=-np.pi/2),  # Link 1
            RevoluteDH(d=0, a=0, alpha=np.pi/2),        # Link 2
            RevoluteDH(d=0.420, a=0, alpha=np.pi/2),    # Link 3
            RevoluteDH(d=0, a=0, alpha=-np.pi/2),       # Link 4
            RevoluteDH(d=0.400, a=0, alpha=-np.pi/2),   # Link 5
            RevoluteDH(d=0, a=0, alpha=np.pi/2),        # Link 6
            RevoluteDH(d=0.152, a=0, alpha=0)           # Link 7
        ]
        super().__init__(links, name='Kuka IIWA')

class JointStateSubscriber(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name=node_name)

        # Initialize KUKA iiwa robot
        self.robot = KukaIIWA()

        # Initialize current joint positions
        self.current_joint_positions = [0.0] * 7

        # Subscribe to joint states
        self.get_logger().info("Subscribing to /lbr/joint_states topic")
        self.subscription = self.create_subscription(
            JointState, "/lbr/joint_states", self.joint_state_callback, 10
        )

    def joint_state_callback(self, msg: JointState):
        # Update current joint positions based on received joint states
        joint_positions = {}
        for name, position in zip(msg.name, msg.position):
            joint_positions[name] = position

        # Adjust the expected joint names (A1, A2, etc.)
        joint_names = [f"A{i+1}" for i in range(7)]
        if all(name in joint_positions for name in joint_names):
            self.current_joint_positions = [
                joint_positions[name] for name in joint_names
            ]

    def perform_inverse_kinematics(self, target_pose):
        # Compute the inverse kinematics using current joint positions as the initial guess
        ik_solution = self.robot.ikine_LM(target_pose, q0=self.current_joint_positions)

        # Check if the IK solution was successful
        if ik_solution.success:
            new_joint_positions = ik_solution.q  # Extract the new joint angles
            return new_joint_positions
        else:
            self.get_logger().error("Inverse Kinematics failed to find a solution.")
            return None

class JointTrajectoryClient(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name=node_name)
        self._joint_trajectory_action_client = ActionClient(
            node=self,
            action_type=FollowJointTrajectory,
            action_name="joint_trajectory_controller/follow_joint_trajectory",
        )
        while not self._joint_trajectory_action_client.wait_for_server(1):
            self.get_logger().info("Waiting for action server to become available...")
        self.get_logger().info("Action server available.")

    def execute(self, positions: list, sec_from_start: int = 10):
        if len(positions) != 7:
            self.get_logger().error("Invalid number of joint positions.")
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

        # send goal
        goal_future = self._joint_trajectory_action_client.send_goal_async(
            joint_trajectory_goal
        )
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by server.")
            return
        self.get_logger().info("Goal was accepted by server.")

        # wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=sec_from_start + goal_sec_tolerance
        )

        if (
            result_future.result().result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            self.get_logger().error("Failed to execute joint trajectory.")
            return

def main(args: list = None) -> None:
    rclpy.init(args=args)

    # Create JointStateSubscriber and JointTrajectoryClient nodes
    joint_state_subscriber = JointStateSubscriber("joint_state_subscriber")
    joint_trajectory_client = JointTrajectoryClient("joint_trajectory_client")

    # Initial position (z = 0.4) and orientation
    initial_position = np.array([0.62238, 0.0, 0.4])  # Target initial position (z = 0.4)
    desired_orientation = np.array([
        [-1, 0, 0],
        [ 0, 1, 0],
        [ 0, 0, -1],
    ])  # Constant rotation matrix

    # Perform inverse kinematics to get the joint positions for the initial position
    joint_state_subscriber.get_logger().info(f"Computing IK for initial position: {initial_position}")
    initial_pose = SE3.Rt(desired_orientation, initial_position)
    joint_positions = joint_state_subscriber.perform_inverse_kinematics(initial_pose)

    if joint_positions is not None:
        # Convert joint_positions to list and execute trajectory
        joint_positions = joint_positions.tolist()
        joint_trajectory_client.get_logger().info(f"Moving to initial position: {joint_positions}")
        joint_trajectory_client.execute(joint_positions, sec_from_start=5)
    else:
        joint_state_subscriber.get_logger().error("Could not compute a valid IK solution for initial position.")
        rclpy.shutdown()
        return

    # Define the eight corners of the 3D box (0.1x0.1x0.05 meters box)
    box_corners = [
        np.array([0.62238, 0.0, 0.4]),          # Bottom-front-left (initial position)
        np.array([0.72238, 0.0, 0.4]),          # Bottom-front-right
        np.array([0.72238, 0.1, 0.4]),          # Bottom-back-right
        np.array([0.62238, 0.1, 0.4]),          # Bottom-back-left
        np.array([0.62238, 0.0, 0.45]),         # Top-front-left
        np.array([0.72238, 0.0, 0.45]),         # Top-front-right
        np.array([0.72238, 0.1, 0.45]),         # Top-back-right
        np.array([0.62238, 0.1, 0.45]),         # Top-back-left
    ]

    # Loop to move through each corner of the 3D box repeatedly
    while rclpy.ok():
        for i, position in enumerate(box_corners):
            # Create the target pose for each corner
            target_pose = SE3.Rt(desired_orientation, position)

            # Perform inverse kinematics for the target pose
            joint_state_subscriber.get_logger().info(f"Computing IK for box corner {i+1}: {position}")
            joint_positions = joint_state_subscriber.perform_inverse_kinematics(target_pose)

            if joint_positions is not None:
                # Convert joint_positions to list and execute trajectory
                joint_positions = joint_positions.tolist()
                joint_trajectory_client.get_logger().info(f"Moving to box corner {i+1}: {joint_positions}")
                joint_trajectory_client.execute(joint_positions, sec_from_start=0.5)
            else:
                joint_state_subscriber.get_logger().error(f"Could not compute a valid IK solution for box corner {i+1}.")
                break  # Exit the loop if IK fails

     

    rclpy.shutdown()

if __name__ == "__main__":
    main()