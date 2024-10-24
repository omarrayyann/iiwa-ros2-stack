import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3

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
        self.get_logger().info(f"Received JointState message: {msg}")

        joint_positions = {}
        for name, position in zip(msg.name, msg.position):
            joint_positions[name] = position

        # Adjust the expected joint names (A1, A2, etc.)
        joint_names = [f"A{i+1}" for i in range(7)]
        if all(name in joint_positions for name in joint_names):
            self.current_joint_positions = [
                joint_positions[name] for name in joint_names
            ]
            self.get_logger().info(f"Current Joint Positions: {self.current_joint_positions}")

            # Perform forward kinematics to find the end-effector pose
            ee_pose = self.robot.fkine(self.current_joint_positions)
            self.get_logger().info(f"End-Effector Cartesian Position: {ee_pose.t}")
        else:
            self.get_logger().warn("Not all expected joint names were found in the message.")

def main(args: list = None) -> None:
    rclpy.init(args=args)
    node = JointStateSubscriber("joint_state_subscriber")

    node.get_logger().info("Spinning the node to listen for messages")
    rclpy.spin(node)

    node.get_logger().info("Shutting down the node")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
