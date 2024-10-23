# IIWA FRI ROS 2 Stack

## Installation

1. Install ROS 2 development tools

    ```shell
    sudo apt install ros-dev-tools
    ```

2. Create a workspace, clone, and install dependencies

    ```shell
    source /opt/ros/rolling/setup.bash
    export FRI_CLIENT_VERSION=1.15
    mkdir -p lbr-stack/src && cd lbr-stack
    vcs import src --input https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/rolling/lbr_fri_ros2_stack/repos-fri-${FRI_CLIENT_VERSION}.yaml
    rosdep install --from-paths src -i -r -y
    ```

3. Build

    ```shell
    colcon build --symlink-install
    ```



**Note:** This repositry was forked from [lbr_fri_ros2_stack](https://github.com/lbr-stack/lbr_fri_ros2_stack) ROS Humble's branch.
