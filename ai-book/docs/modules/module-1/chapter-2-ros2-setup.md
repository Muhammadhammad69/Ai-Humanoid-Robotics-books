# Chapter 2 - ROS 2 Command Line & Workspace Setup

## Overview

This chapter guides you through setting up your development environment for ROS 2. We will cover the installation of ROS 2 Foxy or Humble on an Ubuntu system, demonstrate how to create a ROS 2 workspace, build packages using `colcon`, and run your very first ROS 2 node. A properly configured environment is the foundation for all subsequent robotics development.

---

## Learning Objectives
- Successfully install ROS 2 Foxy/Humble on a compatible Ubuntu system.
- Understand the concept of a ROS 2 workspace and how to create one.
- Learn to build ROS 2 packages using the `colcon` build tool.
- Execute a basic ROS 2 node and verify its operation.

---

## Installing ROS 2 Foxy/Humble on Ubuntu

This guide focuses on installing ROS 2 Foxy Fitzroy or Humble Hawksbill on Ubuntu Linux, which are popular Long Term Support (LTS) releases. We recommend using a fresh installation of Ubuntu 20.04 (for Foxy) or Ubuntu 22.04 (for Humble) for the best experience.

### Pre-installation Steps

1.  **Set locale**: Ensure your locale supports UTF-8.
    ```bash
    locale  # check for UTF-8
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    ```

2.  **Add ROS 2 repository**:
    ```bash
    sudo apt update && sudo apt install curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```

### Installation

Choose either Foxy or Humble based on your Ubuntu version.

#### For Foxy (Ubuntu 20.04 - Focal Fossa)
```bash
sudo apt update
sudo apt install ros-foxy-desktop
# Install development tools
sudo apt install python3-rosdep2 python3-colcon-common-extensions python3-argcomplete
```

#### For Humble (Ubuntu 22.04 - Jammy Jellyfish)
```bash
sudo apt update
sudo apt install ros-humble-desktop
# Install development tools
sudo apt install python3-rosdep python3-colcon-common-extensions python3-argcomplete
```

### Environment Setup

Source the ROS 2 setup script to make ROS 2 commands available in your current shell. For convenience, add this to your `~/.bashrc` or `~/.zshrc`.

```bash
# For Foxy:
source /opt/ros/foxy/setup.bash

# For Humble:
source /opt/ros/humble/setup.bash

# Add to .bashrc for persistence (example for Humble):
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify your installation by running `ros2 --version`.

## Creating a ROS 2 Workspace

A ROS 2 workspace is a directory where you can store, build, and manage your custom ROS 2 packages.

1.  **Create the workspace directory**:
    ```bash
    mkdir -p ~/ros2_ws/src
    ```
    Here, `ros2_ws` is the name of your workspace, and `src` is where your source code (packages) will reside.

2.  **Navigate into the workspace**:
    ```bash
    cd ~/ros2_ws
    ```

## Building Packages with `colcon`

`colcon` is the recommended build tool for ROS 2. It efficiently compiles multiple packages and handles their dependencies.

1.  **Copy a sample package (optional, for testing build)**:
    You can try building an example package from the ROS 2 tutorials. For instance, `demos/demo_nodes_cpp` contains simple C++ nodes.
    ```bash
    # (Assuming you are in ~/ros2_ws/src)
    git clone https://github.com/ros/ros_tutorials.git -b foxy-devel  # or humble-devel
    ```
    This will clone the `ros_tutorials` repository into your `src` folder.

2.  **Build the workspace**:
    Navigate back to the root of your workspace (`~/ros2_ws`) and run `colcon build`.
    ```bash
    cd ~/ros2_ws
    colcon build
    ```
    This command will build all packages found in your `src` directory. You will see `install`, `log`, and `build` directories created in your workspace.

3.  **Source the workspace**:
    After building, you must source the workspace's setup file to make the newly built packages available to your environment.
    ```bash
    source install/setup.bash
    ```
    For convenience, you can add this line to your `~/.bashrc` *after* sourcing the main ROS 2 installation (`/opt/ros/foxy/setup.bash` or `/opt/ros/humble/setup.bash`).
    ```bash
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
    **Important**: Always source the main ROS 2 setup *first*, then your workspace setup.

## Running Your First ROS 2 Node

Let's run the `talker` and `listener` nodes from the `demo_nodes_cpp` package you potentially cloned and built.

1.  **Open two terminal windows**.

2.  **In the first terminal, run the talker node**:
    ```bash
    ros2 run demo_nodes_cpp talker
    ```
    You should see it publishing "Hello World" messages.

3.  **In the second terminal, run the listener node**:
    ```bash
    ros2 run demo_nodes_cpp listener
    ```
    You should see it receiving and printing the "Hello World" messages.

Congratulations! You have successfully set up your ROS 2 environment and run your first nodes.

---

## Key Takeaways

*   ROS 2 installation involves adding repositories, installing packages, and sourcing environment scripts.
*   A ROS 2 workspace (`~/ros2_ws/src`) is essential for organizing and building custom packages.
*   `colcon` is the build tool used to compile ROS 2 packages within a workspace.
*   Always source your main ROS 2 installation *and then* your workspace setup after building.
*   `ros2 run` is used to execute a specific node from a package.

---

## Exercises

1.  **Verify Installation**: After following the installation steps, run `ros2 topic list`. What output do you get, and what does it tell you about the current ROS 2 environment?
2.  **Custom Workspace**: Create a new ROS 2 workspace named `my_first_robot_ws` in your home directory. Build it using `colcon`.
3.  **Explore Example Packages**: Browse the `src` directory of the `ros_tutorials` you cloned. Find a Python example package (e.g., `demo_nodes_py`), build your workspace again, and try running its `talker` and `listener` nodes.
4.  **Persistent Setup**: Explain why adding `source /opt/ros/<distro>/setup.bash` and `source ~/ros2_ws/install/setup.bash` to your `~/.bashrc` is important. What happens if you forget to do this or if you source them in the wrong order?