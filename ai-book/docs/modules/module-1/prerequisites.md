# Prerequisites: Setting Up Your ROS 2 Environment

## Overview

Before diving into the exciting world of ROS 2 and humanoid robotics, it's crucial to set up a robust and functional development environment. This document provides detailed instructions for installing ROS 2 Foxy or Humble on an Ubuntu system, creating a ROS 2 workspace, and configuring the `colcon` build system. A correctly configured environment will ensure a smooth learning experience throughout Module 1.

---

## ROS 2 Foxy/Humble Installation on Ubuntu

This guide covers the installation of ROS 2 Foxy Fitzroy (for Ubuntu 20.04) or Humble Hawksbill (for Ubuntu 22.04), both Long Term Support (LTS) releases. We highly recommend using a fresh installation of the corresponding Ubuntu version for optimal compatibility.

### 1. Set Locale

Ensure your system's locale settings support UTF-8, which is essential for ROS 2.

```bash
locale  # Check current locale settings for UTF-8 support
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8 # Apply for current session
```

### 2. Add ROS 2 Repository

Add the ROS 2 GPG key and the repository to your system's `apt` sources.

```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 3. Install ROS 2 Packages

Update your package list and install the full desktop installation of ROS 2.

#### For ROS 2 Foxy (Ubuntu 20.04 - Focal Fossa)
```bash
sudo apt update
sudo apt install ros-foxy-desktop
```

#### For ROS 2 Humble (Ubuntu 22.04 - Jammy Jellyfish)
```bash
sudo apt update
sudo apt install ros-humble-desktop
```

### 4. Install Development Tools

Install additional Python tools necessary for ROS 2 development, including `colcon` and argument completion.

#### For ROS 2 Foxy
```bash
sudo apt install python3-rosdep2 python3-colcon-common-extensions python3-argcomplete
```

#### For ROS 2 Humble
```bash
sudo apt install python3-rosdep python3-colcon-common-extensions python3-argcomplete
```

### 5. Environment Setup

Source the ROS 2 setup script to make ROS 2 commands available in your shell. For persistent access, add the source command to your shell's configuration file (e.g., `~/.bashrc`).

```bash
# For Foxy (run this in your terminal):
source /opt/ros/foxy/setup.bash

# For Humble (run this in your terminal):
source /opt/ros/humble/setup.bash

# To make it permanent, add to ~/.bashrc (example for Humble):
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc # Apply changes immediately
```

Verify your installation by opening a new terminal and typing:
```bash
ros2 --version
```
You should see the installed ROS 2 distribution and version number.

## ROS 2 Workspace Setup and Configuration

A ROS 2 workspace is a structured directory where you manage your custom ROS 2 packages, allowing them to be built and integrated with your main ROS 2 installation.

### 1. Create a Workspace Directory

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```
This creates a directory `ros2_ws` in your home directory, with a `src` subdirectory where your ROS 2 packages will reside.

### 2. Build Your Workspace with `colcon`

`colcon` is the primary build tool for ROS 2. It efficiently compiles source code, generates necessary files, and manages dependencies for your packages.

```bash
cd ~/ros2_ws # Ensure you are in the root of your workspace
colcon build
```
The first time you run `colcon build` in a new workspace, it will compile all discovered packages. Subsequent builds will only recompile changed packages, speeding up development.

### 3. Source Your Workspace Overlay

After building, you must "overlay" your workspace on top of your ROS 2 installation. This makes your custom packages and executables discoverable by ROS 2.

```bash
source install/setup.bash
```
**Important**: This command must be run *after* you have sourced your main ROS 2 installation. For persistent setup, add this to your `~/.bashrc` file, ensuring it comes *after* the line sourcing `/opt/ros/<distro>/setup.bash`.

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc # Apply changes immediately
```

## `colcon` Build Process and Troubleshooting

### The `colcon build` Command

*   **`colcon build`**: Builds all packages in the `src` directory.
*   **`colcon build --packages-select <package_name>`**: Builds only a specific package and its dependencies. Useful for faster iteration.
*   **`colcon build --symlink-install`**: Creates symbolic links during installation, which is very helpful for Python packages. Changes to Python files don't require a rebuild.
*   **`colcon build --event-handlers console_direct+`**: Provides more detailed output during the build process, useful for debugging.

### Common Troubleshooting Tips

*   **"Package not found"**: Ensure your package is in the `src` directory of your workspace, and you've run `colcon build` and `source install/setup.bash`.
*   **Missing Dependencies**: If `colcon` reports missing dependencies, run `rosdep install --from-paths src --ignore-src -r -y`.
*   **Build Errors**: Always check the `log` directory in your workspace for detailed error messages. Look for output from `cmake`, `make`, or `python` specific to your package.
*   **Environment Not Sourced**: If `ros2` commands are not recognized, or your nodes aren't found, ensure your `~/.bashrc` is correctly configured and sourced in every new terminal.
*   **Clean Build**: Sometimes, old build artifacts can cause issues. A clean build can resolve this:
    ```bash
    rm -rf install build log
    colcon build
    ```

---

## Key Takeaways

*   Proper ROS 2 installation and environment setup are crucial prerequisites for development.
*   ROS 2 workspaces provide an organized structure for developing custom packages.
*   `colcon` is the standard build tool for ROS 2, managing compilation and dependencies.
*   Always source your environment (ROS 2 base and workspace overlay) correctly.
*   `colcon` offers various options for targeted and debug builds.

---

## Exercises

1.  **Repeat Installation**: If possible, try installing ROS 2 on another machine or in a virtual machine (e.g., VirtualBox with Ubuntu) to reinforce the process.
2.  **Verify Setup**: Open a new terminal. Run `printenv | grep ROS`. What environment variables do you see, and what do they indicate about your setup?
3.  **Create and Build Dummy Package**: Create an empty ROS 2 Python package named `my_dummy_package` in your workspace's `src` directory. Build only this package using `colcon build --packages-select my_dummy_package`. What changes do you observe in your workspace directories?
4.  **Simulate Missing Source**: Open a new terminal *without* sourcing your `~/.bashrc` or any ROS 2 setup files. Try to run `ros2 run demo_nodes_cpp talker`. What error do you get, and why? Then, manually source the ROS 2 base setup (`/opt/ros/<distro>/setup.bash`) and try again.
5.  **Clean Build Practice**: Practice performing a clean build of your workspace (`rm -rf install build log && colcon build`). Explain when this might be necessary.
