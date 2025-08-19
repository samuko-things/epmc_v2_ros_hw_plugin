## EPMC (Easy PID Motor Controller) Version 2 ROS2 Hardware Interface Package

## How to Use the Package

#### Prequisite
- ensure you've already set up your microcomputer or PC system with [`ros-humble`](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) with [`colcon`](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) and your `ros workspace` also setup

- install the `libserial-dev` package on your linux machine
  ```shell
  sudo apt-get update
  sudo apt install libserial-dev
  ```

- install `rosdep` so you can install necessary ros related dependencies for the package.
  ```shell
  sudo apt-get update
  sudo apt install python3-rosdep
  sudo rosdep init
  rosdep update
  ```

#

#### How to build the epmc_ros_hw_plugin plugin package 
- In the `src/` folder of your `ros workspace`, clone the repo

- from the `src/` folder, cd into the root directory of your `ros workspace` and run rosdep to install all necessary ros dependencies
  ```shell
  cd ../
  rosdep install --from-paths src --ignore-src -r -y
  ```
- build the `epmc_v2_ros_hw_plugin` package with colcon (in the root folder of your ros workspace):
  ```shell
  colcon build --packages-select epmc_v2_ros_hw_plugin
  ```
> [!NOTE]   
> The **epmc_v2_ros_hw_plugin** package will now be available for use in any project in your ros workspace.
> You can see example of how the use the `epmc_v2_ros_hw_plugin` in the `example_control_file`

#

#### Sample robot test
 - please chekout the [**`easy_demo_bot`**](https://github.com/robocre8/demo_bot) package to see proper sample of how the EPMC is used.

#

#### Serial port check for the EPMC Module
- ensure the **`Easy PID Motor Controller (EPMC_v2) Module`**, with the motors connected and fully set up for velocity PID, is connected to the microcomputer or PC via USB.

- check the serial port the driver is connected to:
  > The best way to select the right serial port (if you are using multiple serial device) is to select by path
  ```shell
  ls /dev/serial/by-path
  ```
  > you should see a value (if the driver is connected and seen by the computer), your serial port would be -> /dev/serial/by-path/[value]. for more info visit this tutorial from [ArticulatedRobotics](https://www.youtube.com/watch?v=eJZXRncGaGM&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=8)

  - OR you can also try this:
  ```shell
  ls /dev/ttyU*
  ```
  > you should see /dev/ttyUSB0 or /dev/ttyUSB1 and so on

- once you have gotten the **port**, update the **port** parameter in the **`<ros2_control>`** tag in the URDF's **`epmc_v2_ros2_control.xacro`**

- you can the build and run your robot.

- Good Luck !!!
