@author Patrick Reichel <patrick.reichel@valeo.com>

@date December 2021

Requirements:
* Tested on Ubuntu 18.04 64bit with gnu7.5 compiler and `ROS Dashing Diademata` distribution
* Tested on Ubuntu 18.04 64bit with gnu7.5 compiler and `ROS Eloquent Elusor` distribution
* Tested on Ubuntu 20.04 64bit with gnu9.3 compiler and `ROS Foxy Fitzroy` distribution
* Tested on Ubuntu 20.04 64bit with gnu9.3 compiler and `ROS Galactic Geochelone` distribution

Build and Usage:
* Step 1: Open a terminal and go to `dev_ws` and run the command
          `colcon build --symlink-install` to build the ROS2 node.
* Step 2: Open a terminal and go to `dev_ws`.
          Start the node with the command `. install/setup.bash` followed by
          `ros2 run scala_decoder_sdk_publisher scala_decoder_sdk_publisher_node --ros-args
          -p HostIP:=YourHostIP -p HostPort:=YourHostPort -p MulticastIP:=YourMulticastIP
          -p PointCloudType:=YourPointCloudType`.
          You can visualize the data with rviz2. Start rviz2 with the command `ros2 run rviz2 rviz2`
          in an additional terminal.
* Step 3: Test the launch file. Adjust the parameters in the `run_launch.py` file with your settings.
          Open a terminal and go to `dev_ws`.
          Start the node with the command `. install/setup.bash` followed by
          `ros2 launch scala_decoder_sdk_publisher run_your_ros_distribution_launch.py`.

Troubleshooting:
The value of `your_ros_distribution` could be `dashing`, `eloquent`, `foxy` or `galactic`.
* Make sure that `source /opt/ros/your_ros_distribution/setup.bash` was executed, e.g. by adding it
  to your bash session every time a new shell is launched:
  `echo "source /opt/ros/your_ros_distribution/setup.bash" >> ~/.bashrc` followed by
  `source ~/.bashrc`
* Please check that you have installed `colcon`: `sudo apt install python3-colcon-common-extensions`
* Please check that you have installed `rclcpp`: `sudo apt-get install ros-your_ros_distribution-rclcpp`
* Please check that you have installed `pcl_conversions`:
  `sudo apt-get install ros-your_ros_distribution-pcl-conversions`
* Please check that you have installed `pcl_ros` (if it is possible with your distribution):
  `sudo apt-get install ros-your_ros_distribution-pcl-ros`

If you experience potential frame drops (incomplete point clouds received from Scala Sensor)
you can try to increase the receiving buffer size of your connections.
Do the following for operation systems Ubuntu 16.04 64bit, Ubuntu 18.04 64bit and Ubuntu 20.04 64bit:
* Open a terminal and execute `sudo sysctl -a | grep mem`. This will display your current buffer settings.
  It is recommended to save these. Maybe you may want to roll-back these changes.
* Execute `sudo sysctl -w net.core.rmem_max=buffersize`. This sets the max OS receive buffer size for all
  types of connections. You need to specify an integer for the placeholder `buffersize` in this command.
* Execute `sudo sysctl -w net.core.rmem_default=buffersize`. This sets the default OS receive buffer size for all
  types of connections. You need to specify an integer for the placeholder `buffersize` in this command.
A recommended value for `buffersize` is 2097152 (2 MB) or a higher value.
Do the following for Windows:
* Change the buffer sizes in the windows registry. Therefore open the registry editor app.
  Press the `Windows key` and the `R key` at the same time to open the Run box.
  Type `regedit` and hit `Enter` and you can access `registry editor` immediately.
  Now go to `HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Services\AFD\Parameters`.
  If the parameters `DefaultReceiveWindow` and `DefaultSendWindow` do not exists you need to
  add them to the registry.
  Add a new entry by left click on `Edit->New->DWORD(32-bit) Value` and set the value with double click
  on the respective parameters.
A recommended value for `DefaultReceiveWindow` and `DefaultSendWindow` is 2097152 in decimal
or 0x200000 in hexadecimal (2 MB) or a higher value.
