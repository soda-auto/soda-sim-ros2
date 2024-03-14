# SODA.Sim ROS2 Plugin
![SodaSim](Docs/img/promo.png)
This repository is a plugin for UnrealEngine and an extension of the [SODA.Sim](https://github.com/soda-auto/SodaSim) plugin to support ROS2 capabilities.
This plugin includes all the necessary precompiled ROS2 dependencies for windows and linux and does not require any other external dependencies.  
Current version of ROS2 is **Humble**.  
ROS2 workspaces are in the **ros2-linux** and **ros2-windows** folders. 

> [!NOTE]
> Resticted:
>   - For the Linux version supported only FastDDS RMW.

## How to Use
Make sure the [SODA.Sim](https://github.com/soda-auto/SodaSim) is installed.  
Copy the plugin to the UnrealEngine Plugins folder or to the your UProject plugins folder. See more information about [Working with Plugins in Unreal Engine](https://docs.unrealengine.com/5.0/en-US/working-with-plugins-in-unreal-engine/). 
 
**For Windows:**  
  - Download and install the Win64 OpenSSL v1.1.1 from [this page](https://slproweb.com/products/Win32OpenSSL.html).
  - ``` $ call ros2-windows/local_setup.bat``` every time before start IDE/UnrealEngine/UPoject.
  - Run IDE/UnrealEngine/UPoject.
	
**For Linux:**  
  - ``` $ source ros2-linux/local_setup.sh``` every time before start IDE/UnrealEngine/UPoject.
  - Run IDE/UnrealEngine/UPoject.
  
> [!TIP]
> If you don't desire execute the **local_setup** script every time, you may:
>   - for Windows - copy all \*.dll from *ros2-windows/bin*  to the *SodaROS2/Binaries/Win64*
>   - for Linux - copy all \*.so for linux from *ros2-linux/lib* to the *SodaROS2/Binaries/Linux*

 
## What is Currently Implemented
 - The **Generic Camera Sensor Publisher** using [sensor_msgs/msg/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html).
 To use it, add the **Generic Pinhole Camera** or **Generic Fisheye Camera** sensor to the vehicle and changle the **Publisher Class** to **ROS2CameraPublisher**.   
 ![](Docs/img/camera_image.jpg)
 - The **Generic Lidar Sensor Publisher** using [sensor_msgs/msg/LaserScan ](https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html).
 To use it, add the **Generic LiDAR Ray Trace** or **Generic LiDAR 2D Depth** sensor to the vehicle and changle the **Publisher Class** to **ROS2LidarLaserScanPublisher**.
 - The **Generic Lidar Sensor Publisher** using [sensor_msgs/msg/PointCloud](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud.html). To use it, add the **Generic LiDAR Ray Trace** or **Generic LiDAR 2D Depth** sensor to the vehicle and changle the **Publisher Class** to **ROS2LidarPointCloudPublisher**.
 - The **Generic Lidar Sensor Publisher** using [sensor_msgs/msg/PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html). To use it, add the **Generic LiDAR Ray Trace** or **Generic LiDAR 2D Depth** sensor to the vehicle and changle the **Publisher Class** to **ROS2LidarPointCloud2Publisher**.
 - The **Generic Nav Sensor Publisher** using [sensor_msgs/msg/NavSatFix ](https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html). To use it, add the **Generic Nav** sensor to the vehicle and changle the **Publisher Class** to **ROS2NavStatFixPublisher**.
 - The **Generic Nav Sensor Publisher** using [nav_msgs/msg/Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html). To use it, add the **Generic Nav** sensor to the vehicle and changle the **Publisher Class** to **ROS2OdometryPublisher**.
 - The **Generic Nav Sensor Publisher** using [nav_msgs/msg/Imu](https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html). To use it, add the **Generic Nav** sensor to the vehicle and changle the **Publisher Class** to **ROS2ImuPublisher**.
 - The *Generic Wheeled Vehicle Sensor Publisher* using [sensor_msgs/msg/JointState ](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html). To use it, add the **Generic Wheeled Vehicle** sensor to the vehicle and changle the **Publisher Class** to **ROS2WheelsJointStatePublisher**.
 - The **TF publisher sensor** using [tf2_msgs/TFMessage.msg](https://docs.ros.org/en/melodic/api/tf2_msgs/html/msg/TFMessage.html). To use it, add the **TF Publisher** component to the vehicle.
 - The **Generic Wheeled Vehicle Control**  using [ackermann_msgs/msg/AckermannDriveStamped](https://github.com/ros-drivers/ackermann_msgs/blob/ros2/msg/AckermannDriveStamped.msg). Using for control the wheeled vehicle via ackermann messages. To use it, add the **Generic Vehicle Dirver** to the vehicle and changle the **Vehicle Control Class** to **ROS2AckermannControl**.
 
## What Next
 - Support of the [ros2 control](https://control.ros.org/master/index.html) .
 - Support radar and ultrasonic sensors.
 - Add the fully equipped turtlebot model.
 - Add custom messages to control simulator via ROS2.
 - Suggest to us what we're missing ?
  
## How to Build ROS2 Manuly
Usually, you do not need to compile ROS2 from source code, but if you still have a need for this, then below it will be described how to do this for Windows and Linux.

### For for Windows
We don't build ROS2 from source for Windows. We use official prebuiled releases from [here](https://github.com/ros2/ros2/releases). 
 - Copy an ROS2 release for windows from [here](https://github.com/ros2/ros2/releases) and unzip it to *ros2-windows* folder.
 - Build and install the [ackermann_msgs](https://github.com/ros-drivers/ackermann_msgs/tree/ros2) to *ros2-windows* folder.
 - Build and install the [ros2-ue-wrapper]() to *Source/ThirdParty/rclcpp_static/Win64* folder
  ``` $ colcon build --merge-install --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=OFF ```

### For Linux 
Tested on Ubuntu 22.04 only.  
  - Perfom all step from [Original Manuale](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html) till to "Install additional DDS implementations (optional)".
  - Copy *Scripts/Build/\** to the ROS2 workspace folder
  - Remove *src/eclipse-cyclonedds* and *src/eclipse-iceoryx* folders
  - Copy [ackermann_msgs](https://github.com/ros-drivers/ackermann_msgs/tree/ros2) to the *src* folder.
  - ```$ export UE_ENGINE=<PATH_TO_UNREAL_ENGINE>/Engine```
  - Execute ```vcpkg-linux.sh``` script
  - Execute ```BuildForLinux.sh``` script
  - Build and install the [ros2-ue-wrapper]() to *Source/ThirdParty/rclcpp_static/Linux:*  
  ``` $ colcon build --merge-install --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=OFF -DCMAKE_TOOLCHAIN_FILE=<PATH_TO_PLUGIN>/Scripts/Build/unreal-linux-toolchain.cmake ```

## Copyright and License
Copyright © 2023 SODA.AUTO UK LTD. ALL RIGHTS RESERVED.  
This software contains code licensed as described in [LICENSE](https://github.com/soda-auto/SodaSim/blob/master/LICENSE.md).  