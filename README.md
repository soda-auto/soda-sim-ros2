# SODA.Sim ROS2 Plugin 
![SodaSim](Docs/img/promo.png)
This repository is a plugin for UnrealEngine and an extension of the [SODA.Sim](https://github.com/soda-auto/SodaSim) plugin to support ROS2 capabilities the for SODA.Sim.
This plugin includes all the necessary precompiled ROS2 dependencies (ROS2 workspace) for windows and linux and does not require any other external ROS2 dependencies.  
Current version of supported ROS2 is **Humble**.  
  
> [!WARNING]
> For the Linux version supported only FastDDS RMW.

## Installing
  - Make sure the [SODA.Sim](https://github.com/soda-auto/SodaSim) plugin is installed.  
  - Clone [ros-humble](https://github.com/soda-auto/ros-humble) to the plugins folder
  - Clone [soda-sim-ros2](https://github.com/soda-auto/soda-sim-ros2) to the plugins folder
  
> [!NOTE]  
> See more information about [Working with Plugins in Unreal Engine](https://docs.unrealengine.com/5.0/en-US/working-with-plugins-in-unreal-engine/). 
  
## What is Currently Implemented
 - **Generic Camera Sensor Publisher** using [sensor_msgs/msg/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html).
 - **Generic Lidar Sensor Publisher** using [sensor_msgs/msg/LaserScan ](https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html).
 - **Generic Lidar Sensor Publisher** using [sensor_msgs/msg/PointCloud](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud.html). 
 - **Generic Lidar Sensor Publisher** using [sensor_msgs/msg/PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html). 
 - **Generic Nav Sensor Publisher** using [sensor_msgs/msg/NavSatFix ](https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html). 
 - **Generic Nav Sensor Publisher** using [nav_msgs/msg/Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html). 
 - **Generic Nav Sensor Publisher** using [nav_msgs/msg/Imu](https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html). 
 - **Generic Wheeled Vehicle Sensor Publisher** using [sensor_msgs/msg/JointState ](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html). 
 - **Generic Radar Sensor Publisher** using [sensor_msgs/msg/PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html). 
 - **TF publisher sensor** using [tf2_msgs/TFMessage.msg](https://docs.ros.org/en/melodic/api/tf2_msgs/html/msg/TFMessage.html). 
 - **Generic Wheeled Vehicle Control**  using [ackermann_msgs/msg/AckermannDriveStamped](https://github.com/ros-drivers/ackermann_msgs/blob/ros2/msg/AckermannDriveStamped.msg).
 - **IOBus** 
 - **CANDev**
 - **Shaft Adapter Component**
 - **Wheel Interface Component**
 
## What Next
 - Support of the [ros2 control](https://control.ros.org/master/index.html) .
 - Support ultrasonic sensors.
 - Add the fully equipped turtlebot model.
 - Add custom messages to control SODA.Sim via ROS2.
 - Support the time sync messages
 - Add LIN/Serial virtual hardware interfaces
 - Suggest to us what we're missed ?
   
## Copyright and License
Copyright © 2023 SODA.AUTO UK LTD. ALL RIGHTS RESERVED.  
This software contains code licensed as described in [LICENSE](https://github.com/soda-auto/SodaSim/blob/master/LICENSE.md).  