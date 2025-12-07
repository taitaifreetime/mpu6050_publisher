# MPU6050 on ESP32 as a ROS 2 NODE 
## Summarize
- ESP32 to ROS 2 publisher on a host PC connected via USB cable
- Up to 70Hz frequency for my case
- Debug log you can see via uart

## Before Getting Started
- Install Arduino IDE and Additional Board for ESP32
    - Important: Install ESP32 Board version ```2.0.X```. 

- Install micro ROS on your host PC


- Write code to ESP32
    - Arduino setting: 
        - Upload Speed > ```921600```
        - Port > ```/dev/ttyUSB0``` usually
        - Others > default
    - Edit mpu6050_publisher.ino
        - callback_duration_ms_: interval to run callback
        - ros_node_name_: 
        - ros_namespace_:
- Connect ESP32 to your host PC and Upload

## Run micro ROS Agent on your host PC
- ```ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baud 921600 --clock```
- You must subscribe the topic on your host PC.
- If you see the result on rviz2, you also need to install a plugin and publish tf from map to sensor. For just quick test, type the following cmd:
    - ```sudo apt install ros-humble-rviz-imu-plugin```
    - ```ros2 topic pub /tf_static tf2_msgs/msg/TFMessage "{transforms: [{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, child_frame_id: 'imu_link', transform: {translation: {x: 0.0, y: 0.0, z: 0.0}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}]}"```

## Bug?
- Topic frequency appears to be up to 30Hz.
    - Related sources on The Internet:
        - [MicroRos does not publish imu data in higher rates #1781](https://github.com/micro-ROS/micro_ros_arduino/issues/1781)
        - [esp32 can only publish up to 2hz. slow performance  #1817](https://github.com/micro-ROS/micro_ros_arduino/issues/1817)
        - [microros seems to have a publish limit of 1 second, anyone know how to change this?](https://www.reddit.com/r/ROS/comments/1fbqk5i/microros_seems_to_have_a_publish_limit_of_1/)
    - This was improved by making the msg size smaller. Imu.msg -> AccelStamped.msg then 30Hz -> 70Hz.