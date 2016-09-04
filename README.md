# enb355_emulator
Small ROS publisher to output dummy data in the form of images, pose messages, and sensor readings

This node will publish the following types:

1x geometry_msgs/TransformStamped
1x sensor_msgs/Image
3x std_msgs/Int32

To run:
roslaunch enb355_emulator emulator.launch
