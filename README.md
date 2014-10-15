Drivers
=======

USBoard

ROS Indigo with catkin Ubuntu 14.04


Steps
=======
1. Clone the repository into your catkin workspace src folder ($ git clone https://github.com/punithm/drivers.git)
2. Check the ttyUSB port in which the USBoard connected to PC via RS-232 or CAN (genrally /dev/ttyUSB0, update in ros/launch/usboard_param.yaml file if any other)
3. Make sure that neo_msgs package is added to your catkin build path
4. Build the package using catkin_make in your catkin workspace ( $ catkin_make --pkg neo_usboard)
5. Run the usboard roslaunch file ( $ roslaunch neo_usboard neo_usboard.launch)
6. To print and montior the published sensors data, run rostopic srb_us_measurements in seperate command window ($ rostopic echo /srb_us_measurements)
7. Stop the process killing all the nodes (Ctrl+C)
