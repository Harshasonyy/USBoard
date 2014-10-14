drivers
=======

USBoard

ROS Indigo with catkin Ubuntu 14.04


1. Clone the repository into your catkin workspace src folder
2. build the package using catkin_make ( $ catkin_make --pkg neo_usboard)
3. check the /ttyUSB port in which the USBoard connected to PC via RS-232 (genrally /dev/ttyUSB0, update in usboard_param.yaml file if other)
4. run the roslaunch file ( $ roslaunch neo_usboard neo_usboard.launch)
5. to print and montior the published data run rostopic /srb_us_measurements in seperate command window ($ rostopic echo /srb_us_measurements)
6. stop the process killing all the node (Ctrl+c)
