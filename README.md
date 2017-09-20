# TSRT10

******
Setup of LIDAR (scanse sweep, https://github.com/scanse/sweep-ros):
- Install the libsweep library from sweep-sdk (https://github.com/scanse/sweep-sdk): 
- - $ cd ~/TSRT10
- - $ git clone https://github.com/scanse/sweep-sdk
- - $ cd sweep-sdk/libsweep
- - $ mkdir -p build
- - $ cd build
- - $ cmake .. -DCMAKE_BUILD_TYPE=Release
- - $ cmake --build .
- - $ sudo cmake --build . --target install
- - $ sudo ldconfig

- Create a catkin workspace:
- - $ cd ~/TSRT10
- - $ mkdir catkin_ws
- - $ cd catkin_ws
- - $ mkdir src
- - $ catkin_make
- - Add "source ~/TSRT10/catkin_ws/devel/setup.bash" to the bottom of ~/.bashrc ($ sudo nano ~/.bashrc to edit, we do this for this line to be run everytime we open the terminal, otherwise we'd have to do it manually)  
- - $ source ~/TSRT10/catkin_ws/devel/setup.bash
 
- Clone the sweep-ros repo:
- - $ cd ~/TSRT10/catkin_ws/src
- - $ git clone https://github.com/scanse/sweep-ros.git
- - $ cd ~/TSRT10/catkin_ws
- - $ catkin_make

- Make sure that your username is in the dialout group in /etc/group (otherwise you won't have permission to open /dev/ttyUSB0):
- - $ sudo nano /etc/group
- - In my case, I hade to change the line "dialout:x:20:" to "dialout:x:20:fregu856"
- - Restart the computer

- Now, you should be able to launch sweep.launch:
- - $ roslaunch sweep_ros sweep.launch

- sweep2scan.launch and view_sweep_laser_scan.launch will however not work. For this we need to install the package pointcloud_to_laserscan:
- - $ sudo apt-get install ros-kinetic-pointcloud-to-laserscan

- Now, you should be able to launch sweep2scan.launch:
- - $ roslaunch sweep_ros sweep2scan.launch
- - Open a new terminal window
- - $ rostopic echo /scan
- - Alot of data should now be printed

- You should also be able to launch view_sweep_laser_scan.launch:
- - $ roslaunch sweep_ros view_sweep_laser_scan.launch (this should launch rviz where you can see a visualization of the scans)

****

Run Hector SLAM (only using the LiDAR scans, no odometry):
- Useful links: http://wiki.ros.org/hector_slam/Tutorials/SettingUpForYourRobot, https://github.com/tu-darmstadt-ros-pkg/hector_slam/blob/catkin/hector_slam_launch/rviz_cfg/mapping_demo.rviz, https://github.com/tu-darmstadt-ros-pkg/hector_slam/blob/catkin/hector_slam_launch/launch/mapping_box.launch
- $ cd ~/TSRT10/catkin_ws/src
- $ git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
- $ cd ~/TSRT10/catkin_ws
- $ catkin_make

- Create and build a package (called test_pckg) in the catkin workspace:
- - $ cd ~/TSRT10/catkin_ws/src
- - $ catkin_create_pkg test_pckg std_msgs roscpp rospy
- - $ cd ~/TSRT10/catkin_ws
- - $ catkin_make

- - Create a scripts directory in the package (it's in this directory we would place all python ROS code/scripts):
- - - $ cd ~/TSRT10/catkin_ws/src/test_pckg
- - - $ mkdir scripts
- - Every python script that one writes and places in scripts (e.g. test.py) must be made executable:
- - - $ chmod a+x test.py
- - You should always also build the package (this is sometimes (quite often) needed even for python scripts since we use C++ messages):
- - - $ cd ~/TSRT10/catkin_ws
- - - $ catkin_make

- Create a directory called "launch" in ~/TSRT10/catkin_ws/src/test_pckg
- Create a directory called "rviz" in ~/TSRT10/catkin_ws/src/test_pckg
- Write test_Hector.launch (based on the above links) and place it in the launch directory
- Write test_Hector.rviz (based on mapping_demo.rviz linked above) and place it in the rviz directory
- $ cd ~/TSRT10/catkin_ws
- $ catkin_make
- [terminal 1] $ roslaunch sweep_ros sweep2scan.launch
- [terminal 2] $ roslaunch test_pckg test_Hector.launch

****

Test communication between two computers:

- Connect the computers to the same WiFi network (eduroam doesn't seems to work)
- On computer1 (Master):
- - Find computer1's IP address by running $ ifconfig and look for "inet addr" below "wlan0, we call this xxx.xx.x.xx
- - $ sudo nano ~/.bashrc
- - Add the following two lines to the bottom of the file: "export ROS_MASTER_URI=http://xxx.xx.x.xx:11311" and "export ROS_HOSTNAME=xxx.xx.x.xx"
- - $ source ~/.bashrc


- Note the laptop's IP address (by running $ ifconfig. In my case I got: 172.24.1.72) 
- In the Ubuntu RPI terminal:
- - $ sudo nano ~/.bashrc
- - Replace the second-to-last line "export ROS_MASTER_URI=http://localhost:11311" with "export ROS_MASTER_URI=http://172.24.1.72:11311" (where 172.24.1.72 is the IP address of the laptop)
- - Replace the last line "export ROS_HOSTNAME=localhost" with "export ROS_HOSTNAME=172.24.1.57" (where 172.24.1.57 is the IP address of the Ubuntu RPI)
- - $ source ~/.bashrc
- In the laptop terminal:
- - $ sudo nano ~/.bashrc
- - Add the following two lines to the bottom of the file: "export ROS_MASTER_URI=http://172.24.1.72:11311" and "export ROS_HOSTNAME=172.24.1.72" (where again, 172.24.1.72 is the laptop's IP address)
- - $ source ~/.bashrc
- To test that everything works:
- - [Laptop terminal 1] $ roscore
- - [Ubuntu RPI terminal] $ roslaunch hls_lfcd_lds_driver hlds_laser.launch (the LiDAR should now start spinning, it might however take a few seconds)
- - [Laptop terminal 2] $ rostopic echo /scan (a stream of scan messages should now start appearing in the terminal)
- To also visualize the LiDAR measurements on the laptop using rviz:
- - Create a directory called "rviz" in the package test_pckg:
- - - $ cd ~/Summer17/Laptop/ROS_code/catkin_ws/src/test_pckg
- - - $ mkdir rviz
- - Create a file called "basic_lidar_visualization.rviz" that is a copy of the file "hlds_laser.rviz" in https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver and place it in the rviz directory AND make it executable
- - [Laptop terminal 1] $ roscore
- - [Ubuntu RPI terminal] $ roslaunch hls_lfcd_lds_driver hlds_laser.launch
- - [Laptop terminal 2] $ rosrun rviz rviz -d /home/fregu856/Summer17/Laptop/ROS_code/catkin_ws/src/test_pckg/rviz/basic_lidar_visualization.rviz (rviz should now open and you should see some green/yellow/red lines/dots)





