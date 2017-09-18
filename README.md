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
- [Laptop terminal 1] $ roslaunch sweep_ros sweep2scan.launch
- [Laptop terminal 2] $ roslaunch test_pckg test_Hector.launch





