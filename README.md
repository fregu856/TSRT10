# TSRT10

******
Basic setup (this guide assumes that you're running Ubuntu (or possibly some different kind of Linux distribution)):

- Install ROS (http://wiki.ros.org/lunar/Installation/Ubuntu):
- - $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
- - $ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
- - $ sudo apt-get update
- - $ sudo apt-get install ros-lunar-desktop (for RPIs, I'd probably recommend 'sudo apt-get install ros-lunar-ros-base' instead)
- - $ sudo rosdep init
- - $ rosdep update
- - $ echo "source /opt/ros/lunar/setup.bash" >> ~/.bashrc
- - $ source ~/.bashrc

- Install catkin (http://wiki.ros.org/catkin) (might have been installed automatically in the above step):
- - $ sudo apt-get install ros-lunar-catkin

- Create a directory called TSRT10 in your home directory:
- - $ cd --
- - $ mkdir TSRT10

- Create a catkin workspace:
- - $ cd ~/TSRT10
- - $ mkdir catkin_ws
- - $ cd catkin_ws
- - $ mkdir src
- - $ catkin_make
- - $ sudo nano ~/.bashrc
- - Add the below line to the bottom of this file (Ctrl+Shift+v to paste the line, Ctrl+x - y - Enter to save the file)
```
source ~/TSRT10/catkin_ws/devel/setup.bash
```  
- - $ source ~/.bashrc

- Create and build a package (called test_package) in the catkin workspace:
- - $ cd ~/TSRT10/catkin_ws/src
- - $ catkin_create_pkg test_package std_msgs roscpp rospy
- - $ cd ~/TSRT10/catkin_ws
- - $ catkin_make

- - Create a python_scripts directory in the package (it's in this directory we will place all python ROS scripts):
- - - $ cd ~/TSRT10/catkin_ws/src/test_package
- - - $ mkdir python_scripts
- - Every python script that one writes and places in python_scripts (e.g. test.py) must be made executable:
- - - $ cd ~/TSRT10/catkin_ws/src/test_package/python_scripts 
- - - $ chmod a+x test.py
- - You should always also build the package (this is sometimes (quite often) needed even for python scripts since we use C++ messages):
- - - $ cd ~/TSRT10/catkin_ws
- - - $ catkin_make

- Create a python script called publisher.py containing the code below and place it in python_scripts:
```
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publisher():
    # create a publisher that publishes messages of type String on the
    # topic /test_topic:
    pub = rospy.Publisher("/test_topic", String, queue_size=10)

    # initialize this code as a ROS node named publisher_node:
    rospy.init_node("publisher_node", anonymous=True)

    # specify the desired loop frequency in Hz:
    rate = rospy.Rate(1)

    while not rospy.is_shutdown(): # (while the ROS node is still active:)
        # specify the string that you want to send/publish:
        message = "Hej!"

        # publish the message (on the specified topic, i.e., /test_topic):
        pub.publish(message)

        # sleep to get a loop frequency of 1 Hz:
        rate.sleep()

if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
```

- Create a python script called subscriber.py containing the code below and place it in python_scripts:
```
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def subscriber():
    # initialize this code as a ROS node named subscriber_node:
    rospy.init_node("subscriber_node", anonymous=True)

    # subscribe to the topic /test_topic. That is, read every message (of type
    # String) that is published on /test_topic. We will do this by calling the
    # function callback_function everytime a new message is published:
    rospy.Subscriber("/test_topic", String, callback_function)

    # keep python from exiting until this ROS node is stopped:
    rospy.spin()

# define the callback function for the /test_topic subscriber:
def callback_function(message_object):
    # get the actual message String that was published:
    received_message = message_object.data

    # print the received String:
    print received_message

if __name__ == "__main__":
    subscriber()
```

- Make the files executable:
- - $ cd ~/TSRT10/catkin_ws/src/test_package/python_scripts 
- - $ chmod a+x publisher.py
- - $ chmod a+x subscriber.py

- Build the package:
- - $ cd ~/TSRT10/catkin_ws
- - $ catkin_make


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

- On computer2 (Slave):
- - Find computer2's IP address by running $ ifconfig and look for "inet addr" below "wlan0, we call this yyy.yy.y.yy
- - $ sudo nano ~/.bashrc
- - Add the following two lines to the bottom of the file: "export ROS_MASTER_URI=http://xxx.xx.x.xx:11311" and "export ROS_HOSTNAME=yyy.yy.y.yy"
- - $ source ~/.bashrc

- [computer1 terminal 1] $ roscore
- [computer1 terminal 2] $ rosrun test_pckg test.py
- [computer1 terminal 3] $ rostopic echo /test_topic (now you should start receiving messages)
- [computer2] $ rostopic echo /test_topic (now you should start receiving messages)

****

To reset the ROS IP addresses (so that you can always run ROS code locally):
- $ sudo nano ~/.bashrc
- Modify the ROS_MASTER_URI and ROS_HOSTNAME lines so that they say:
- - export ROS_MASTER_URI=http://localhost:11311
- - export ROS_HOSTNAME=localhost
- $ source ~/.bashrc





