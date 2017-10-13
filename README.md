# TSRT10

******

# Basic setup: 
(this guide assumes that you're running Ubuntu (or possibly some different kind of Linux distribution))

- Install ROS kinetic (lunar is newer but not all packages seem to be available for lunar) (http://wiki.ros.org/kinetic/Installation/Ubuntu):
- - $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
- - $ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
- - $ sudo apt-get update
- - $ sudo apt-get install ros-kinetic-desktop (for RPIs, I'd probably recommend 'sudo apt-get install ros-kinetic-ros-base' instead)
- - $ sudo rosdep init
- - $ rosdep update
- - $ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
- - $ source ~/.bashrc

- Install catkin (http://wiki.ros.org/catkin) (might have been installed automatically in the above step):
- - $ sudo apt-get install ros-kinetic-catkin

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

- Start the ROS master:
- - $ roscore

- Run publisher.py to publish messages to /test_topic:
- - Open a new terminal window
- - $ rosrun test_package publisher.py

- Test that publisher.py is running:
- - Open a new terminal window
- - $ rostopic echo /test_topic ('data: "Hej!"' should now be printed to the terminal with 1 Hz)

- Run subscriber.py to read messages from /test_topic:
- - Open a new terminal window
- - $ rosrun test_package subscriber.py ('Hej!' should now be printed to the terminal with 1 Hz)








****
# Test communication between two computers: 
(this assumes that one has followed the above steps on both computers)

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
- [computer1 terminal 2] $ rosrun test_package publisher.py
- [computer1 terminal 3] $ rostopic echo /test_topic (now you should start receiving messages)
- [computer2 terminal 1] $ rostopic echo /test_topic (now you should start receiving messages)
- [computer2 terminal 2] $ rosrun test_package subscriber.py (now you should start receiving messages)






****
# To reset the ROS IP addresses: 
(so that you can always run ROS code locally)

- $ sudo nano ~/.bashrc
- Modify the ROS_MASTER_URI and ROS_HOSTNAME lines so that they say:
- - export ROS_MASTER_URI=http://localhost:11311
- - export ROS_HOSTNAME=localhost
- $ source ~/.bashrc








******
# Setup of LIDAR: 
(scanse sweep, https://github.com/scanse/sweep-ros):


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
- - $ sudo apt-get install ros-kinetic-pcl-conversions
 
- Clone the sweep-ros repo:
- - $ cd ~/TSRT10/catkin_ws/src
- - $ git clone https://github.com/scanse/sweep-ros.git
- - $ cd ~/TSRT10/catkin_ws
- - $ catkin_make

- Make sure that your username is in the dialout group in /etc/group (otherwise you won't have permission to open /dev/ttyUSB0):
- - $ sudo nano /etc/group
- - In my case (my username is 'fregu856'), I hade to change the line "dialout:x:20:" to "dialout:x:20:fregu856"
- - Restart the computer ($ sudo reboot)

- Now, you should be able to launch sweep.launch (after connecting the LIDAR to the computer):
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







***
# Setup the balrog package:

- $ cd ~/TSRT10/catkin_ws/src
- $ catkin_create_pkg balrog std_msgs roscpp rospy
- $ cd ~/TSRT10/catkin_ws
- $ catkin_make

- Create a python_scripts directory in the package (it's in this directory we will place all python ROS scripts):
- - $ cd ~/TSRT10/catkin_ws/src/balrog
- - $ mkdir python_scripts
- Every python script that one writes and places in python_scripts (e.g. test.py) must be made executable:
- - $ cd ~/TSRT10/catkin_ws/src/balrog/python_scripts 
- - $ chmod a+x test.py
- You should always also build the package (this is sometimes (quite often) needed even for python scripts since we use C++ messages):
- - $ cd ~/TSRT10/catkin_ws
- - $ catkin_make

- Create a launch directory in ~/TSRT10/catkin_ws/src/balrog
- Place the file lidar.launch in ~/TSRT10/catkin_ws/src/balrog/launch
- Place the file lidar_and_viz.launch in ~/TSRT10/catkin_ws/src/balrog/launch
- Create an rviz directory in ~/TSRT10/catkin_ws/src/balrog
- Place the file lidar.rviz in ~/TSRT10/catkin_ws/src/balrog/rviz

- To run the LIDAR without any visualization:
- - $ roslaunch balrog lidar.launch

- To run the LIDAR together with a visualization:
- - $ roslaunch balrog lidar_and_viz.launch











****
# Run Hector SLAM 
(only using the LiDAR scans, no odometry)

- Useful links: http://wiki.ros.org/hector_slam/Tutorials/SettingUpForYourRobot, https://github.com/tu-darmstadt-ros-pkg/hector_slam/blob/catkin/hector_slam_launch/rviz_cfg/mapping_demo.rviz, https://github.com/tu-darmstadt-ros-pkg/hector_slam/blob/catkin/hector_slam_launch/launch/mapping_box.launch
- $ cd ~/TSRT10/catkin_ws/src
- $ git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
- $ cd ~/TSRT10/catkin_ws
- $ catkin_make

- Place the file hector.launch (which is written based on the above links) and place it in ~/TSRT10/catkin_ws/src/balrog/launch
- Place the file hector.rviz (which is based on mapping_demo.rviz linked above) and place it in ~/TSRT10/catkin_ws/src/balrog/rviz
- $ cd ~/TSRT10/catkin_ws
- $ catkin_make
- [terminal 1] $ roslaunch balrog lidar.launch
- [terminal 2] $ roslaunch balrog hector.launch










*****
# Balrog simulator:

- $ sudo apt-get install ros-kinetic-turtlebot
- $ sudo apt-get install ros-kinetic-turtlebot-apps
- $ sudo apt-get install ros-kinetic-turtlebot-interactions
- $ sudo apt-get install ros-kinetic-turtlebot-simulator

- Reset the ROS IP addresses:
- - Make sure that the lines with ROS_MASTER_URI and ROS_HOSTNAME in ~/.bashrc looks as below:
```
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
```
- - $ source ~/.bashrc

- Create a package called balrog_sim in the workspace:
- - $ cd ~/TSRT10/catkin_ws/src  
- - $ catkin_create_pkg balrog_sim std_msgs roscpp rospy  
- - $ cd ~/TSRT10/catkin_ws  
- - $ catkin_make   

- Create a python_scripts directory in the package:
- - $ cd ~/TSRT10/catkin_ws/src/balrog_sim  
- - $ mkdir python_scripts  

- Every python script that you write and place in python_scripts (e.g. test.py) must be made executable:
- - $ chmod a+x test.py    
- You should also always build the package:
- - $ cd ~/TSRT10/catkin_ws  
- - $ catkin_make  

- Clone the repo needed to simulate turtlebot in Gazebo:
- - $ cd ~/TSRT10/catkin_ws/src  
- - $ git clone https://github.com/StanfordASL/asl_turtlebot.git  
- - $ cd ~/TSRT10/catkin_ws    
- - $ catkin_make  
- - Add "export GAZEBO_MODEL_PATH=~/TSRT10/catkin_ws/src/asl_turtlebot/models" to the bottom of ~/.bashrc ($ sudo nano ~/.bashrc to edit)
- - $ source ~/.bashrc

- Test the simulator:
- - $ roslaunch asl_turtlebot turtlebot_sim.launch  
- - If everything works as expected, Gazebo should launch and you should see this:
![alt text](https://lh3.googleusercontent.com/QOwaV2rTqrQWygOWAjcyfVmfAgBeBoOIuDD2pVh_V-oSXmR4psYg7j9c65WXFMbY1sENTiC1QcvwNVarQ1PMG_wOWtXRDyU_tHX5OggibJSWAhiZBtfspHJDHhqyiDEAIhG8BqOO77lZ0aUBStz96QbcENAdTfnC-Uas2-U_yRkigpAPd3JlByerDjuLwzrsBipIuq3wqzHhBhAGRaBTI_drODZvH-Q1rMUmWfV2ei854h17PMvKH1J6Nb4Gr3-WqMw8EbZvAwg4AhrjP-m_qmnvKrAKktPdIkGzPpQ2KtZuTMETaI2X66kX06Xc5nORQHrR252jz0nTSArxF9doQnwBaWFcRALz7cd5f3dWy3-b89c-1irwyEYBfxqab-C0QgSY9CBBJcT8kHbIkeqtelWHs5SwY3BuKd2zaGnWpmFT_XADFgEdNM6fNTo9L_i9z-mtoJPMJaANdni7TriLSH-0Vm7mzplYFRTZhfGxZykfQyJ1-6jHOuI1lTCnSVypnPc-M3xn52Y2RP6gPiyp5yLSVGVai1iEtbgpefooobPvExa73ZhJRiK2Em3_5N4PrvMG0v0q-hic4dJl6icOIlku7skioTTk3VUFL3e2iwtnOJVYr8fQ4z2dFQo0ijgcharzBlrFibKv2evsjwu6nYiK8O2IffRfUIA=w700-h393-no)
- - If this doesn't work, open another terminal and do this FIRST:
- - - $ rosrun gazebo_ros gzclient

- Run a test controller to check that everything is working:  
- - Place "test_controller.py" in ~/TSRT10/catkin_ws/src/balrog_sim/python_scripts, make it executable and build the package  
- - [terminal 1] $ roslaunch asl_turtlebot turtlebot_sim.launch  
- - [terminal 2] $ rosrun balrog_sim test_controller.py  
- - The robot should now move to the coordinate (1, 1)

- Start a simulation of the robot in the TSRT10 test area:
- - Create a launch directory in ~/TSRT10/catkin_ws/src/balrog_sim
- - Place the file test_area.launch in ~/TSRT10/catkin_ws/src/balrog_sim/launch
- - Create a world directory in ~/TSRT10/catkin_ws/src/balrog_sim
- - Place the file test_area.world in ~/TSRT10/catkin_ws/src/balrog_sim/world
- - $ cd ~/TSRT10/catkin_ws 
- - $ catkin_make
- - $ roslaunch balrog_sim test_area.launch
- - If everything works as expected, Gazebo will launch and you will see this:
![alt text](https://lh3.googleusercontent.com/ouaKpsPjT0b60fiYlbAbSutpYMIS16xXIKH_00L-WMpCKFYQFMVC9m2ygWSxQ_JNuJPOocEQBfmyCeNKPI7LbxcvY2w2mwmOQluUBOgyZHGxGNGi5bYW0F__B8yiq8PYW8s2exJyf4-zf-Ps-bbLvGlgqUJIWHTiD5PgmuhXR4t_FBpjN8o5heY6EQKUnVNKXTBKCnRNUhJxmFK6MqtxVLbqBNLjft340DvSHL1AzrG9ou4j8fP1vV58jCJhbDw7swVhHbKP1LFJNgF1tCOQWb03PVuyexQmg_9Jh_m2T14ZnSbNDRj-ebI_K2OfRPYthwb-Lyw4gtCk8Ppk2IxwppFyzdQLYOGHD1JWJLsNwqStT_gFIYoklqYn7c465bQwEJidoHnMKH9ojsGxG6ohoMYedSz9RzJ1tvWiZM-HTXeC9PRXef9ah2kdXBk4-r-Pcl0nJby8J-n1qIpYnyIJDkBMltt1k8ANwWLR6LX8YoRgfWB5_kQkoXXNFtOMnrUvHTKsGf26kpC0_8spnth8i3deWhcQdqVuF2NvoWobk1Rqa9vvAyOTjl4a1A0Pp5y9-2dbXRaiDk0NWVO9eOjs-fkrEENmvZfkN4xNkKbAopmUh9pj4Yqk7IHNi_pkbeaX1sxhhduoxSFeoZL1Ci2zssU-GuUPDfJxK1I=w700-h393-no)

- Manually control the robot using WASD:
- - $ cd ~/TSRT10/catkin_ws/src
- - $ git clone https://github.com/lrse/ros-keyboard.git
- - $ cd ~/TSRT10/catkin_ws/
- - $ catkin_make
- - Place the file cmd_reader.py in ~/TSRT10/catkin_ws/src/balrog_sim/python_scripts and make it executable
- - Place the file manual_controller.py in ~/TSRT10/catkin_ws/src/balrog_sim/python_scripts and make it executable
- - $ cd ~/TSRT10/catkin_ws/
- - $ catkin_make
- - [terminal 1] $ roslaunch balrog_sim test_area.launch
- - [terminal 2] $ rosrun keyboard keyboard (a small window should appear)
- - [terminal 3] $ rosrun balrog_sim cmd_reader.py
- - [terminal 4] $ rosrun balrog_sim manual_controller.py
- - If you now click in the small window that appeared, you can use WASD to control the robot
- Equivalent but more convenient way to run the four commands above:
- - [terminal 1] $ roslaunch balrog_sim test_area.launch
- - [terminal 2] $ roslaunch balrog_sim manual_control.launch (a small window should appear)
- - If you now click in the small window that appeared, you can use WASD to control the robot
- Or, just do (after having placed test_area_manual_control.launch in the launch directory):
- - $ roslaunch balrog_sim test_area_manual_control.launch

### SLAM:

- Install slam_gmapping:
- - $ cd ~/TSRT10/catkin_ws/src
- - $ git clone https://github.com/ros-perception/slam_gmapping.git
- - $ cd ~/TSRT10/catkin_ws/
- - $ catkin_make

- Place the file gmapping.rviz in ~/TSRT10/catkin_ws/src/balrog_sim/rviz
- Place the file gmapping.launch in ~/TSRT10/catkin_ws/src/balrog_sim/launch
- Place the file asl_turtlebot_custom_lidar.urdf.xacro in ~/TSRT10/catkin_ws/src/balrog_sim/robots (this file contains the specs for the simulated LIDAR)
- Replace balrog_sim/world/test_area.world with the version found in the repo

- $ roslaunch balrog_sim gmapping.launch 

- Or, if you'd rather use Hector:
- - Place hector.launch (from balrog_sim/launch in this repo) in balrog_sim/launch
- - $ roslaunch balrog_sim hector.launch 



# Odom:

- $ rosrun balrog read_encoders.py
- $ rosrun balrog slam_odom.py 


# GMapping on Balrog:

- $ roslaunch balrog lidar.launch
- $ rosrun balrog read_encoders.py
- $ rosrun balrog slam_odom.py 
- $ roslaunch balrog gmapping.launch 

# OpenKarto in simulation:

- Install * some package that I don't remember right know *
- Install * some other package that I don't remember right know *
- Copy ~/TSRT10/catkin_ws/src/navigation_2d to your ~/TSRT10/catkin_ws/src
- $ cd ~/TSRT10/catkin_ws
- $ catkin_make
- $ roslaunch balrog_sim OpenKarto.launch

# OpenKarto on Balrog:

- $ roslaunch balrog lidar.launch
- $ rosrun balrog read_encoders.py
- $ rosrun balrog slam_odom.py 
- $ roslaunch balrog OpenKarto.launch 

# Matlab:

To launch Matlab:
- $ cd /usr/local/MATLAB/R2017a/bin
- $ ./matlab

Create alias so that one can launch Matlab by running just "matlab" in any directory (needed to compile code for Balrog):
- $ cd /usr/local/bin/
- $ sudo ln -s /usr/local/MATLAB/R2017a/bin/matlab matlab

To be able to generate code from Matlab:
- $ sudo chown -R fregu856:fregu856 /usr/local/MATLAB (to change owner of this folder from root to fregu856, this is needed to be able to install toolboxes in this folder)
- Install MATLAB Coder (go to "Get more apps" in Matlab and search for "Coder")

I wasn't able compile the ComputerVision code (which isn't needed anyway), to fix this:
- Remove "$(RPI_CODE)/ComputerVision" in Makefile

Had to restart my laptop once because for some reason it didn't seem to use the correct compiler (eventhough it looked like it did).  


If main doesnt start automatically, try to transmit the file and then SSH into the RPI and run ./main manually.

# Install OpenKarto:
- Install the ROS navigation stack in order to get the costmap_2D package:
- - $ sudo apt-get install ros-kinetic-navigation
- Install SuiteSparse:
- - $ sudo apt-get install libsuitesparse-dev
- $ cd ~/TSRT10/catkin_ws/src
- $ git clone https://github.com/skasperski/navigation_2d.git
- $ cd ~/TSRT10/catkin_ws
- $ catkin_make

# Balrog-PC:

- $ sudo apt install git
- I followed the "Basic setup" steps above
- I followed the "Setup of LIDAR" steps above
- I followed the "Setup the balrog package" steps above
