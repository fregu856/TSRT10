#!/usr/bin/env python

# This code reads from the topics /keyboard/keydown and /keyboard/keyup. On
# these topics, the ros-keyboard node publishes info about all keys that are
# either pressed (keydown) or released (keyup) when the small window that
# appears when you run "rosrun keyboard keyboard" is in focus.

# This code publishes steering commands ("Forward", "Backward", "No_throttle",
# "Right", "Left", "No_steering") on the topic /steering_cmds.

import rospy
from std_msgs.msg import String
from keyboard.msg import Key

class Cmd_reader:
    def __init__(self):
        # initialize this code as a ROS node named cmd_reader_node:
        rospy.init_node("cmd_reader_node", anonymous=True)

        # create a publisher that publishes messages of type String on the
        # topic /steering_cmds:
        self.pub = rospy.Publisher("/steering_cmds", String, queue_size=10)

        # subscribe to the topic /keyboard/keydown (all published messages on
        # this topic are of type Key), i.e., call the function keydown_callback
        # every time a new message is published:
        rospy.Subscriber("/keyboard/keydown", Key, self.keydown_callback)

        # subscribe to the topic /keyboard/keyup (all published messages on
        # this topic are of type Key), i.e., call the function keyup_callback
        # every time a new message is published:
        rospy.Subscriber("/keyboard/keyup", Key, self.keyup_callback)

        # keep python from exiting until this ROS node is stopped:
        rospy.spin()

    # define the callback function for the /keyboard/keydown subscriber:
    def keydown_callback(self, msg_obj):
        # get the code of the key that was pressed down:
        key_code = msg_obj.code

        if key_code in [119, 273]: # (119 = w, 273 = up arrow)
            msg = "Forward"
            self.pub.publish(msg)

        elif key_code in [100, 275]: # (100 = d, 275 = right arrow)
            msg = "Right"
            self.pub.publish(msg)

        elif key_code in [115, 274]: # (115 = s, 274 = down arrow)
            msg = "Backward"
            self.pub.publish(msg)

        elif key_code in [97, 276]: # (97 = a, 276 = left arrow)
            msg = "Left"
            self.pub.publish(msg)

    # define the callback function for the /keyboard/keyup subscriber:
    def keyup_callback(self, msg_obj):
        # get the code of the key that was released:
        key_code = msg_obj.code

        if key_code in [119, 115, 273, 274]: # (119 = w, 115 = s)
            msg = "No_throttle"
            self.pub.publish(msg)
            #print "No_throttle"

        elif key_code in [100, 97, 275, 276]: # (100 = d, 97 = a)
            msg = "No_steering"
            self.pub.publish(msg)
            #print "No_steering"

if __name__ == "__main__":
    # create an instance of the Cmd_reader class (this will run
    # its __init__ function):
    cmd_reader = Cmd_reader()
