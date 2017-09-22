#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from keyboard.msg import Key

class Cmd_reader:
    def __init__(self):
        rospy.init_node("cmd_reader_node", anonymous=True)

        self.pub = rospy.Publisher("/steering_cmds", String, queue_size=10)

        rospy.Subscriber("/keyboard/keydown", Key, self.keydown_callback)

        rospy.Subscriber("/keyboard/keyup", Key, self.keyup_callback)

        rospy.spin()

    def keydown_callback(self, msg_obj):
        key_code = msg_obj.code

        if key_code == 119: # (w)
            msg = "Forward"
            self.pub.publish(msg)
            #print "Forward"

        elif key_code == 100: # (d)
            msg = "Right"
            self.pub.publish(msg)
            #print "Right"

        elif key_code == 115: # (s)
            msg = "Backward"
            self.pub.publish(msg)
            #print "Backward"

        elif key_code == 97: # (a)
            msg = "Left"
            self.pub.publish(msg)
            #print "Left"

    def keyup_callback(self, msg_obj):
        key_code = msg_obj.code

        if key_code in [119, 115]: # (w, s)
            msg = "No_throttle"
            self.pub.publish(msg)
            #print "No_throttle"

        elif key_code in [100, 97]: # (d, a)
            msg = "No_steering"
            self.pub.publish(msg)
            #print "No_steering"

if __name__ == "__main__":
    cmd_reader = Cmd_reader()
