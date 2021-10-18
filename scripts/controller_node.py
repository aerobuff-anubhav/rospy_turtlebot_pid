#! /usr/bin/env python

import rospy
from controller import Controller

rospy.init_node("pid_node", anonymous=True)
controller = Controller()

while not rospy.is_shutdown():
    controller.spin()