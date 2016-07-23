#!/usr/bin/env python

from __future__ import division

import sys
import random
import rospy
import moveit_commander
import angles

from quadrotor import Quadrotor

__author__ = 'Aijun Bai'


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_quadrotor', anonymous=True)

    q = Quadrotor(display=False, debug=True)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        p = q.get_current_pose()
        p.x += random.uniform(-2.5, 2.5)
        p.y += random.uniform(-2.5, 2.5)
        p.z += random.uniform(-2.5, 2.5)
        p.yaw += random.uniform(angles.d2r(-90.0), angles.d2r(90.0))
        p.z = max(p.z, 0.5)
        q.moveto_pose(p, wait=True)
        rate.sleep()

    moveit_commander.roscpp_shutdown()


if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
