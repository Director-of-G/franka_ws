#!/usr/bin/env python

import sys
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')

import rospy

from easy_handeye.handeye_server import HandeyeServer


def main():
    rospy.init_node('easy_handeye', log_level=rospy.ERROR)
    while rospy.get_time() == 0.0:
        pass

    cw = HandeyeServer()

    rospy.spin()


if __name__ == '__main__':
    main()
