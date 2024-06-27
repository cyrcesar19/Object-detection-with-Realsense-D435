#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

def image_callback(msg):
    cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    show_image(cv2_img)

def main():
    rospy.init_node('image_listener')
    image_topic = "/camera/color/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()