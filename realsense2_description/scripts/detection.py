#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

bridge = CvBridge()
obj_coordinate = {'x':0.0, 'y':0.0, 'z':0.18}


def object_detection(img):
    # Convert to grayscale and threshold
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(imgray, 127, 255, 0)

    # Find and draw contours
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    # Save the coordinate 
    for cnt in contours:
        (x,y),radius = cv2.minEnclosingCircle(cnt)
        center = (int(x),int(y))
        radius = int(radius)
        #object detection
        if x != 399 and x != 171 and x != 628 and x != 399.5:
            cv2.drawContours(img, contours, -1, (255, 255, 255),1)

            print('Contour: centre {},{}, radius {}'.format(x,y,radius))
            print('center =', center)
            obj_detected = True
            print("object detected")

            #saving coordinates
            obj_coordinate['x'] = x
            obj_coordinate['y'] = y
        else:
            print("any object has been detected")

    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

def image_callback(msg):
    cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    object_detection(cv2_img)

def main():
    rospy.init_node('image_listener')
    image_topic = "/camera/color/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()