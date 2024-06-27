#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as n
import time
import sys
import math
import moveit_commander
import actionlib
import moveit_msgs.msg
import control_msgs.msg
from copy import deepcopy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from std_msgs.msg import String
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import pyrealsense2 as pr2 

bridge = CvBridge()

x = 0.0
y = 0.0
z = 0.035895
depth = 1

def object_detection(img):
    global x
    global y
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
        if x == 319 or x == 319.5 or x == 399.5 or center[0] == 319.5 or center[0] == 319:
            print("any object has been detected")
        else:
            cv2.drawContours(img, contours, -1, (255, 255, 255),1)

            #print('Contour: centre {},{}, radius {}'.format(x,y,radius))
            #print('center =', center)
            obj_detected = True
            #print("object detected")

            time.sleep(15) #to let robot complete the motion
    print(img.size)

    cv2.imshow("Detection 2D coordinate", img)
    cv2.waitKey(3)

def image_callback(msg):
    cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    object_detection(cv2_img)

def depth_calculate(Camera_depth):
    global x
    global y
    global depth
    
    cv_image = bridge.imgmsg_to_cv2(Camera_depth, Camera_depth.encoding)
    depth = pr2.cv2_img.get_distance(int(x), int(y))
    a = 0
    for i in range(1200):
        depth = cv_image[a, i]
        if depth == 0 :
            a = a+1 
        else:
            break
    depth = depth * 0.001


def callback(cameraInfo):
    global x
    global y
    global depth

    intrinsics = pr2.intrinsics()

    intrinsics.width = cameraInfo.width
    intrinsics.height = cameraInfo.height
    intrinsics.ppx = cameraInfo.K[2]
    intrinsics.ppy = cameraInfo.K[5]
    intrinsics.fx = cameraInfo.K[0]
    intrinsics.fy = cameraInfo.K[4]


    #_intrinsics.model = cameraInfo.distortion_model
    intrinsics.model  = pr2.distortion.none
    for i in cameraInfo.D:
        intrinsics.coeffs = [i]
    
    result = pr2.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)
    a = -result[0] #* 1.806
    b = -result[1]
    c = result[2]

    final_result = [a, b, z]
    print(final_result)



def main():
    rospy.init_node('image_listener')
    image_topic = "/camera/color/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback)
    intrinsic = '/camera/depth/camera_info'
    rospy.Subscriber(intrinsic, CameraInfo, callback, queue_size = 1)
    image_topic = "/camera/depth/image_raw"
    rospy.Subscriber(image_topic, Image, depth_calculate)
    rospy.spin()

    

if __name__ == '__main__':
    main()
    rospy.spin()