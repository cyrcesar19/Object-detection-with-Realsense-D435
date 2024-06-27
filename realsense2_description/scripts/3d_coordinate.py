#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
import sensor_msgs
from cv_bridge import CvBridge
import cv2
import pcl
from pcl import PointCloud
import pcl.pcl_visualization
import numpy as np

bridge = CvBridge()

def image_callback(data):
    try:
        # Convertir l'image ROS en image OpenCV
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

        # Utiliser un classificateur en cascade pour la détection d'objets (par exemple, visages)
        #face_cascade = cv2.CascadeClassifier('/home/cyr-cesar/ur5_project/realsense_ws/src/realsense2_description/scripts/coco.names')
        #gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #faces = face_cascade.detectMultiScale(gray_image, scaleFactor=1.3, minNeighbors=5)

        # Dessiner des rectangles autour des objets détectés
        #for (x, y, w, h) in faces:
        #    cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)

        # Afficher l'image avec les rectangles
        #cv2.imshow('Object Detection', cv_image)
        #cv2.waitKey(1)

    except Exception as e:
        print(e)

def pc_callback(data):
    try:
        point_cloud = np.array(data)
        gen = pc2.read_points(data, skip_nans=True)
        int_data = list(gen)

        #points = [int(digit) for digit in str(point_cloud[0])]
        point_cloud = np.array_split(int_data, 3)
        
        points = point_cloud[0]
        point = points[0]

        x = point[2]
        y = point[0]*(-0.1)

        coordinate = [x, y, 0.03589510200014]
        if coordinate == [5.800528526306152, 0.5329689502716065, 0.03589510200014]:
            print("any object detected")
        else:
            print('object detected')
            print(coordinate)
        
    except Exception as e:
        print(e)

if __name__ == '__main__':
    rospy.init_node('object_position_calculator', anonymous=True)
    # Subscriber pour la capture d'image
    image_sub = rospy.Subscriber('/camera/color/image_raw', Image, image_callback)

    # Subscriber pour la capture du nuage de points
    pc_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, pc_callback)
    rospy.spin()