#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import dlib
import copy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class FaceDetection():
    def __init__(self):
        rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        self.pub_rgb = rospy.Publisher("face_result", Image, queue_size=10)
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None

    def rgb_callback(self, data):
        cv_array = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.rgb_image = cv_array
    
    def depth_callback(self, data):
        cv_array = self.bridge.imgmsg_to_cv2(data, "passthrough")
        self.depth_image = cv_array

    def shape_to_np(self, shape, dtype="int"):
        coords = np.zeros((68, 2), dtype=dtype)
        for i in range(0, 68):
            coords[i] = (shape.part(i).x, shape.part(i).y)
        return coords

    def detection(self):
        img = copy.deepcopy(self.rgb_image)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        detector = dlib.get_frontal_face_detector()
        rects = detector(gray, 1) 
        predictor = dlib.shape_predictor("/root/catkin_ws/src/ros_docker/view_tracker/src/shape_predictor_68_face_landmarks.dat")
        for (i, rect) in enumerate(rects):
            shape = predictor(gray, rect)
            shape = self.shape_to_np(shape)
            for (x, y) in shape:
                cv2.circle(img, (x, y), 2, (0, 0, 255), -1)
        rgb_ros = self.bridge.cv2_to_imgmsg(img, "rgb8")
        self.pub_rgb.publish(rgb_ros)

        
if __name__ == '__main__':
    rospy.init_node('face_detection')
    fd = FaceDetection()
    while not rospy.is_shutdown():
        if fd.rgb_image is not None and fd.depth_image is not None:
            fd.detection()