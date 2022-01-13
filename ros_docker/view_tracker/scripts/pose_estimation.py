#!/usr/bin/env python3

from imutils import face_utils
import imutils
import dlib
import cv2
import numpy as np
from math import *
import rospy
import copy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

class FaceDetector():

    def __init__(self):
        self.detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor("/root/catkin_ws/src/ros_docker/view_tracker/src/shape_predictor_68_face_landmarks.dat")
        rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        self.pub_rgb = rospy.Publisher("face_result", Image, queue_size=10)
        self.pub_point = rospy.Publisher("cam_position", Point, queue_size=10)
        self.size = None
        self.bgr_image = None
        self.depth_image = None
        self.img = None
        self.bridge = CvBridge()
        self.image_points = np.array([[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]], dtype="double")
        self.model_points = np.array([
                                    (0.0, 0.0, 0.0),        
                                    (0,0 -270.0, -90.0),    
                                    (-144.0, 105.0, -90.0), 
                                    (144.0, 105.0, -90.0),  
                                    (-99.0, -99.0, -45.0),  
                                    (99.0, -99.0, -45.0)    
                                    ])

        self.dist_coeffs = np.zeros((4,1))

        self.rotation_vector = np.ndarray((3,1), dtype=np.float64)
        self.translation_vector = np.ndarray((3,1), dtype=np.float64)
        self.shape = None

    def rgb_callback(self, data):
        cv_array = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.bgr_image = cv_array
        if self.size is None:
            self.size = self.bgr_image.shape
            self.focal_length = self.size[1]
            self.center = (self.size[1]/2, self.size[0]/2)
            self.camera_matrix = np.array(
                                        [[self.focal_length, 0, self.center[0]],
                                        [0, self.focal_length, self.center[1]],
                                        [0, 0, 1]], dtype = "double"
                                        )
    def depth_callback(self, data):
        cv_array = self.bridge.imgmsg_to_cv2(data, "passthrough")
        self.depth_image = cv_array

    def detect_face(self):
        self.img = copy.deepcopy(self.bgr_image)
        self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        self.rects = self.detector(self.gray, 0)
        for rect in self.rects:
            self.shape = self.predictor(self.gray, rect)
            self.shape = face_utils.shape_to_np(self.shape)
            self.image_points = np.array([
                                        self.shape[33], 
                                        self.shape[8],  
                                        self.shape[45], 
                                        self.shape[36], 
                                        self.shape[54], 
                                        self.shape[48]  
                                    ], dtype="double")
                                    
            (self.success, self.rotation_vector, self.translation_vector) = cv2.solvePnP(self.model_points, self.image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

    def get_radians(self):
        return self.rotation_vector

    def draw_face_contour(self):
        for (x, y) in self.shape:
            cv2.circle(self.img, (x, y), 1, (0, 0, 255), -1)
        for p in self.image_points:
            cv2.circle(self.img, (int(p[0]), int(p[1])), 3, (0,0,255), -1)

    def draw_face_viewline(self):
        (self.nose_end_point2D, self.jacobian) = cv2.projectPoints(np.array([(0.0, 0.0, 1000.0)]), self.rotation_vector, self.translation_vector, self.camera_matrix, self.dist_coeffs)
        p1 = ( int(self.image_points[0][0]), int(self.image_points[0][1]))
        p2 = ( int(self.nose_end_point2D[0][0][0]), int(self.nose_end_point2D[0][0][1]))
        cv2.line(self.img, p1, p2, (255,0,0), 2)

    def draw_faceparts(self):
        colors = [(19, 199, 109), (79, 76, 240), (230, 159, 23),
			        (168, 100, 168), (158, 163, 32),
			        (163, 38, 32), (180, 42, 220), (180, 42, 220)]
        self.img = face_utils.visualize_facial_landmarks(self.img, self.shape, colors)

    def show(self):
        rgb_ros = self.bridge.cv2_to_imgmsg(self.img, "bgr8")
        self.pub_rgb.publish(rgb_ros)

def main():
    rospy.init_node("pose_estimation")

    myCap = FaceDetector()

    while myCap.bgr_image is None or myCap.depth_image is None:
        print("waitinig for realsense")
        rospy.sleep(1)

    while not rospy.is_shutdown():
        myCap.detect_face()
        if myCap.shape is None:
            continue
        # myCap.draw_faceparts()
        angles = myCap.get_radians()
        # print(angles * 180/pi)
        # print(myCap.shape[33])
        # print(myCap.depth_image.shape)
        # print(myCap.depth_image[myCap.shape[33][1]][myCap.shape[33][0]])
        p = Point()
        p.x = myCap.shape[33][0]
        p.y = myCap.shape[33][1]
        p.z = myCap.depth_image[myCap.shape[33][1]][myCap.shape[33][0]]
        print(myCap.depth_image.shape)
        myCap.pub_point.publish(p)
        myCap.draw_face_viewline()
        myCap.draw_face_contour()
        myCap.show()

        myCap.shape = None

if __name__ == "__main__":
    main()