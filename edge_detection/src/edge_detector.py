#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge
from edge_detection.srv import EdgeDetection, EdgeDetectionResponse
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
import numpy as np

class EdgeDetector:
    def __init__(self):

        # Creates publisher to publish edge image, points and initializes OpenCV bridge, and creates service
        # Creates subscriber to subscribe to depth image and camera info

        self.bridge = CvBridge()
        self.service = rospy.Service('edge_detection_service', EdgeDetection, self.edge_detection_service)
        self.point_cloud_pub = rospy.Publisher('edge_points', PointCloud, queue_size=10)

        self.camera_matrix = None
        self.distortion_coeffs = None
        self.base_frame_id = "camera_color_optical_frame"
        self.camera_info_received = False
        self.depth_image = None

        self.point_cloud = PointCloud()
        self.point_cloud.header.frame_id = self.base_frame_id
        

        self.camera_info_sub = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_image_callback)

    def camera_info_callback(self, camera_info):
        
        # Gets camera matrix and distortion coefficients from camera info

        if not self.camera_info_received:
            self.camera_matrix = np.array(camera_info.K).reshape((3, 3))
            self.distortion_coeffs = np.array(camera_info.D)
            self.camera_info_received = True
            self.camera_info_sub.unregister()

    def depth_image_callback(self, depth_image):
        
        # Converts depth image to NumPy array

        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')
        except Exception as e:
            rospy.logerr("Depth image conversion unsuccessful")

    def pixel_to_point(self, u, v, depth):
        
        # Converts pixel coordinates to 3D coordinates using camera matrix and depth value
        
        if self.camera_matrix is not None:
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
            
            x = (u - cx) * depth / fx
            y = (v - cy) * depth / fy
            z = depth

            return x, y, z
        else:
            return None


    def publish_edge_points(self, edge_image):
        
        # Publishes edge points as a PointCloud message after converting them to 3D coordinates

        if self.depth_image is not None and self.camera_matrix is not None:

            self.point_cloud.points = []  

            for v in range(edge_image.shape[0]):
                for u in range(edge_image.shape[1]):
                    if edge_image[v, u][1] == 255:
                        depth = self.depth_image[v, u]
                        depth /= 1000.0 # Converting depth to meters 
                        if depth > 0: 
                            x, y, z = self.pixel_to_point(u, v, depth)
                            if x is not None:
                                point = Point(x, y, z)
                                self.point_cloud.points.append(point)


            self.point_cloud.header.stamp = rospy.Time.now()
            self.point_cloud_pub.publish(self.point_cloud) # Publishes point cloud

    def detect_edges(self, image):
        
        #Detects edges in the image and returns the image with edges drawn in green
        
        #Converts the image to grayscale, blurs it, and performs Canny edge detection
        #Filters out non-grid edges and draws green lines on the opened edges

        img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        img_blur = cv2.GaussianBlur(img_gray, (3, 3), 0)

        edges = cv2.Canny(img_blur, 100, 600, 3, L2gradient=False)

        kernel = np.ones((3, 3), np.uint8)
        dilated_edges = cv2.dilate(edges, kernel, iterations=1)
        opened_edges = cv2.morphologyEx(dilated_edges, cv2.MORPH_OPEN, kernel)

        # Draws green lines on the edges
        result = img.copy()
        result[opened_edges > 0] = [0, 255, 0]

        return self.bridge.cv2_to_imgmsg(result, encoding='bgr8')

    def edge_detection_service(self, req):
        
        # Service callback which calls detect_edges and returns the edge image

        try:
            edge_image = self.detect_edges(req.image) 
            edge_numpy = self.bridge.imgmsg_to_cv2(edge_image, desired_encoding='bgr8')  # Convert Image to NumPy array
            self.publish_edge_points(edge_numpy)  
            return EdgeDetectionResponse(edge_image)
        except Exception as e:
            rospy.logerr("Edge detection failed: %s", str(e))
            return None

if __name__ == '__main__':
    rospy.init_node('edge_detection_node')
    edge_detector = EdgeDetector()
    rospy.spin()
