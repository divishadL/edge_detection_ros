#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from edge_detection.srv import EdgeDetection, EdgeDetectionRequest

class EdgeDetectionClient:
    def __init__(self):
        # Creates service proxy and subscribes to camera image
        rospy.init_node('edge_detection_client')
        self.edge_detection_service = rospy.ServiceProxy('edge_detection_service', EdgeDetection)
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        rospy.spin()

    def image_callback(self, image_msg):
        # Calls service with image
        edge_detection_request = EdgeDetectionRequest()
        edge_detection_request.image = image_msg

        try:
            response = self.edge_detection_service(edge_detection_request)
        except rospy.ServiceException as e:
            rospy.logerr("Service call Unsuccessful")

if __name__ == '__main__':
    EdgeDetectionClient()
