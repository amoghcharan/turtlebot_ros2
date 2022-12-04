import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
import sys

import numpy as np
import cv2
from cv_bridge import CvBridge


class FindObject(Node):
    def __init__(self):
        # Create find_object Node
        super().__init__('find_object')

        self.msg = Point()

        self.cx = 0
        self.cy = 0

        # Declare Parameters of Node
        self.declare_parameter('window_name', 'Base Image')
        self.declare_parameter('show_image_bool', True)

        # Get Parameters of Node
        self.windowName = self.get_parameter('window_name').value
        self._display_image = bool(self.get_parameter('show_image_bool').value)

        # View Base Image from RasPi and move window
        # if(self._display_image):
        #     cv2.namedWindow(self.windowName, cv2.WINDOW_AUTOSIZE)
        #     cv2.moveWindow(self.windowName, 100, 100)

        # Declare node is subscribing to 'camera/image/compressed' topic
        self._camera_subscriber = self.create_subscription(CompressedImage, 'camera/image/compressed',
                                                           self._image_detection, 1)
        self._camera_subscriber

        # Declare node is publishing to 'coordinate' topic
        self._coord_publisher = self.create_publisher(Point, '/coord', 1)
        self._coord_publisher

        # Declare node is publishing to 'debug' topic
        self._debug_publisher = self.create_publisher(CompressedImage, '/debug', 1)
        self._debug_publisher

    # Callback function for subscriber
    def _image_detection(self, CompressedImage):
        print('callback')
        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")

        pt = Point()
        pt.x = 0.0

        # Convert current frame to HSV format
        hsv1 = cv2.cvtColor(self._imgBGR, cv2.COLOR_BGR2HSV)
        # Set lower HSV threshold range
        mask1 = cv2.inRange(hsv1, (25, 25, 25), (30, 255, 255))
        # Set upper HSV threshold range
        mask2 = cv2.inRange(hsv1, (150, 100, 20), (180, 255, 255))
        # Combine HSV threshold ranges
        final = mask2  # + mask1
        # Set mask onto original frame
        bit = cv2.bitwise_and(self._imgBGR, self._imgBGR, mask=final)
        # Add Gaussian Blur
        blur = cv2.GaussianBlur(bit, (7, 7), cv2.BORDER_DEFAULT)
        # Covert HSV frame to Grayscale
        frame2 = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

        # Insert image-detection of ball
        circles = cv2.HoughCircles(frame2, cv2.HOUGH_GRADIENT, 1, 100,
                                   param1=15,
                                   param2=50,
                                   minRadius=10,
                                   maxRadius=0)
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for i in circles:
                cv2.circle(self._imgBGR, (i[0], i[1]), i[2], (0, 255, 0), 3)
                self.x1 = circles[0][0]
                self.y1 = circles[0][1]
                self.r1 = circles[0][2]
                cv2.putText(self._imgBGR, 'X = ' + str(self.x1), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(self._imgBGR, 'Y = ' + str(self.y1), (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(self._imgBGR, 'R = ' + str(self.r1), (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)


                self.cx = float(self.x1)
                self.cy = float(self.y1)
                self.cr = float(self.r1)

                print('if')

                pt.x = self.cx
                self._coord_publisher.publish(pt)
        else:
            pt.x = 0.0000
            print('else')
            self._coord_publisher.publish(pt)


def main():
    rclpy.init()
    obj = FindObject()

    while rclpy.ok():
        rclpy.spin_once(obj)  # Trigger callback processing.

    obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()