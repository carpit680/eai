#!/usr/bin/env python3
"""
IMAGE SUBSCRIBER

This file contains the ImageSubscriber class, which subscribes to the '/camera/color/image_raw' topic and stores the latest image in a deque.

The ImageSubscriber class contains the following methods:

    __init__(self)

    _image_callback(self, msg)

    get_latest_frame(self)
"""
from threading import Lock
from collections import deque

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ImageSubscriber(Node):
    def __init__(self):
        """
        Initializes the ImageSubscriber class.

        This function is the constructor for the ImageSubscriber class. It initializes the necessary attributes and sets up the subscription to the '/camera/color/image_raw' topic.

        Parameters:
            None

        Returns:
            None
        """
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera/camera/color/image_raw',
            self._image_callback,
            10
        )

        self.image_buffer = deque(maxlen=1)
        self.bridge = CvBridge()
        self.get_logger().info("Listening for RGB images on /camera/color/image_raw")

    def _image_callback(self, msg):
        """
        Internal callback function that is triggered when a new image message is received. It saves the image in the image buffer of max length 1.

        Args:
            msg (sensor_msgs.msg.Image): The image message received.

        Returns:
            None

        Raises:
            Exception: If an error occurs while processing the image.
        """
        try:
            # print("got image")
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            self.image_buffer.append(cv_image)
        except Exception as e:
            self.get_logger().error(f"{e}")
    
    def get_latest_frame(self):
        """
        Returns the latest frame from the image buffer.

        This function acquires the image lock to ensure exclusive access to the image buffer. It then returns the first element of the image buffer, which represents the latest frame.

        Returns:
            numpy.ndarray: The latest frame from the image buffer.

        """
        if len(self.image_buffer) == 0:
            print("No image received yet")
        return self.image_buffer.popleft()
