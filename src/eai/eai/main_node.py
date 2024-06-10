#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from collections import deque
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from eai.kosmos2 import kosmos2
from eai.llama_groq import llama3_groq
from cv_bridge import CvBridge


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.callback_group = ReentrantCallbackGroup()
        self.image_pub = self.create_publisher(Image, 'captioned_image', 10)
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            1
            # callback_group=self.callback_group
        )
        self.subscription  # prevent unused variable warning
        self.image_buffer = deque(maxlen=1)
        self.bridge = CvBridge()
        self.get_logger().info("Listening for RGB images on /camera/color/image_raw")
        self.kosmos = kosmos2(debug=True)
        self.llm = llama3_groq(self.get_logger, debug=True)

        with open("/home/arpit/eai/eai/src/eai/config/env_prompt.txt", 'r') as file:
            self.env_prompt = file.read()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            
            self.image_buffer.append(cv_image)
            # processed_text, entities, captioned_image = self.kosmos.ground_frame(cv_image)
            # self.image_pub.publish(self.bridge.cv2_to_imgmsg(captioned_image, "rgb8"))

            # final_env_prompt = "The text below is the description of an environment as seen by a camera./n" + processed_text + self.env_prompt
            # env_description = self.llm.get_response(final_env_prompt)
            # env_description = env_description[env_description.find("```") + 3: env_description.find("```", env_description.find("```") + 3)]
            # self.get_logger().info("#############################")
            # self.get_logger().info(env_description)
            # self.get_logger().info("#############################")

        except Exception as e:
            self.get_logger().error(f"{e}")


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    executor = MultiThreadedExecutor()
    executor.add_node(image_subscriber)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
