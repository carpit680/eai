#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from eai.eai_pipeline import EaiPipeline


def main(args=None):
    rclpy.init(args=args)
    try:

        eai_pipeline = EaiPipeline()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(eai_pipeline)
        executor.add_node(eai_pipeline.image_subscriber)
        try:
            executor.spin()
        finally:
            eai_pipeline.listening_thread.join()
            executor.shutdown()
            eai_pipeline.destroy_node()
            eai_pipeline.image_subscriber.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()