import rclpy
from rclpy.node import Node
from eai_interfaces.srv import Instruction

class InstructionServer(Node):
    def __init__(self):
        super().__init__('instruction_server')
        self.srv = self.create_service(Instruction, 'process_instruction', self.process_instruction_callback)

    def process_instruction_callback(self, request, response):
        self.get_logger().info(f"Received instruction: {request.request}")
        response.response = f"Processed: {request.request}"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = InstructionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
