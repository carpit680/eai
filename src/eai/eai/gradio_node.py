import rclpy
from rclpy.node import Node
import gradio as gr
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from eai_interfaces.srv import Instruction
import cv2
import base64
from eai.get_ip import get_ip

# Image to Base 64 Converter
def image_to_base64(image):
    _, buffer = cv2.imencode('.jpg', image)
    encoded_string = base64.b64encode(buffer).decode('utf-8')
    return encoded_string

class GradioClient(Node):
    def __init__(self):
        super().__init__('gradio_client_node')
        self.client = self.create_client(Instruction, 'eai_server')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('eai service not available, waiting again...')
        self.get_logger().info('connected to eai server.')
        self.req = Instruction.Request()

        self.start_gradio()

    def send_request(self, user_input, history):
        self.req.request = user_input
        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        try:
            response_msg = future.result()
            response = response_msg.response
            image = response_msg.image
            if image.height > 0:
                response_img = CvBridge().imgmsg_to_cv2(image, "rgb8")
                base64_img = image_to_base64(response_img)
                data_url = f"data:image/jpg;base64,{base64_img}"
                history.append((user_input, f"{response} ![]({data_url})"))
            else:
                history.append((user_input, response))
        except Exception as e:
            self.get_logger().info(f'Service call failed: {e}')
            response = f"Service call failed: {e}"
            history.append((user_input, response))
        return history, history, ""

    def start_gradio(self):
        with gr.Blocks(fill_height=True) as demo:
            gr.Markdown("<h1><center>eai interface</center></h1>")

            chatbot = gr.Chatbot(scale=1,label="eai chat")
            with gr.Row():
                with gr.Column(scale=12):
                    user_input = gr.Textbox(show_label=False, placeholder="type your message here...", lines=1)
                with gr.Column(scale=2, min_width=50):
                    send_button = gr.Button("send")
                
            clear_button = gr.Button("clear chat")

            def clear_chat():
                return [], []

            def gradio_callback(user_input, history=[]):
                return self.send_request(user_input, history)

            user_input.submit(gradio_callback, [user_input, chatbot], [chatbot, chatbot, user_input])
            send_button.click(gradio_callback, [user_input, chatbot], [chatbot, chatbot, user_input])
            clear_button.click(clear_chat, [], [chatbot, chatbot])

        server_port = 7860
        while rclpy.ok():
            try:
                server_name=str(get_ip())
                self.get_logger().info(f"Starting Gradio server on {server_name}:{server_port}.")
                demo.launch(server_name=server_name, server_port=server_port, share=True)
                break
            except Exception as e:
                print(f"Server port {server_port} is not available. Trying again...")
                server_port += 1

def main(args=None):
    rclpy.init(args=args)
    node = GradioClient()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
