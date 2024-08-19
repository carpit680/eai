import gradio as gr

# Dummy chatbot function
def chatbot_response(user_input, history=[]):
    response = f"echo: {user_input}"
    history.append((user_input, response))
    return history, history

# Create a Gradio interface
with gr.Blocks() as demo:
    gr.Markdown("<h1><center>eai interface</center></h1>")
    
    chatbot = gr.Chatbot(label="eai")
    with gr.Row():
        with gr.Column(scale=12):
            user_input = gr.Textbox(show_label=False, placeholder="type your message here...", lines=1)
        with gr.Column(scale=1, min_width=50):
            send_button = gr.Button("send")
        
    clear_button = gr.Button("clear chat")

    def clear_chat():
        return [], []

    user_input.submit(chatbot_response, [user_input, chatbot], [chatbot, chatbot])
    send_button.click(chatbot_response, [user_input, chatbot], [chatbot, chatbot])
    clear_button.click(clear_chat, [], [chatbot, chatbot])

# Launch the interface
demo.launch(server_name="192.168.1.16", server_port=7860, share=False)
