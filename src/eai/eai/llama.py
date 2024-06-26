#!/usr/bin/env python3

import os
from groq import Groq

class llama3_groq():
    def __init__(self, debug):
        self.client = Groq(
            api_key=os.environ.get("GROQ_API_KEY"),
        )
        self.debug = debug
        self.conversation_history = []  # Initialize conversation history
        print("Llama3 on Groq initialized.")
    
    def get_response(self, content, save_history=False):
        # Add current message to conversation history
        prompt = [{"role": "user", "content": content}]
        
        # Get response from the model using conversation history as context
        chat_completion = self.client.chat.completions.create(
            messages=prompt,
            model="llama3-70b-8192",
        )

        
        if self.debug:
            print(f"RESPONSE: {chat_completion.choices[0].message.content}")
        
        if save_history:
            # Add model response to conversation history
            self.conversation_history.append({"role": "llama3", "content": chat_completion.choices[0].message.content})
            return chat_completion.choices[0].message.content, self.conversation_history

        return chat_completion.choices[0].message.content

if __name__ == "__main__":
    llm = llama3_groq(debug=True)
    text = "Hello, this is a test of the llm model."
    print(llm.get_response(text))
