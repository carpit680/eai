#!/usr/bin/env python3

import os
from groq import Groq

class llama3_groq():
    def __init__(self, get_logger, debug):
        self.client = Groq(
            api_key=os.environ.get("GROQ_API_KEY"),
        )
        self.debug = debug
        self.get_logger = get_logger
        self.get_logger().info("Llama3 on Groq initialized.")
    
    def get_response(self, content):
        chat_completion = self.client.chat.completions.create(
            messages=[
                {
                    "role": "user",
                    "content": content,
                }
            ],
            model="llama3-70b-8192",
        )
        if self.debug:
            self.get_logger().info(chat_completion.choices[0].message.content)
        return chat_completion.choices[0].message.content