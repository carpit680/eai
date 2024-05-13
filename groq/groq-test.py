import os

from groq import Groq

client = Groq(
    api_key=os.environ.get("GROQ_API_KEY"),
)

with open("prompt.txt", "r") as f:
    content = f.read()
chat_completion = client.chat.completions.create(
    messages=[
        {
            "role": "user",
            "content": content,
        }
    ],
    model="llama3-70b-8192",
)

print(chat_completion.choices[0].message.content)