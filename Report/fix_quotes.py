import sys

file_path = "/Users/vernias/Desktop/verniastesi/Report/main.tex"

with open(file_path, "r", encoding="utf-8") as f:
    text = f.read()

# Replace \" with "
new_text = text.replace(r'\"', '"')

with open(file_path, "w", encoding="utf-8") as f:
    f.write(new_text)

