from PIL import Image
import numpy as np

def check_image(filename):
    try:
        img = Image.open(filename).convert('RGB')
        arr = np.array(img)
        print(f"File: {filename}, Shape: {arr.shape}")
    except Exception as e:
        print(f"Error reading {filename}: {e}")

check_image("../scenario_los.png")
check_image("../scenario_nlos.png")
