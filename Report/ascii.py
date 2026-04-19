from PIL import Image
import numpy as np

def ascii_art(filepath):
    print(f"\n--- ASCII ART for {filepath} ---")
    img = Image.open(filepath).convert('L')
    img = img.resize((80, 40))
    arr = np.array(img)
    
    # 0 is black, 255 is white
    chars = " @%#*+=-:. "
    
    for row in range(arr.shape[0]):
        line = ""
        for col in range(arr.shape[1]):
            val = arr[row, col]
            # scale 0-255 to 0-10
            idx = int((val / 255.0) * 10)
            if idx > 10: idx = 10
            line += chars[idx]
        print(line)

ascii_art("../scenario_los.png")
ascii_art("../scenario_nlos.png")
