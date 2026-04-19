from PIL import Image
import numpy as np

def clean_edges(filepath, wipe_top=80, wipe_bottom=80, wipe_right=150):
    img = Image.open(filepath).convert('RGB')
    arr = np.array(img)
    
    # Wipe top (title)
    if wipe_top > 0:
        arr[:wipe_top, :] = [255, 255, 255]
        
    # Wipe bottom (legend / labels)
    if wipe_bottom > 0:
        arr[-wipe_bottom:, :] = [255, 255, 255]
        
    # Wipe right (legend)
    if wipe_right > 0:
        arr[:, -wipe_right:] = [255, 255, 255]
        
    img = Image.fromarray(arr)
    # Crop to just the non-white bounding box
    gray = np.array(img.convert('L'))
    dark_pixels = np.where(gray < 250)
    if len(dark_pixels[0]) > 0:
        y_min = max(0, np.min(dark_pixels[0]) - 10)
        y_max = min(gray.shape[0], np.max(dark_pixels[0]) + 10)
        x_min = max(0, np.min(dark_pixels[1]) - 10)
        x_max = min(gray.shape[1], np.max(dark_pixels[1]) + 10)
        img = img.crop((x_min, y_min, x_max, y_max))
        
    img.save(filepath)

# For LOS, wipe top 100, bottom 100, right 200
clean_edges("../scenario_los.png", wipe_top=150, wipe_bottom=100, wipe_right=200)

# For NLOS, wipe top 80, bottom 100, right 100
clean_edges("../scenario_nlos.png", wipe_top=80, wipe_bottom=100, wipe_right=100)
