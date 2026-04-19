from PIL import Image
import numpy as np

def crop_and_clean(filepath, outpath, crop_box=None, wipe_rects=None):
    img = Image.open(filepath).convert('RGB')
    arr = np.array(img)
    
    if wipe_rects:
        for (x1, y1, x2, y2) in wipe_rects:
            arr[y1:y2, x1:x2] = [255, 255, 255]
            
    img = Image.fromarray(arr)
    if crop_box:
        img = img.crop(crop_box)
    
    img.save(outpath)

# For LOS:
# Wipe title if it's there (top 170 pixels)
# Wipe legend (bottom 100 pixels, or right inside the plot)
# Let's crop it tightly to x=60..990, y=180..930. The title was above 180 (even if invisible to my <220 check, or maybe it's just white).
# Wait, let's wipe the top 180 and bottom 100 before crop? No, cropping removes them!
# Legend is at the bottom right.
# Let's crop LOS: (67, 182, 980, 910) -> wait, 910 doesn't include the x-labels if they are at 920. 
# Row chunks: 182-907
# So the box + x-labels fits in 182-907! Okay, let's crop (60, 180, 990, 920)
crop_and_clean("../scenario_los.png", "../scenario_los.png", crop_box=(60, 180, 990, 920))

# For NLOS:
# Row chunks: 45-819 (box), 846-881 (x-labels), 904-995 (legend).
# Let's crop y from 40 to 890. This keeps the box and x-labels, but removes the legend!
# Top 45 is the box. The title might be inside the top part of the box? 
# If the title is inside the top of the box, wiping y=45..100 would break the top border line. 
# Let's just crop it tightly and wipe legend.
crop_and_clean("../scenario_nlos.png", "../scenario_nlos.png", crop_box=(40, 40, 990, 895))
