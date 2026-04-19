from PIL import Image
import numpy as np

def process_image(filepath, outpath):
    try:
        img = Image.open(filepath).convert('L')
    except Exception as e:
        print(f"Failed to open {filepath}: {e}")
        return
        
    arr = np.array(img)
    # Background is white (255). Text/lines are darker.
    # Let's find rows and columns that have dark pixels.
    # We define dark as < 240
    dark_pixels = arr < 220
    
    # row sums (number of dark pixels per row)
    row_sums = dark_pixels.sum(axis=1)
    # col sums
    col_sums = dark_pixels.sum(axis=0)
    
    # Let's print out the chunks of rows that have dark pixels (to identify title vs main body)
    dark_rows = np.where(row_sums > 5)[0]
    
    chunks = []
    if len(dark_rows) > 0:
        start = dark_rows[0]
        prev = dark_rows[0]
        for idx in dark_rows[1:]:
            if idx - prev > 10: # gap of 10 white pixels
                chunks.append((start, prev))
                start = idx
            prev = idx
        chunks.append((start, prev))
        
    print(f"File {filepath}: {arr.shape}")
    print("Row chunks (y_start, y_end):", chunks)
    
    dark_cols = np.where(col_sums > 5)[0]
    col_chunks = []
    if len(dark_cols) > 0:
        start = dark_cols[0]
        prev = dark_cols[0]
        for idx in dark_cols[1:]:
            if idx - prev > 10:
                col_chunks.append((start, prev))
                start = idx
            prev = idx
        col_chunks.append((start, prev))
    print("Col chunks (x_start, x_end):", col_chunks)

process_image("../scenario_los.png", "")
process_image("../scenario_nlos.png", "")
