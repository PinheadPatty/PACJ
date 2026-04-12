import cv2
import numpy as np

# A4 page size in pixels at 300 DPI
PAGE_WIDTH = 2480
PAGE_HEIGHT = 3508

# Create a blank white image
page = np.ones((PAGE_HEIGHT, PAGE_WIDTH), dtype=np.uint8) * 255

# We used DICT_4X4_50 in your drone code
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Size of the marker in pixels (at 300 DPI, 46mm is roughly 543 pixels)
marker_size_px = 543 
padding = 150 # Space between markers

x_offset = padding
y_offset = padding

# Let's generate 6 markers (IDs 0 to 5)
for marker_id in range(12):
    # Generate the marker
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_px)
    
    # Check if we need to move to the next row
    if x_offset + marker_size_px > PAGE_WIDTH:
        x_offset = padding
        y_offset += marker_size_px + padding

    # Paste the marker onto the white page
    page[y_offset:y_offset+marker_size_px, x_offset:x_offset+marker_size_px] = marker_img
    
    # Move X offset for the next marker
    x_offset += marker_size_px + padding

# Save the resulting page
cv2.imwrite("aruco_46mm_markers.png", page)
print("Saved aruco_46mm_markers.png! Print this image at 100% scale (Do not fit to page).")