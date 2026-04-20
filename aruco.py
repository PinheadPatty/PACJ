import cv2
import numpy as np
from fpdf import FPDF
import os

# Generate markers first as temporary images
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# We just need high-res images for the PDF to insert.
marker_size_px = 600

# Create a PDF in Portrait mode, standard US Letter size (8.5 x 11 inches)
# FPDF uses millimeters by default: 8.5in = 215.9mm, 11in = 279.4mm
pdf = FPDF(orientation='P', unit='mm', format='letter')
pdf.add_page()

# Settings for layout in millimeters
target_marker_size_mm = 46
padding_mm = 15

# Starting positions on the page (leaving a 20mm margin)
margin_x = 20
margin_y = 20

current_x = margin_x
current_y = margin_y

temp_files = []

for marker_id in range(6):
    # 1. Generate OpenCV marker
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_px)
    
    # 2. Save it temporarily
    temp_filename = f"temp_marker_{marker_id}.png"
    cv2.imwrite(temp_filename, marker_img)
    temp_files.append(temp_filename)
    
    # 3. Check if we need to wrap to the next row (US Letter is 215.9mm wide)
    if current_x + target_marker_size_mm > (215.9 - margin_x):
        current_x = margin_x
        current_y += target_marker_size_mm + padding_mm
        
    # 4. Insert the image into the PDF at EXACTLY 46mm x 46mm
    pdf.image(temp_filename, x=current_x, y=current_y, w=target_marker_size_mm, h=target_marker_size_mm)
    
    # Move X for the next marker
    current_x += target_marker_size_mm + padding_mm

# --- Add 50mm Calibration Line at the bottom ---
pdf.set_line_width(1.0)
line_y = 250  # mm down from top
pdf.line(margin_x, line_y, margin_x + 50, line_y)

pdf.set_font("Helvetica", size=12)
pdf.text(margin_x, line_y - 10, "MEASURE THIS LINE. IT SHOULD BE EXACTLY 50MM.")
pdf.text(margin_x, line_y - 4, "PRINT THIS PDF AT 'ACTUAL SIZE' OR '100% SCALE'.")

# Save the final PDF
pdf.output("aruco_46mm_markers_exact.pdf")

# Cleanup temporary PNGs
for f in temp_files:
    if os.path.exists(f):
        os.remove(f)

print("Saved 'aruco_46mm_markers_exact.pdf'!")
print("Open this PDF and print it using 'Actual Size' or '100%' scale. The markers will be exactly 46mm.")