import cv2
import numpy as np
import os
import subprocess

# 1. Force the sensor to output maximum data
subprocess.run(['v4l2-ctl', '-d', '/dev/v4l-subdev0', '-c', 'exposure=1200,analogue_gain=150,digital_gain=2000'])
subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--set-fmt-video=width=1280,height=720,pixelformat=RG10', '--stream-mmap', '--stream-count=1', '--stream-to=test_raw_auto.bin'])

# 2. Read the raw bayer data
if not os.path.exists('test_raw_auto.bin'):
    print("Capture failed")
    exit(1)
    
raw = np.fromfile('test_raw_auto.bin', dtype=np.uint16).reshape((720, 1280))

# Convert 10-bit packed into 8-bit
img8 = (raw >> 2).astype(np.uint8)

# 3. Demosaic (Bayer to RGB)
color = cv2.cvtColor(img8, cv2.COLOR_BayerRG2BGR)

# 4. Simple Auto White Balance & Auto Exposure
# Get the average brightness of each channel
mean_b = np.mean(color[:, :, 0])
mean_g = np.mean(color[:, :, 1])
mean_r = np.mean(color[:, :, 2])

# Scale to make the image properly bright (target mean ~120)
target_mean = 120.0
current_mean = (mean_b + mean_g + mean_r) / 3.0
scale = target_mean / max(current_mean, 1.0)

b = np.clip(color[:, :, 0] * scale * (mean_g / max(mean_b, 1.0)), 0, 255).astype(np.uint8)
g = np.clip(color[:, :, 1] * scale, 0, 255).astype(np.uint8)
r = np.clip(color[:, :, 2] * scale * (mean_g / max(mean_r, 1.0)), 0, 255).astype(np.uint8)

final_img = cv2.merge([b, g, r])

# Optional slightly boost saturation and contrast
hsv = cv2.cvtColor(final_img, cv2.COLOR_BGR2HSV)
hsv[:,:,1] = np.clip(hsv[:,:,1] * 1.2, 0, 255)
final_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

# Write output file
cv2.imwrite('normal_photo.jpg', final_img)
print("Saved simulated regular photo as normal_photo.jpg")
