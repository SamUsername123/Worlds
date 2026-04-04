from PIL import Image
import struct, os, cv2

INPUT = "/home/sam/Videos/Rick Astley - Never Gonna Give You Up (Official Video) (4K Remaster) [dQw4w9WgXcQ].mp4"
OUT_DIR = "usd/rick"

os.makedirs(OUT_DIR, exist_ok=True)

cap = cv2.VideoCapture(INPUT)
i = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # OpenCV loads as BGR, convert to RGBA
    frame_rgba = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
    img = Image.fromarray(frame_rgba)

    with open(f"{OUT_DIR}/frame_{i:04d}.bin", "wb") as f:
        for r, g, b, a in img.getdata():
            argb = (a << 24) | (r << 16) | (g << 8) | b
            f.write(struct.pack('<I', argb))

    i += 1

cap.release()
print(f"Wrote {i} frames")