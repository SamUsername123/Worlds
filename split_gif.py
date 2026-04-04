from PIL import Image
import struct, os

INPUT = "Rick Astley GIF.gif"
OUT_DIR = "usd/rick"
W, H = 480, 272

os.makedirs(OUT_DIR, exist_ok=True)
img = Image.open(INPUT)

for i in range(getattr(img, 'n_frames', 1)):
    img.seek(i)
    frame = img.convert("RGBA").resize((W, H), Image.LANCZOS)
    pixels = []
    for r, g, b, a in frame.getdata():
        # Pack as 0xAARRGGBB big-endian uint32
        argb = (a << 24) | (r << 16) | (g << 8) | b
        pixels.append(struct.pack('<I', argb))
    with open(f"{OUT_DIR}/frame_{i:04d}.bin", "wb") as f:
        f.write(b"".join(pixels))

print(f"Wrote {i+1} frames")