#!/usr/bin/env python3
import cv2
import time

import subprocess

subprocess.call([
    "v4l2-ctl", "-d", "/dev/video2",
    "--set-ctrl=auto_exposure=1",
    "--set-ctrl=exposure_dynamic_framerate=0",
    "--set-ctrl=exposure_time_absolute=150"
])


# === GStreamer-Pipeline (MJPEG → NVDEC → BGR) ===
gst_str = (
    "v4l2src device=/dev/video2 ! "
    "image/jpeg,framerate=30/1,width=1280,height=720 ! "
    "jpegdec ! nvvidconv ! video/x-raw,format=BGRx ! "
    "videoconvert ! appsink drop=true sync=false"
)

print("[INFO] Öffne Kamera über GStreamer:")
print("   ", gst_str)

cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    raise RuntimeError("❌ Kamera konnte über GStreamer nicht geöffnet werden")

print("[INFO] Kamera erfolgreich geöffnet.")

# === FPS-Messung ===
num_frames = 100
print(f"[INFO] Starte FPS-Messung über {num_frames} Frames...")

start_time = time.time()
count = 0

while count < num_frames:
    ok, frame = cap.read()
    if not ok:
        print("⚠️ Frame konnte nicht gelesen werden.")
        continue
    count += 1

end_time = time.time()
elapsed = end_time - start_time
fps = count / elapsed
print(f"[RESULT] Effektive FPS: {fps:.2f}")
print(f"[RESULT] Durchschnittliche Frame-Zeit: {1000/fps:.1f} ms")

# === Beispielanzeige eines Frames ===
cv2.imshow("GStreamer Camera", frame)
cv2.waitKey(0)
cv2.destroyAllWindows()

cap.release()
