#!/usr/bin/env python3
"""Generate the OpenCV DICT_4X4_50 marker PNGs used by the MJCF orchard."""
from pathlib import Path
import cv2

out=Path(__file__).resolve().parents[1]/'assets'; out.mkdir(parents=True,exist_ok=True)
if not hasattr(cv2,'aruco'): raise SystemExit('OpenCV ArUco module unavailable; install opencv-contrib-python')
d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
for i in range(1,10):
    image=cv2.aruco.generateImageMarker(d,i,256,borderBits=1)
    if not cv2.imwrite(str(out/f'aruco_{i:02d}.png'),image): raise OSError(f'cannot write marker {i}')
