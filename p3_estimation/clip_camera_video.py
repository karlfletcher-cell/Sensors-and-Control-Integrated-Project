"""
clip_camera_video.py — cuts the moment the marker appears out of the camera video (P3).
Run from the repo root:  python -m p3_estimation.clip_camera_video
Output: analysis/camera_marker_appears.mp4
"""
import cv2

START_STEP = 55          # 5.5 s
END_STEP = 85            # 8.5 s

reader = cv2.VideoCapture("analysis/camera_pov.mp4")
frames_per_second = reader.get(cv2.CAP_PROP_FPS)
width = int(reader.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(reader.get(cv2.CAP_PROP_FRAME_HEIGHT))
writer = cv2.VideoWriter("analysis/camera_marker_appears.mp4",
                         cv2.VideoWriter_fourcc(*"mp4v"), frames_per_second, (width, height))

frame_number = 0
while True:
    got_frame, frame = reader.read()
    if not got_frame:
        break
    if START_STEP <= frame_number <= END_STEP:
        writer.write(frame)
    frame_number += 1

reader.release()
writer.release()
print("saved analysis/camera_marker_appears.mp4")
