import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import numpy as np
import urllib.request
import os

# Download model if not present
model_path = "hand_landmarker.task"
if not os.path.exists(model_path):
    print("Downloading hand landmark model...")
    urllib.request.urlretrieve(
        "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task",
        model_path
    )
    print("Download complete.")

last_left_state  = None
last_right_state = None
latest_result    = None

def is_fist(landmarks):
    tips  = [8, 12, 16, 20]
    bases = [6, 10, 14, 18]
    folded = sum(1 for t, b in zip(tips, bases)
                 if landmarks[t].y > landmarks[b].y)
    return folded >= 3

def palm_center(lm):
    pts = [0, 5, 9, 13, 17]
    x = sum(lm[p].x for p in pts) / len(pts)
    y = sum(lm[p].y for p in pts) / len(pts)
    return round(x, 3), round(y, 3)

def result_callback(result, output_image, timestamp_ms):
    global latest_result
    latest_result = result

# Setup detector
base_options = python.BaseOptions(model_asset_path=model_path)
options = vision.HandLandmarkerOptions(
    base_options=base_options,
    running_mode=vision.RunningMode.LIVE_STREAM,
    num_hands=2,
    min_hand_detection_confidence=0.7,
    min_hand_presence_confidence=0.7,
    min_tracking_confidence=0.5,
    result_callback=result_callback
)
detector = vision.HandLandmarker.create_from_options(options)

cap = cv2.VideoCapture(0)
timestamp = 0

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    timestamp += 1

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
    detector.detect_async(mp_image, timestamp)

    left_info  = None
    right_info = None

    if latest_result and latest_result.hand_landmarks:
        for i, hand_landmarks in enumerate(latest_result.hand_landmarks):
            label = latest_result.handedness[i][0].display_name

            lm = hand_landmarks
            x, y  = palm_center(lm)
            state = "FIST" if is_fist(lm) else "OPEN"

            if label == "Left":
                left_info  = (x, y, state)
            else:
                right_info = (x, y, state)

            # Draw landmarks manually
            h, w = frame.shape[:2]
            for connection in [(0,1),(1,2),(2,3),(3,4),
                               (0,5),(5,6),(6,7),(7,8),
                               (0,9),(9,10),(10,11),(11,12),
                               (0,13),(13,14),(14,15),(15,16),
                               (0,17),(17,18),(18,19),(19,20),
                               (5,9),(9,13),(13,17)]:
                x1 = int(lm[connection[0]].x * w)
                y1 = int(lm[connection[0]].y * h)
                x2 = int(lm[connection[1]].x * w)
                y2 = int(lm[connection[1]].y * h)
                cv2.line(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)

            for lmk in lm:
                cx2 = int(lmk.x * w)
                cy2 = int(lmk.y * h)
                cv2.circle(frame, (cx2, cy2), 4, (255, 255, 255), -1)

            # Draw palm center dot
            cx = int(x * w)
            cy = int(y * h)
            cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)

            # Label on screen
            cv2.putText(frame, f"{label} {state} ({x},{y})",
                        (cx - 60, cy - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0), 2)

    # Only print to terminal on state change
    if left_info:
        if left_info[2] != last_left_state:
            print(f"Left hand: {left_info[2]}")
            last_left_state = left_info[2]
    else:
        if last_left_state is not None:
            print("Left hand: not detected")
            last_left_state = None

    if right_info:
        if right_info[2] != last_right_state:
            print(f"Right hand: {right_info[2]}")
            last_right_state = right_info[2]
    else:
        if last_right_state is not None:
            print("Right hand: not detected")
            last_right_state = None

    cv2.imshow("Hand Test", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
detector.close()