# %%
# %pip install mediapipe opencv-python


# %%
import socket
import cv2
import mediapipe as mp
import math
import time

# %%
def count_extended_fingers(lm_list):
    # Tip IDs: thumb=4, index=8, middle=12, ring=16, pinky=20
    tip_ids = [4, 8, 12, 16, 20]
    extended = 0
    # Thumb: compare x of tip vs IP for handedness
    if lm_list[tip_ids[0]][1] < lm_list[tip_ids[0] - 1][1]:
        extended += 1
    # Other fingers: tip y < PIP y => extended
    for i in range(1, 5):
        if lm_list[tip_ids[i]][2] < lm_list[tip_ids[i] - 2][2]:
            extended += 1
    return extended



# %%

verbose = True  # Set to False to disable drawing
esp_ip = "192.168.229.171"  # Replace with the IP shown in Serial Monitor
# esp_ip = "172.20.10.3"  # Replace with the IP shown in Serial Monitor
esp_port = 8888             # Must match `WiFiServer server(8888)`


with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    # s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    s.settimeout(10)  # Optional: timeout for safety
    s.connect((esp_ip, esp_port))
    print("Connected to ESP8266 at", esp_ip)
    # Send message
    msg = "Connected to ESP8266\n"
    s.sendall(msg.encode())

    time.sleep(0.1)
    # Read echo and confirmation
    # response1 = s.recv(1024).decode().strip()
    # print(f"Echoed: {response1}")

    mp_hands = mp.solutions.hands
    mp_drawing = mp.solutions.drawing_utils
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.7
    )
    prev_is_open = 0
    prev_theta = 0
    prev_time = time.time()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open webcam")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Mirror and get dimensions
            frame = cv2.flip(frame, 1)
            h, w, _ = frame.shape
            # Compute center after flip
            center_frame = (w // 2, h // 2)


            # Draw center dot correctly at center_frame
            cv2.circle(frame, center_frame, 5, (255, 255, 255), -1)

            # Process with MediaPipe
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            results = hands.process(rgb)
            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]
                # Landmark list in pixel coords
                lm_list = [[id, int(lm.x * w), int(lm.y * h)]
                            for id, lm in enumerate(hand_landmarks.landmark)]

                # Compute bounding box and hand center
                xs = [pt[1] for pt in lm_list]
                ys = [pt[2] for pt in lm_list]
                xmin, xmax = min(xs), max(xs)
                ymin, ymax = min(ys), max(ys)
                hand_center = ((xmin + xmax) // 2, (ymin + ymax) // 2)

                # Determine open vs fist
                cnt = count_extended_fingers(lm_list)
                is_open = cnt >= 2

                # Compute angle θ from North clockwise
                dx = hand_center[0] - center_frame[0]
                dy = center_frame[1] - hand_center[1]
                theta = math.degrees(math.atan2(dx, dy))
                # theta = theta if theta >= 0 else theta + 360

                # Send data to ESP8266
                if is_open != prev_is_open or abs(theta - prev_theta) > 5:
                    prev_is_open = is_open
                    prev_theta = theta
                    # Send data to ESP8266

                    now = time.time()
                    if (s is not None) and (now - prev_time > 0.1):
                        message = f"{int(is_open)},{int(theta)}\n"
                        s.sendall(message.encode())
                        # time.sleep(0.1)
                        prev_time = now


                # Visual Feedback
                if verbose:
                    # Draw bounding box
                    if is_open:
                        box_color = (0, 255, 0)
                        label = f'Open ({cnt})'
                    else:
                        box_color = (0, 0, 255)
                        label = f'Fist ({cnt})'

                    cv2.rectangle(frame,
                                (xmin - 10, ymin - 10),
                                (xmax + 10, ymax + 10),
                                box_color, 2)
                    cv2.putText(frame, label,
                                (xmin, ymin - 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                box_color, 2)

                    # Draw center dot and angle line
                    cv2.circle(frame, center_frame, 5, (255, 255, 255), -1)
                    cv2.line(frame, center_frame, hand_center, box_color, 2)

                    # Display angle text
                    angle_text = f'Theta: {int(theta):3d}°'
                    cv2.putText(frame, angle_text,
                                (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                (255, 255, 255), 2)

                    # Draw landmarks AFTER everything else
                    mp_drawing.draw_landmarks(
                        frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)


                    print(f"Is Open: {is_open}, Angle: {theta:.2f} degrees")

            else:
                now = time.time()
                if (s is not None) and (now - prev_time > 0.1):
                    message = f"{0},{0}\n"
                    s.sendall(message.encode())
                    # time.sleep(0.1)
                    prev_time = now
            cv2.imshow("Hand Tracking & Angle", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break

    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        cap.release()
        cv2.destroyAllWindows()

        message = f"{0},{0}\n"
        s.sendall(message.encode())

# %%



