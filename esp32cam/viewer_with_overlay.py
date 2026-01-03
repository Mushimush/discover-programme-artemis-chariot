#!/usr/bin/env python3
"""
ESP32-CAM Detection Viewer - Live Video with Overlay

Uses requests with streaming for ESP32-CAM MJPEG.

Usage:
    python viewer_with_overlay.py [--ip IP_ADDRESS]

Controls:
    q - Quit
"""

import cv2
import numpy as np
import requests
import argparse
import time
from threading import Thread

# Default ESP32-CAM IP
DEFAULT_IP = "192.168.68.61"

# Global detection data
g_detection = None
g_running = True


def fetch_detection_loop(url):
    """Background thread to fetch detection data"""
    global g_detection, g_running
    while g_running:
        try:
            response = requests.get(url, timeout=2)
            if response.status_code == 200:
                g_detection = response.json()
        except:
            pass
        time.sleep(0.3)


def main():
    global g_detection, g_running

    parser = argparse.ArgumentParser(description="ESP32-CAM Detection Viewer")
    parser.add_argument("--ip", default=DEFAULT_IP,
                        help=f"ESP32-CAM IP address (default: {DEFAULT_IP})")
    args = parser.parse_args()

    stream_url = f"http://{args.ip}/stream"
    detection_url = f"http://{args.ip}/detection"

    print(f"Connecting to ESP32-CAM stream: {stream_url}")
    print("Press 'q' to quit")

    # Start detection fetcher in background
    det_thread = Thread(target=fetch_detection_loop, args=(detection_url,), daemon=True)
    det_thread.start()

    # Create window
    cv2.namedWindow("ESP32-CAM Detection Viewer", cv2.WINDOW_AUTOSIZE)

    frame_count = 0

    while g_running:
        try:
            # Use requests with stream=True and iter_content
            print(f"Connecting to stream...")
            response = requests.get(stream_url, stream=True, timeout=10)
            print(f"Connected! Status: {response.status_code}")

            bytes_buffer = b''

            for chunk in response.iter_content(chunk_size=1024):
                if not g_running:
                    break

                if chunk:
                    bytes_buffer += chunk

                    # Find JPEG boundaries
                    while True:
                        start = bytes_buffer.find(b'\xff\xd8')
                        end = bytes_buffer.find(b'\xff\xd9')

                        if start != -1 and end != -1 and end > start:
                            # Extract and decode JPEG
                            jpg_data = bytes_buffer[start:end + 2]
                            bytes_buffer = bytes_buffer[end + 2:]

                            frame = cv2.imdecode(
                                np.frombuffer(jpg_data, dtype=np.uint8),
                                cv2.IMREAD_COLOR
                            )

                            if frame is None:
                                continue

                            frame_count += 1

                            # Draw overlays
                            h, w = frame.shape[:2]
                            cx, cy = w // 2, h // 2

                            # Center crosshair (green)
                            cv2.line(frame, (cx - 30, cy), (cx + 30, cy), (0, 255, 0), 2)
                            cv2.line(frame, (cx, cy - 30), (cx, cy + 30), (0, 255, 0), 2)
                            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

                            # Detection overlay
                            det = g_detection

                            if det and det.get("detected"):
                                x = det["bbox"]["x"]
                                y = det["bbox"]["y"]
                                bw = det["bbox"]["w"]
                                bh = det["bbox"]["h"]
                                cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 0, 255), 2)

                                ocx = det["center"]["x"]
                                ocy = det["center"]["y"]
                                cv2.circle(frame, (ocx, ocy), 8, (0, 0, 0), -1)
                                cv2.circle(frame, (ocx, ocy), 6, (0, 255, 255), -1)

                                cv2.line(frame, (cx, cy), (ocx, ocy), (255, 0, 255), 2)

                                label = det.get("label", "?")
                                conf = det.get("confidence", 0)
                                offset_x = det.get("offset", {}).get("x", 0)
                                turn = det.get("turn_percent", 0)
                                direction = det.get("direction", "?")

                                cv2.putText(frame, f"{label} {conf:.0%}", (10, 25),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                                cv2.putText(frame, f"Offset:{offset_x} Turn:{turn}%", (10, 50),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                                cv2.putText(frame, f"{direction}", (10, 75),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                            # Frame counter
                            cv2.putText(frame, f"F:{frame_count}", (w - 70, 20),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)

                            cv2.imshow("ESP32-CAM Detection Viewer", frame)

                            key = cv2.waitKey(1) & 0xFF
                            if key == ord('q'):
                                g_running = False
                                break
                        else:
                            break

                    # Prevent buffer overflow
                    if len(bytes_buffer) > 100000:
                        bytes_buffer = bytes_buffer[-50000:]

        except Exception as e:
            print(f"Stream error: {e}, reconnecting in 2s...")
            time.sleep(2)

    cv2.destroyAllWindows()
    print("Viewer closed")


if __name__ == "__main__":
    main()
