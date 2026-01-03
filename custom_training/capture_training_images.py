#!/usr/bin/env python3
"""
ESP32-CAM Training Image Capture Tool

This script captures images from your ESP32-CAM for training custom object detection.

Usage:
    python capture_training_images.py --stream-url http://192.168.68.63/stream

Controls:
    's' - Save current frame
    'n' - Start new object class
    'q' - Quit

Instructions:
    1. Point ESP32-CAM at your first object
    2. Press 's' to save image
    3. Move object slightly (different angle/distance)
    4. Press 's' again
    5. Repeat 100-200 times
    6. Press 'n' to start next object
    7. Repeat for all objects
"""

import cv2
import numpy as np
import argparse
import os
import time
import urllib.request
from pathlib import Path


class MJPEGStreamReader:
    """Reads MJPEG stream from ESP32-CAM"""

    def __init__(self, url):
        self.url = url
        self.stream = None
        self.bytes_buffer = b''

    def open(self):
        """Open the stream connection"""
        try:
            self.stream = urllib.request.urlopen(self.url, timeout=10)
            return True
        except Exception as e:
            print(f"Failed to open stream: {e}")
            return False

    def isOpened(self):
        """Check if stream is open"""
        return self.stream is not None

    def read(self):
        """Read next frame from stream"""
        try:
            # Read chunks until we have a complete JPEG
            while True:
                chunk = self.stream.read(4096)
                if not chunk:
                    return False, None

                self.bytes_buffer += chunk

                # Look for JPEG start and end markers
                start = self.bytes_buffer.find(b'\xff\xd8')  # JPEG start
                end = self.bytes_buffer.find(b'\xff\xd9')    # JPEG end

                if start != -1 and end != -1 and end > start:
                    # Extract the JPEG image
                    jpg_data = self.bytes_buffer[start:end+2]
                    self.bytes_buffer = self.bytes_buffer[end+2:]

                    # Decode JPEG to numpy array
                    frame = cv2.imdecode(
                        np.frombuffer(jpg_data, dtype=np.uint8),
                        cv2.IMREAD_COLOR
                    )

                    if frame is not None:
                        return True, frame

                # Prevent buffer from growing too large
                if len(self.bytes_buffer) > 500000:
                    self.bytes_buffer = self.bytes_buffer[-100000:]

        except Exception as e:
            print(f"Stream read error: {e}")
            return False, None

    def release(self):
        """Close the stream"""
        if self.stream:
            self.stream.close()
            self.stream = None

class TrainingImageCapture:
    """Captures training images from ESP32-CAM stream"""

    def __init__(self, stream_url, output_dir):
        self.stream_url = stream_url
        self.output_dir = Path(output_dir)

        # Current capture state
        self.current_class = None
        self.image_count = 0
        self.total_images = 0

        # Create output directory
        self.output_dir.mkdir(parents=True, exist_ok=True)

        print("="*60)
        print("ESP32-CAM Training Image Capture Tool")
        print("="*60)
        print(f"Saving images to: {self.output_dir}")
        print()

    def start_new_class(self):
        """Prompt user for new object class name"""
        print("\n" + "="*60)
        class_name = input("Enter object class name (e.g., 'red_ball'): ").strip()

        if not class_name:
            print("Invalid class name!")
            return False

        # Create directory for this class
        class_dir = self.output_dir / class_name
        class_dir.mkdir(exist_ok=True)

        self.current_class = class_name
        self.image_count = 0

        print(f"\nCapturing images for: {class_name}")
        print(f"Save location: {class_dir}")
        print("\nControls:")
        print("  's' - Save image")
        print("  'n' - Start new class")
        print("  'q' - Quit")
        print("="*60 + "\n")

        return True

    def save_image(self, frame):
        """Save current frame as training image"""
        if self.current_class is None:
            print("⚠️  Start a class first! Press 'n'")
            return

        # Generate filename
        timestamp = int(time.time() * 1000)
        filename = f"{self.current_class}_{self.image_count:04d}_{timestamp}.jpg"
        filepath = self.output_dir / self.current_class / filename

        # Save image
        cv2.imwrite(str(filepath), frame)

        self.image_count += 1
        self.total_images += 1

        print(f"✅ Saved: {filename} (Class total: {self.image_count}, Overall: {self.total_images})")

    def run(self):
        """Main capture loop"""
        # Connect to stream using custom MJPEG reader
        print(f"Connecting to ESP32-CAM: {self.stream_url}")
        cap = MJPEGStreamReader(self.stream_url)

        if not cap.open() or not cap.isOpened():
            print(f"❌ ERROR: Could not connect to {self.stream_url}")
            print("Make sure ESP32-CAM is running and URL is correct")
            return

        print("✅ Connected to stream!\n")

        # Prompt for first class
        if not self.start_new_class():
            return

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("⚠️  Failed to grab frame")
                    time.sleep(0.1)
                    continue

                # Create display frame with info overlay
                display = frame.copy()

                # Add info text
                info_y = 30
                cv2.putText(display, f"Class: {self.current_class or 'None'}",
                           (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                info_y += 35
                cv2.putText(display, f"Images: {self.image_count}",
                           (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                info_y += 35
                cv2.putText(display, f"Total: {self.total_images}",
                           (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # Add crosshair to help framing
                h, w = display.shape[:2]
                cv2.drawMarker(display, (w//2, h//2), (0, 0, 255),
                             cv2.MARKER_CROSS, 30, 2)

                # Show controls
                info_y = h - 90
                cv2.putText(display, "Controls:", (10, info_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                info_y += 20
                cv2.putText(display, "S - Save  |  N - New Class  |  Q - Quit",
                           (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                # Scale up for easier viewing (2x)
                display_scaled = cv2.resize(display, (w*2, h*2))

                # Display
                cv2.imshow('Training Image Capture - ESP32-CAM', display_scaled)

                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF

                if key == ord('s') or key == ord('S'):
                    self.save_image(frame)

                elif key == ord('n') or key == ord('N'):
                    self.start_new_class()

                elif key == ord('q') or key == ord('Q'):
                    print("\nQuitting...")
                    break

        except KeyboardInterrupt:
            print("\n\nInterrupted by user")

        finally:
            cap.release()
            cv2.destroyAllWindows()

            print("\n" + "="*60)
            print("Capture Session Summary")
            print("="*60)
            print(f"Total images captured: {self.total_images}")
            print(f"Saved to: {self.output_dir}")
            print()
            print("Next steps:")
            print("1. Upload images to Roboflow for labeling")
            print("2. Or use LabelImg to annotate locally")
            print("3. Then run train_custom_yolo.py")
            print("="*60)

def main():
    parser = argparse.ArgumentParser(description='Capture training images from ESP32-CAM')
    parser.add_argument('--stream-url', type=str,
                       default='http://192.168.68.63/stream',
                       help='ESP32-CAM stream URL')
    parser.add_argument('--output', type=str,
                       default='../custom_dataset/raw_images',
                       help='Output directory for captured images')

    args = parser.parse_args()

    # Create capture tool
    capture = TrainingImageCapture(args.stream_url, args.output)
    capture.run()

if __name__ == '__main__':
    main()
