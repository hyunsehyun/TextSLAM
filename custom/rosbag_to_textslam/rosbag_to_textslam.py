#!/usr/bin/env python3
"""
ROS2 Bag to TextSLAM Dataset Converter with PaddleOCR

This script converts ROS2 bag files to TextSLAM dataset format by:
1. Extracting images from specified topic
2. Running PaddleOCR for text detection and recognition
3. Generating TextSLAM-compatible output files
"""

import argparse
import os
from pathlib import Path
from typing import List, Tuple, Optional
import numpy as np
import cv2
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from paddleocr import PaddleOCR


class RosbagToTextSLAM:
    """Convert ROS2 bag to TextSLAM dataset format."""

    # Camera parameters (from /camera/infra1/camera_info)
    CAMERA_PARAMS = {
        'fx': 432.17333984375,
        'fy': 432.17333984375,
        'cx': 431.1373291015625,
        'cy': 237.8980712890625,
        'k1': 0.0,
        'k2': 0.0,
        'p1': 0.0,
        'p2': 0.0,
        'k3': 0.0,
        'width': 848,
        'height': 480,
        'fps': 30.0,
        'rgb': 0  # 0: BGR/Grayscale, 1: RGB
    }

    def __init__(
        self,
        bag_path: str,
        output_dir: str,
        image_topic: str = '/camera/infra1/image_rect_raw',
        lang: str = 'multilingual'
    ):
        """
        Initialize the converter.

        Args:
            bag_path: Path to ROS2 bag directory
            output_dir: Output directory for TextSLAM dataset
            image_topic: ROS topic for images
            lang: PaddleOCR language setting
        """
        self.bag_path = Path(bag_path)
        self.output_dir = Path(output_dir)
        self.image_topic = image_topic
        self.lang = lang

        # Create output directories
        self.images_dir = self.output_dir / 'images'
        self.text_dir = self.output_dir / 'text'
        self.images_dir.mkdir(parents=True, exist_ok=True)
        self.text_dir.mkdir(parents=True, exist_ok=True)

        # Initialize PaddleOCR (English only, CPU mode for compatibility)
        self.ocr = PaddleOCR(lang='en')

        self.processed_timestamps: List[str] = []

    def _timestamp_to_str(self, sec: int, nanosec: int) -> str:
        """Convert ROS2 timestamp to string format for TextSLAM."""
        return f"{sec}.{nanosec:09d}"

    def _decode_image(self, msg) -> np.ndarray:
        """Decode ROS2 Image message to numpy array."""
        # Get image dimensions
        height = msg.height
        width = msg.width
        encoding = msg.encoding

        # Convert byte data to numpy array
        if encoding in ['mono8', '8UC1']:
            # Grayscale image
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width)
        elif encoding in ['bgr8', '8UC3']:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
        elif encoding in ['rgb8']:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif encoding in ['16UC1', 'mono16']:
            # 16-bit grayscale (depth images)
            img = np.frombuffer(msg.data, dtype=np.uint16).reshape(height, width)
            # Convert to 8-bit for OCR
            img = (img / 256).astype(np.uint8)
        else:
            raise ValueError(f"Unsupported image encoding: {encoding}")

        return img

    def _run_ocr(self, img: np.ndarray) -> Tuple[List[List[float]], List[Tuple[str, float]]]:
        """
        Run PaddleOCR on image.

        Returns:
            detections: List of bounding box coordinates (8 values each)
            recognitions: List of (text, confidence) tuples
        """
        # Convert grayscale to BGR for PaddleOCR if needed
        if len(img.shape) == 2:
            img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            img_color = img

        # Run OCR
        result = self.ocr.ocr(img_color, cls=True)

        detections = []
        recognitions = []

        if result and result[0]:
            for line in result[0]:
                box = line[0]  # [[x1,y1], [x2,y2], [x3,y3], [x4,y4]]
                text_info = line[1]  # (text, confidence)

                # Flatten box coordinates: x1,y1,x2,y2,x3,y3,x4,y4
                coords = []
                for point in box:
                    coords.extend([float(point[0]), float(point[1])])
                detections.append(coords)

                recognitions.append((text_info[0], float(text_info[1])))

        return detections, recognitions

    def _save_detection_file(self, timestamp: str, detections: List[List[float]]) -> None:
        """Save detection results to _dete.txt file."""
        filepath = self.text_dir / f"{timestamp}_dete.txt"
        with open(filepath, 'w') as f:
            for coords in detections:
                # Format: x1,y1,x2,y2,x3,y3,x4,y4
                line = ','.join(f"{c:.1f}" for c in coords)
                f.write(line + '\n')

    def _save_recognition_file(self, timestamp: str, recognitions: List[Tuple[str, float]]) -> None:
        """Save recognition results to _mean.txt file."""
        filepath = self.text_dir / f"{timestamp}_mean.txt"
        with open(filepath, 'w') as f:
            for text, confidence in recognitions:
                # Format: text,confidence
                f.write(f"{text},{confidence}\n")

    def _save_image(self, timestamp: str, img: np.ndarray) -> None:
        """Save image as PNG."""
        filepath = self.images_dir / f"{timestamp}.png"
        cv2.imwrite(str(filepath), img)

    def _generate_exper_txt(self) -> None:
        """Generate Exper.txt file with image list."""
        filepath = self.output_dir / 'Exper.txt'
        with open(filepath, 'w') as f:
            for ts in self.processed_timestamps:
                f.write(f"{ts} images/{ts}.png\n")

    def _generate_yaml_config(self) -> None:
        """Generate TextSLAM YAML configuration file."""
        filepath = self.output_dir / 'custom_indoor.yaml'

        # Get absolute path for Exp read path
        read_path = str(self.output_dir.absolute()) + '/'

        yaml_content = f"""%YAML:1.0

Exp name: 1
Exp noText: 0
# Exp name  : 0: general motion; 1: indoor loop 1; 2: indoor loop 2; 3: outdoor
# Exp noText: 0: text+scene; 1: scene (only useful in exp 0)

Exp read path: {read_path}
Exp read list: Exper

Camera.fx: {self.CAMERA_PARAMS['fx']}
Camera.fy: {self.CAMERA_PARAMS['fy']}
Camera.cx: {self.CAMERA_PARAMS['cx']}
Camera.cy: {self.CAMERA_PARAMS['cy']}

Camera.k1: {self.CAMERA_PARAMS['k1']}
Camera.k2: {self.CAMERA_PARAMS['k2']}
Camera.p1: {self.CAMERA_PARAMS['p1']}
Camera.p2: {self.CAMERA_PARAMS['p2']}
Camera.k3: {self.CAMERA_PARAMS['k3']}

Camera.fps: {self.CAMERA_PARAMS['fps']}
Camera.RGB: {self.CAMERA_PARAMS['rgb']}
Camera.height: {self.CAMERA_PARAMS['height']}
Camera.width: {self.CAMERA_PARAMS['width']}
"""
        with open(filepath, 'w') as f:
            f.write(yaml_content)

    def run(self) -> None:
        """Process the ROS bag and generate TextSLAM dataset."""
        print(f"Processing bag: {self.bag_path}")
        print(f"Output directory: {self.output_dir}")
        print(f"Image topic: {self.image_topic}")

        frame_count = 0

        # Create typestore for ROS2 message deserialization
        typestore = get_typestore(Stores.ROS2_HUMBLE)

        with Reader(self.bag_path) as reader:
            # Get connections for the image topic
            connections = [c for c in reader.connections if c.topic == self.image_topic]

            if not connections:
                print(f"Error: Topic '{self.image_topic}' not found in bag.")
                print("Available topics:")
                for c in reader.connections:
                    print(f"  - {c.topic}")
                return

            # Process messages
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                # Deserialize message using typestore
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

                # Get timestamp from message header
                ts_str = self._timestamp_to_str(msg.header.stamp.sec, msg.header.stamp.nanosec)

                # Decode image
                try:
                    img = self._decode_image(msg)
                except ValueError as e:
                    print(f"Warning: {e}, skipping frame")
                    continue

                # Run OCR
                detections, recognitions = self._run_ocr(img)

                # Save outputs
                self._save_image(ts_str, img)
                self._save_detection_file(ts_str, detections)
                self._save_recognition_file(ts_str, recognitions)

                self.processed_timestamps.append(ts_str)
                frame_count += 1

                if frame_count % 10 == 0:
                    print(f"Processed {frame_count} frames...")

        # Sort timestamps
        self.processed_timestamps.sort()

        # Generate Exper.txt and YAML config
        self._generate_exper_txt()
        self._generate_yaml_config()

        print(f"\nDone! Processed {frame_count} frames.")
        print(f"Output files:")
        print(f"  - {self.output_dir}/Exper.txt")
        print(f"  - {self.output_dir}/custom_indoor.yaml")
        print(f"  - {self.output_dir}/images/ ({frame_count} images)")
        print(f"  - {self.output_dir}/text/ ({frame_count * 2} files)")


def main():
    parser = argparse.ArgumentParser(
        description='Convert ROS2 bag to TextSLAM dataset format with PaddleOCR'
    )
    parser.add_argument(
        '--bag_path',
        type=str,
        required=True,
        help='Path to ROS2 bag directory'
    )
    parser.add_argument(
        '--output_dir',
        type=str,
        required=True,
        help='Output directory for TextSLAM dataset'
    )
    parser.add_argument(
        '--image_topic',
        type=str,
        default='/camera/infra1/image_rect_raw',
        help='ROS topic for images (default: /camera/infra1/image_rect_raw)'
    )
    parser.add_argument(
        '--lang',
        type=str,
        default='multilingual',
        help='OCR language: en, korean, multilingual, etc. (default: multilingual)'
    )

    args = parser.parse_args()

    converter = RosbagToTextSLAM(
        bag_path=args.bag_path,
        output_dir=args.output_dir,
        image_topic=args.image_topic,
        lang=args.lang
    )
    converter.run()


if __name__ == '__main__':
    main()
