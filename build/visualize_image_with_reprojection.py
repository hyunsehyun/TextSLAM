#!/usr/bin/env python3
"""
Visualize reprojection error on actual image
Shows detected bounding boxes and projected centers
"""

import re
import cv2
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

def parse_reprojection_file(filename):
    """Parse Text_reprojection_error.txt file"""
    keyframes = {}

    with open(filename, 'r') as f:
        current_kf = None

        for line in f:
            line = line.strip()

            # Parse keyframe header
            if line.startswith('KF_id='):
                match = re.match(r'KF_id=(\d+),frame_id=(\d+),timestamp=([\d.]+)', line)
                if match:
                    kf_id = int(match.group(1))
                    frame_id = int(match.group(2))
                    timestamp = float(match.group(3))

                    current_kf = {
                        'kf_id': kf_id,
                        'frame_id': frame_id,
                        'timestamp': timestamp,
                        'observations': []
                    }
                    keyframes[kf_id] = current_kf

            # Parse text observation
            elif line.startswith('text_id=') and current_kf is not None:
                match = re.match(
                    r'text_id=(\d+),text="([^"]*)",proj=\(([\d.]+),([\d.]+)\),detected=\(([\d.]+),([\d.]+)\),error=([\d.]+)',
                    line
                )
                if match:
                    obs = {
                        'text_id': int(match.group(1)),
                        'text': match.group(2),
                        'proj_u': float(match.group(3)),
                        'proj_v': float(match.group(4)),
                        'det_u': float(match.group(5)),
                        'det_v': float(match.group(6)),
                        'error': float(match.group(7))
                    }
                    current_kf['observations'].append(obs)

    return keyframes

def find_closest_image(timestamp, image_dir):
    """Find image closest to timestamp"""
    image_dir = Path(image_dir)
    images = list(image_dir.glob('*.png'))

    if not images:
        return None

    # Extract timestamps from filenames
    closest_img = None
    min_dt = float('inf')

    for img_path in images:
        try:
            img_timestamp = float(img_path.stem)
            dt = abs(img_timestamp - timestamp)

            if dt < min_dt:
                min_dt = dt
                closest_img = img_path
        except:
            continue

    return closest_img

def visualize_keyframe(kf_data, image_path, output_path, show=False):
    """Visualize reprojection on image"""
    # Load image
    img = cv2.imread(str(image_path))
    if img is None:
        print(f"Error: Could not load image {image_path}")
        return False

    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))

    # === Left subplot: Detected bounding boxes ===
    ax1.imshow(img_rgb)
    ax1.set_title(f'Detected Text Bounding Boxes\nKF #{kf_data["kf_id"]}, Frame #{kf_data["frame_id"]}',
                  fontsize=14, fontweight='bold')
    ax1.axis('off')

    for obs in kf_data['observations']:
        det_u, det_v = obs['det_u'], obs['det_v']
        text = obs['text']

        # Draw detected center (GREEN circle)
        ax1.plot(det_u, det_v, 'go', markersize=8, markeredgewidth=2, markeredgecolor='white')

        # Draw text label
        ax1.text(det_u + 10, det_v - 10, f"{text}\n({det_u:.0f}, {det_v:.0f})",
                color='lime', fontsize=10, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='black', alpha=0.7))

    # === Right subplot: Reprojection comparison ===
    ax2.imshow(img_rgb)
    ax2.set_title(f'Reprojection Error Visualization\n(Green=Detected, Red=Projected)',
                  fontsize=14, fontweight='bold')
    ax2.axis('off')

    for obs in kf_data['observations']:
        det_u, det_v = obs['det_u'], obs['det_v']
        proj_u, proj_v = obs['proj_u'], obs['proj_v']
        text = obs['text']
        error = obs['error']

        # Draw detected center (GREEN)
        ax2.plot(det_u, det_v, 'go', markersize=10, markeredgewidth=2, markeredgecolor='white', label='Detected' if obs == kf_data['observations'][0] else '')

        # Draw projected center (RED)
        ax2.plot(proj_u, proj_v, 'rx', markersize=12, markeredgewidth=3, markeredgecolor='white', label='Projected' if obs == kf_data['observations'][0] else '')

        # Draw error vector (line connecting detected to projected)
        ax2.plot([det_u, proj_u], [det_v, proj_v], 'y-', linewidth=2, alpha=0.7)

        # Draw text label
        mid_u = (det_u + proj_u) / 2
        mid_v = (det_v + proj_v) / 2

        ax2.text(mid_u + 15, mid_v, f"{text}\nError: {error:.2f}px",
                color='yellow', fontsize=10, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='black', alpha=0.8))

    # Add legend
    ax2.legend(loc='upper right', fontsize=12, framealpha=0.9)

    # Add statistics box
    errors = [obs['error'] for obs in kf_data['observations']]
    stats_text = f"Observations: {len(errors)}\n"
    stats_text += f"Mean error: {np.mean(errors):.2f}px\n"
    stats_text += f"Max error: {np.max(errors):.2f}px\n"
    stats_text += f"Min error: {np.min(errors):.2f}px"

    ax2.text(0.02, 0.98, stats_text, transform=ax2.transAxes,
            verticalalignment='top', horizontalalignment='left',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.9),
            fontsize=11, family='monospace', fontweight='bold')

    plt.tight_layout()
    plt.savefig(output_path, dpi=200, bbox_inches='tight')
    if show:
        plt.show()
    plt.close(fig)
    return True

def main():
    import os

    # Configuration
    reprojection_file = 'Text_reprojection_error.txt'
    image_dir = '/home/sehyeon/ros2_ws/src/TextSLAM/output/images'
    output_dir = 'reprojection_results'

    # Create output directory
    os.makedirs(output_dir, exist_ok=True)

    # Parse reprojection data
    print("Parsing reprojection error data...")
    keyframes = parse_reprojection_file(reprojection_file)

    if not keyframes:
        print("Error: No keyframe data found!")
        return

    # Filter keyframes with observations
    kf_with_obs = {kf_id: kf_data for kf_id, kf_data in keyframes.items()
                   if kf_data['observations']}

    if not kf_with_obs:
        print("Error: No keyframes with observations found!")
        return

    print(f"Found {len(kf_with_obs)} keyframes with text observations")
    print(f"Output directory: {output_dir}/")
    print("=" * 80)

    # Process all keyframes
    processed = 0
    failed = 0
    total_obs = 0

    sorted_kf_ids = sorted(kf_with_obs.keys())

    for idx, kf_id in enumerate(sorted_kf_ids, 1):
        kf_data = kf_with_obs[kf_id]
        num_obs = len(kf_data['observations'])

        print(f"[{idx}/{len(sorted_kf_ids)}] KF #{kf_id:04d} (Frame #{kf_data['frame_id']}, {num_obs} obs)...", end=" ")

        # Find closest image
        image_path = find_closest_image(kf_data['timestamp'], image_dir)

        if image_path is None:
            print("SKIP (no image)")
            failed += 1
            continue

        # Visualize and save
        output_path = os.path.join(output_dir, f'KF_{kf_id:04d}.png')
        success = visualize_keyframe(kf_data, image_path, output_path, show=False)

        if success:
            print("OK")
            processed += 1
            total_obs += num_obs
        else:
            print("FAILED")
            failed += 1

    # Summary
    print("=" * 80)
    print(f"Completed: {processed}/{len(sorted_kf_ids)} keyframes")
    print(f"Total observations visualized: {total_obs}")
    print(f"Output saved to: {output_dir}/")

if __name__ == '__main__':
    main()
