import cv2
import numpy as np
import os
import json

def stitch_images(image_paths, output_path):
    """
    Stitches multiple images into one panorama.
    image_paths: List of absolute paths to images.
    output_path: Path to save the stitched image.
    Returns: (success_bool, status_message)
    """
    images = []
    for path in image_paths:
        img = cv2.imread(path)
        if img is None:
            return False, f"Could not read image: {path}"
        # Resize images to speed up stitching and improve matching
        h, w = img.shape[:2]
        if w > 1200:
            scale = 1200 / w
            img = cv2.resize(img, (1200, int(h * scale)))
        images.append(img)
    
    if len(images) < 2:
        return False, "Need at least 2 images to stitch."

    # Use OpenCV's built-in Stitcher with SCANS mode for better flat-plane stitching
    # Note: cv2.Stitcher_create(mode) might not work on all OpenCV versions
    # Use the default if the mode isn't supported
    try:
        stitcher = cv2.Stitcher_create(cv2.Stitcher_SCANS)
    except:
        stitcher = cv2.Stitcher_create()
        
    status, stitched = stitcher.stitch(images)

    if status == cv2.Stitcher_OK:
        cv2.imwrite(output_path, stitched)
        return True, "Success"
    else:
        err_msgs = {
            1: "NEED_MORE_IMGS (Not enough overlap or features)",
            2: "HOMOGRAPHY_EST_FAIL (Could not align images)",
            3: "CAMERA_PARAMS_ADJUST_FAIL (Exposure or focus mismatch)"
        }
        return False, err_msgs.get(status, f"Error Code: {status}")

if __name__ == "__main__":
    # Test stub
    import sys
    if len(sys.argv) > 2:
        success, msg = stitch_images(sys.argv[1:-1], sys.argv[-1])
        print(f"Stitching result: {success}, {msg}")
    else:
        print("Usage: python stitching.py img1.jpg img2.jpg ... output.jpg")
