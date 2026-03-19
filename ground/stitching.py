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
        images.append(img)
    
    if len(images) < 2:
        return False, "Need at least 2 images to stitch."

    # Use OpenCV's built-in Stitcher
    # For newer OpenCV versions, use cv2.Stitcher_create()
    cv2.ocl.setUseOpenCL(False)
    stitcher = cv2.Stitcher_create()
    status, stitched = stitcher.stitch(images)

    if status == cv2.Stitcher_OK:
        cv2.imwrite(output_path, stitched)
        return True, "Success"
    else:
        # Status codes:
        # ERR_NEED_MORE_IMGS = 1
        # ERR_HOMOGRAPHY_EST_FAIL = 2
        # ERR_CAMERA_PARAMS_ADJUST_FAIL = 3
        return False, f"Stitching failed with status code: {status}"

if __name__ == "__main__":
    # Test stub
    import sys
    if len(sys.argv) > 2:
        success, msg = stitch_images(sys.argv[1:-1], sys.argv[-1])
        print(f"Stitching result: {success}, {msg}")
    else:
        print("Usage: python stitching.py img1.jpg img2.jpg ... output.jpg")
