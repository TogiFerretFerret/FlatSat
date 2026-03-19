import cv2
import numpy as np
import os
import json

def stitch_images(image_paths, output_path):
    """
    Stitches multiple images into one panorama.
    Tries OpenCV high-level Stitcher first, falls back to manual Homography if needed.
    """
    images = []
    for path in image_paths:
        img = cv2.imread(path)
        if img is None:
            continue
        # Resize for performance and better feature detection consistency
        h, w = img.shape[:2]
        if w > 1000:
            scale = 1000 / w
            img = cv2.resize(img, (1000, int(h * scale)))
        images.append(img)
    
    if len(images) < 2:
        return False, "Need at least 2 valid images to stitch."

    # --- ATTEMPT 1: OpenCV Stitcher (High Quality) ---
    try:
        # Use PANORAMA for natural scenes or SCANS for flat tabletop
        # We try SCANS first as it's better for "moving over a table"
        stitcher = cv2.Stitcher_create(cv2.Stitcher_SCANS)
        status, stitched = stitcher.stitch(images)
        if status == cv2.Stitcher_OK:
            cv2.imwrite(output_path, stitched)
            return True, "Success (OpenCV Stitcher)"
    except Exception as e:
        print(f"[Stitcher] OpenCV Stitcher crashed: {e}")

    # --- ATTEMPT 2: Manual Feature Matching (Robust Fallback) ---
    # This is much more reliable for mapping when overlap is tricky
    print("[Stitcher] Falling back to manual feature matching...")
    try:
        result = images[0]
        orb = cv2.ORB_create(nfeatures=2000)
        
        for i in range(1, len(images)):
            img2 = images[i]
            
            # Find keypoints and descriptors
            kp1, des1 = orb.detectAndCompute(result, None)
            kp2, des2 = orb.detectAndCompute(img2, None)
            
            if des1 is None or des2 is None: continue
            
            # Match features
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(des1, des2)
            matches = sorted(matches, key=lambda x: x.distance)
            
            # Need at least 10 good matches
            if len(matches) < 10: 
                print(f"[Stitcher] Not enough matches for image {i}")
                continue
                
            src_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
            
            # Find Homography
            H, mask = cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 5.0)
            if H is None: continue
            
            # Warp image 2 to align with result
            h1, w1 = result.shape[:2]
            h2, w2 = img2.shape[:2]
            
            # Calculate corners of warped image to find new canvas size
            pts = np.float32([[0, 0], [0, h2], [w2, h2], [w2, 0]]).reshape(-1, 1, 2)
            dst = cv2.perspectiveTransform(pts, H)
            
            # Combine canvas
            all_pts = np.concatenate((np.float32([[0, 0], [0, h1], [w1, h1], [w1, 0]]).reshape(-1, 1, 2), dst), axis=0)
            [x_min, y_min] = np.int32(all_pts.min(axis=0).ravel() - 0.5)
            [x_max, y_max] = np.int32(all_pts.max(axis=0).ravel() + 0.5)
            
            translation_dist = [-x_min, -y_min]
            H_translation = np.array([[1, 0, translation_dist[0]], [0, 1, translation_dist[1]], [0, 0, 1]])
            
            output_img = cv2.warpPerspective(img2, H_translation.dot(H), (x_max - x_min, y_max - y_min))
            output_img[translation_dist[1]:h1 + translation_dist[1], translation_dist[0]:w1 + translation_dist[0]] = result
            result = output_img

        cv2.imwrite(output_path, result)
        return True, "Success (Manual Fallback)"
        
    except Exception as e:
        import traceback
        traceback.print_exc()
        return False, f"Manual stitching failed: {str(e)}"

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 2:
        success, msg = stitch_images(sys.argv[1:-1], sys.argv[-1])
        print(f"Stitching result: {success}, {msg}")
