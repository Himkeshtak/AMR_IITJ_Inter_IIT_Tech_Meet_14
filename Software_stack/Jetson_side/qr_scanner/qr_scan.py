import cv2
import numpy as np
from pyzbar.pyzbar import decode, ZBarSymbol
import json
import os
import subprocess

# --- CONFIGURATION ---
CAMERA_INDEX = 0             # 0 for USB, or use GStreamer string for CSI
DESIRED_WIDTH = 1920
DESIRED_HEIGHT = 1080
BLUR_THRESHOLD = 100.0       # Adjust this: Higher = Stricter (requires sharper images)

# --- GLOBAL STORAGE ---
found_codes = set()
data_list = []

def set_camera_properties(cap):
    """
    Attempts to lock focus and exposure for motion scanning.
    Note: This works best on USB cameras. For CSI/MIPI cameras, 
    settings are often handled in the GStreamer string.
    """
    # 1. Disable Auto Focus (0 = off, 1 = on usually, varies by camera)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) 
    
    # 2. Set Focus manually (Try values 0-255 to find the sharp distance for your rack)
    # cap.set(cv2.CAP_PROP_FOCUS, 40) 
    
    # 3. Disable Auto Exposure (1 = Manual, 3 = Auto)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) 
    
    # 4. Set Exposure Time (Lower = Less Blur, but Darker)
    # Value range depends on camera. Try -4, -5, or raw ms values.
    cap.set(cv2.CAP_PROP_EXPOSURE, 100) 

def is_blurry(image):
    """
    Returns True if the image is too blurry to be reliable.
    Uses the Laplacian variance method.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    score = cv2.Laplacian(gray, cv2.CV_64F).var()
    # Uncomment below to debug your blur threshold
    # print(f"Blur Score: {score}")
    return score < BLUR_THRESHOLD

def initialize_camera():
    # --- OPTION A: USB WEBCAM ---
    cap = cv2.VideoCapture(CAMERA_INDEX)
    
    # --- OPTION B: JETSON CSI CAMERA (IMX219/RPi Cam) ---
    # If using a ribbon cable camera, uncomment the lines below and comment out Option A
    # gstreamer_str = (
    #     "nvarguscamerasrc ! "
    #     "video/x-raw(memory:NVMM), width=1920, height=1080, format=NV12, framerate=30/1 ! "
    #     "nvvidconv flip-method=0 ! "
    #     "video/x-raw, width=1920, height=1080, format=BGRx ! "
    #     "videoconvert ! "
    #     "video/x-raw, format=BGR ! appsink"
    # )
    # cap = cv2.VideoCapture(gstreamer_str, cv2.CAP_GSTREAMER)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, DESIRED_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, DESIRED_HEIGHT)
    
    # Apply manual settings to prevent motion blur
    set_camera_properties(cap)
    
    if not cap.isOpened():
        print("Error: Could not access the camera.")
        return None
    return cap

def decoder(image):
    # 1. Check for Blur before processing (Saves CPU/Battery on Jetson)
    if is_blurry(image):
        cv2.putText(image, "TOO BLURRY", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        return image

    gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Decode only QR Codes
    decoded_objects = decode(gray_img, symbols=[ZBarSymbol.QRCODE])
    
    for obj in decoded_objects:
        qr_data = obj.data.decode("utf-8")
        
        if qr_data not in found_codes:
            found_codes.add(qr_data)
            entry = {
                "Serial_Number": len(data_list) + 1,
                "QR_Data": qr_data
            }
            data_list.append(entry)
            with open("qr_codes.json", "w") as f:
                json.dump(data_list, f, indent=4)
            print(f"[SAVED] {qr_data}")

        # Draw Box
        points = obj.polygon
        if len(points) == 4:
            pts = np.array(points, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(image, [pts], True, (0, 255, 0), 3)
            display_text = f"{qr_data} (Saved)" if qr_data in found_codes else qr_data
            rect = obj.rect
            cv2.putText(image, display_text, (rect.left, rect.top - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    return image

def main():
    if os.path.exists("qr_codes.json"):
        os.remove("qr_codes.json")
    
    cap = initialize_camera()
    if cap is None: return

    print("--- Jetson QR Scanner (Motion Optimized) ---")
    
    while True:
        ret, frame = cap.read()
        if not ret: break
        
        frame = decoder(frame)
        cv2.imshow('Jetson View', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()