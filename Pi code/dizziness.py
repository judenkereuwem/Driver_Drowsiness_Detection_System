import cv2
import mediapipe as mp
import numpy as np
import serial
from picamera2 import Picamera2
from gpiozero import LED
import time

buzzer = LED(16)
red_led = LED(21)
green_led = LED(20)

green_led.on()
time.sleep(1)
green_led.off()
red_led.off()
buzzer.off()

# Serial comm config
ser = serial.Serial('/dev/ttyUSB0',
                    baudrate=9600,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE)

# Initialize MediaPipe Face Mesh
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# Initialize MediaPipe Drawing
mp_drawing = mp.solutions.drawing_utils
drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)

# Start capturing video from the rpi_cam
cv2.startWindowThread()
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

# Initialize counter and time
counter = 0
buzzer_threshold = 35  # 4 seconds

# Initial pan and tilt angles (starting at the middle)
pan_angle = 90
tilt_angle = 40

# Limits for pan and tilt angles
PAN_MIN = 20
PAN_MAX = 160
TILT_MIN = 0
TILT_MAX = 70

# Sensitivity for angle adjustment
PAN_SENSITIVITY = 0.1
TILT_SENSITIVITY = 0.1

# Max step size for smoother movement
PAN_STEP = 1.0  # Maximum change in pan angle per update
TILT_STEP = 1.0  # Maximum change in tilt angle per update

frame_width = 640
frame_height = 480
center_frame_x = frame_width // 2
center_frame_y = frame_height // 2

def send_angle_to_arduino(pan, tilt):
    # Send formatted pan and tilt angles to Arduino
    pan_command = f'P{int(pan)}\n'
    tilt_command = f'T{int(tilt)}\n'
    ser.write(pan_command.encode())
    ser.write(tilt_command.encode())
    print(f"Sent to Arduino: Pan={pan}, Tilt={tilt}")

def smooth_move(current_angle, target_angle, step_size):
    # Ensure the angle change is gradual and smooth
    if abs(target_angle - current_angle) > step_size:
        if target_angle > current_angle:
            return current_angle + step_size
        else:
            return current_angle - step_size
    return target_angle
    

# Function to calculate Euclidean distance between two points
def calculate_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
    


# Global variables to calculate FPS
fps = 0
frame_count = 0
start_time = time.time()    
    

while True:
    im = picam2.capture_array()
    image = cv2.resize(im, (frame_width, frame_height))
    
    # Flip the image vertically and horizontally
    #image = cv2.flip(image, -1)  # Flip vertically
    image = cv2.flip(image, 1)   # Flip horizontally

    # Convert the BGR image to RGB
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Process the image and find face landmarks
    results = face_mesh.process(image_rgb)

    # Draw the face mesh annotations on the image
    if results.multi_face_landmarks:
        for face_landmarks in results.multi_face_landmarks:
            # # Draw the face mesh
            # mp_drawing.draw_landmarks(
                # image=image,
                # landmark_list=face_landmarks,
                # connections=mp_face_mesh.FACEMESH_TESSELATION,
                # landmark_drawing_spec=drawing_spec,
                # connection_drawing_spec=drawing_spec)
            
            # Get face bounding box coordinates
            h, w, _ = image_rgb.shape
            face_coords = [(int(landmark.x * w), int(landmark.y * h)) for landmark in face_landmarks.landmark]
            x_min = min([coord[0] for coord in face_coords])
            y_min = min([coord[1] for coord in face_coords])
            x_max = max([coord[0] for coord in face_coords])
            y_max = max([coord[1] for coord in face_coords])
            #cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            
            #-------------------------------------------------------------------#
            # Get eyelid coordinates
            left_eye_upper = face_coords[159]
            left_eye_lower = face_coords[145]
            right_eye_upper = face_coords[386]
            right_eye_lower = face_coords[374]
            
            # Drawing
            cv2.line(image,left_eye_lower,right_eye_lower,(0,200,0),1)
    
            w = int(calculate_distance(left_eye_lower,right_eye_lower))
            W = 6.3
    
            # Focal length
            d = 50
            f = (w*d)/(W)
    
            # Distance from camera
            f = 840
            d = (W*f)/w
            print(d)

            # Draw points on eyelids
            cv2.circle(image, left_eye_upper, 2, (255, 0, 0), -1)
            cv2.circle(image, left_eye_lower, 2, (255, 0, 0), -1)
            cv2.circle(image, right_eye_upper, 2, (255, 0, 0), -1)
            cv2.circle(image, right_eye_lower, 2, (255, 0, 0), -1)
            
            #------------------------------------------------------------------#
            
            # Calculate distance
            if d < 50:
                green_led.on()
                # Center point of the face bounding box
                face_center_x = (x_min + x_max) // 2
                face_center_y = (y_min + y_max) // 2    
                #print("Face Center Point:", (face_center_x, face_center_y))
    
                # Calculate the error from the center of the frame
                error_x = face_center_x - center_frame_x
                error_y = face_center_y - center_frame_y
                #print("Error from center:", (error_x, error_y))
                
                # Calculate target pan and tilt angles based on error
                target_pan_angle = pan_angle + error_x * PAN_SENSITIVITY
                target_tilt_angle = tilt_angle + error_y * TILT_SENSITIVITY
    
                # Keep target angles within bounds
                target_pan_angle = max(PAN_MIN, min(PAN_MAX, target_pan_angle))
                target_tilt_angle = max(TILT_MIN, min(TILT_MAX, target_tilt_angle))
    
                # Smoothly move to the target angles
                pan_angle = smooth_move(pan_angle, target_pan_angle, PAN_STEP)
                tilt_angle = smooth_move(tilt_angle, target_tilt_angle, TILT_STEP)
                
                # Send the updated angles to Arduino
                send_angle_to_arduino(pan_angle, tilt_angle)

                # Draw circle at the center of the bounding box
                cv2.circle(image, (face_center_x, face_center_y), 5, (0, 255, 0), 2)
                
                #--------------------------------------------------------------------------#
                left_distance = int(calculate_distance(left_eye_upper, left_eye_lower))
                right_distance = int(calculate_distance(right_eye_upper, right_eye_lower))
                print(f"Distance Left: {left_distance}")
                print(f"Distance Right: {right_distance}")
                
    
                if left_distance and right_distance < 14:
                    cv2.line(image, left_eye_upper, left_eye_lower, (0,0,255),2)
                    cv2.line(image, right_eye_upper, right_eye_lower, (0,0,255),2)
                    cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2)
                    cv2.rectangle(image, (x_min, y_max-35), (x_max, y_max), (0,0,255),cv2.FILLED)
                    cv2.putText(image,"EYES CLOSED",(x_min+6,y_max-6),cv2.FONT_HERSHEY_COMPLEX,0.9,(255,255,255),2)
                    
                    counter += 3  # Increment counter if condition is true
                else:
                    counter = 0  # Reset counter if condition is false
                    

                    
                # Turn on the buzzer if counter is above the threshold
                if counter >= buzzer_threshold:
                    buzzer.on()
                    red_led.on()
                    green_led.off()
                else:
                    buzzer.off()
                    red_led.off()
                    
                    
                if left_distance and right_distance > 14:
                    cv2.line(image, left_eye_upper, left_eye_lower, (0,255,0),2)
                    cv2.line(image, right_eye_upper, right_eye_lower, (0,255,0),2)
                    cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                    cv2.rectangle(image, (x_min, y_max-35), (x_max, y_max), (0,255,0),cv2.FILLED)
                    cv2.putText(image,"EYES OPEN",(x_min+6,y_max-6),cv2.FONT_HERSHEY_COMPLEX,0.9,(255,255,255),2)
                    
    
                print(counter)
                
                
            elif d > 50:
                # Send the updated angles to Arduino
                send_angle_to_arduino(90, 40)
                buzzer.off()
                red_led.off()
                green_led.off()
                
         
    # Calculate the time taken for this frame
    frame_count += 1
    elapsed_time = time.time() - start_time
    
    # Calculate the FPS every second
    if elapsed_time > 1:
        fps = frame_count / elapsed_time
        frame_count = 0
        start_time = time.time()

    # Show the FPS
    cv2.putText(image, f"FPS: {fps:.2f}", (24, 50), cv2.FONT_HERSHEY_COMPLEX,
                1, (0, 255, 0), 1, cv2.LINE_AA)     
         
                
            

    # Display the image
    #cv2.imshow('Face Tracking', image)

    # Break the loop on 'q' key press
    #if cv2.waitKey(5) & 0xFF == ord('q'):
        #break

    # Add a small delay to slow down the loop and allow smoother movement
    #time.sleep(0.05)  # Adjust this value for smoother or faster movements

# Release the capture and close windows
#cv2.destroyAllWindows()
green_led.off()
ser.close()
