# Driver_Drowsiness_Detection_System

Drowsy driving is a major cause of road accidents, but early signs of fatigue can be detected before a critical situation arises. This device is a Driver Drowsiness Detection System which uses face landmark detection and image processing to track eye movements and alert drivers if they start dozing off.

Hardware Tech Stack
- Raspberry Pi 5 
🔹 Pi Camera
🔹 Pan/tilt camera mount with 2 servo motors
🔹 Arduino Nano - controlling servos
🔹 Buzzer

Software Tech Stack
🔹 Raspberry Pi OS (Bookworm)
🔹 Python3.11
🔹 Mediapipe (Face landmark detection)
🔹 Arduino IDE

How It Works
📌 The device is placed on the car dashboard, facing the driver.
📌 The system only tracks the face that is within 60 cm of the camera. (To ensure the camera focuses only on the driver).
📌 The pan/tilt camera mount adjusts the camera automatically to keep the driver’s face always in focus.
📌 The system monitors the distance between the upper and lower eyelids:
	- Large distance = Eyes open (awake).
	- Small distance = Eyes closed (potentially sleeping).
📌 If eyes remain closed for 2+ seconds, a buzzer sounds until the driver is alert.

Future Upgrades
🔹 Night Vision for low-light detection.
🔹 Call or Notification System to alert a third party if the driver remains unresponsive to the buzzer.
🔹 Vibration Motor on the steering wheel or seat to physically alert the driver.

This project aims to reduce road accidents and save lives with AI-powered monitoring. Looking forward to enhancing it further! 🚀
