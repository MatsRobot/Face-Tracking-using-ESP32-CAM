# Moving-head-using-ESP32-CAM
This part of the Project is using ESP32-CAM to detect your face and use the coordinates to Tilt and Pans to point to your face.

![Screenshot_20240813-100706_Gallery](https://github.com/user-attachments/assets/d3844ae2-bbff-478e-a5de-2d20f7aa0e16)

ESP32-CAM uses a lot of its GPIO pins for the camera, so to control the two servo motors, PCA9685 16-Channel 12-bit PWM Servo Driver I2C Interface is used to isolate the two circuits and it seems that this method working ok.

More details are in the documentations file.
