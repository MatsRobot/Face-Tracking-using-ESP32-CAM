# Face Tracking using ESP32-CAM without the use of Webserver app
This part of the Project is using ESP32-CAM to detect your face and use the coordinates to Tilt and Pans to point to your face.
When I was searching the web for a solution, I realised that all examples of the ESP32-cam uses the standard example of using a webserver app to access the data and for my project this was not enough.
The solution that I found was to get the X-Y location of the face and use it to move the Tilt and Pan servos accordingly without the need to switch to any other device like your phone.
The downside is that you cannot see what the camera sees and you need to rely on the date that is passed by the functions, which in my case was adequate and I used a little OLED display to display the detected XY coordinates whilst tracking a face.

![Screenshot_20240813-100706_Gallery](https://github.com/user-attachments/assets/d3844ae2-bbff-478e-a5de-2d20f7aa0e16)

ESP32-CAM uses a lot of its GPIO pins for the camera, so to control the two servo motors, PCA9685 16-Channel 12-bit PWM Servo Driver I2C Interface is used to isolate the two circuits and it seems that this method working ok.

More details are in the documentations file.
