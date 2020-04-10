# ESP32-CAM and Elegoo Tumbller Face Following Robot

Face tracking robot that follow a persons face. Uses the ESP-FACE libraries on an ESP32-CAM to detect a face and measure distance and location of the face. Data is then sent to an Arduino Nano to control speed and direction of the robot.

The esp32-wifi-version.ino sketch has extra code which enables viewing of the camera feed in a browser, including the green box around the face. This extra code reduces the frame rate and therefore the fluidity of the face capture. The robot is more responsive when the esp32-fast-version.ino  sketch is used.



Full details on the blog: https://robotzero.one/face-tracking-robot
