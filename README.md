**Note:** this is a fork with minor changes to the original. It is made to suit Dragger and my other projects. Refer to the origin if in doubt.

See https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Dragger

See https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/MPU9250.md

BTW, I didn't see abnormal CPU utilization on my Raspberry Pi 5 - mentioned by Nils Schulte below.

========================= _Original Text Below_ ===========================

This is a ROS2 driver (written in Python, so expect your RPi to have one core at 50% all the time if you run this) for the MPU9250 Accelerometer, Gyroscope and Magnetometer sensor package.
Works alright if the sensor is calibrated.
This is a wraper around the nice library form niru-5. He also has some examples and shows how to calibrate the sensor.

# Credits
ROS2 wrapper around [```https://github.com/niru-5/imusensor```](https://github.com/niru-5/imusensor)
