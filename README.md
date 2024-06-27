# Flight_Controller
The work is to create its own drone
software, allowing remote control and automatic stabilization. My flight controller is based on
ESP32, which communicates via WiFi (UDP) with a laptop. From a practical point of view, it is
possible that this is not a good choice, but I used didactic purposes in order to regain experience
useful in a wider range of professions during the implementation. I write code in C / C++ in ESPIDF
framework using FreeRTOS. The server on the laptop with which the drone communicates was
written in Python, from which I still lack knowledge and experience. At this point, I'm focusing on
controlling and stabilizing the "roll" and "pitch" angles using the PID algorithm. For now, the drone
receives its orientation data from the MPU6050 accelerometer / gyroscope, which asynchronously
calculates the accommodation thanks to a built-in DMP program from the sensor manufacturer.
Below is a video of the drone trying to maintain a 15°angle:
https://drive.google.com/file/d/10MOK0iPmIq1nZwT0Ri85-gEO-PvRt7S6/view?usp=sharing 
