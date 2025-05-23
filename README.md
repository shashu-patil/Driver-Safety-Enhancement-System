
#  Drowsiness and Alcohol Detection System with Engine Control (Raspberry Pi)

This project is a real-time driver monitoring system built using **OpenCV**, **MediaPipe**, and **Raspberry Pi GPIO**. It detects drowsiness through Eye Aspect Ratio (EAR) and alcohol presence using a digital alcohol sensor. Upon detection, it alerts the driver using a buzzer and LED and takes safety measures like reducing motor speed or stopping it.

---

##  Features

*  **Drowsiness Detection** using Eye Aspect Ratio (EAR) via MediaPipe Face Mesh.
*  **Alcohol Detection** via digital alcohol sensor (MQ3 or MQ2).
*  Alerts using **Buzzer** and **LED**.
*  **Motor Control**: Slows or stops the vehicle upon detection.

---

##  Hardware Requirements

* Raspberry Pi (any model with GPIO)
* USB Webcam
* L293D Motor Driver
* DC Motor
* Alcohol Sensor (MQ2 or MQ3 with digital output)
* Buzzer
* LED
* Jumper wires
* 12V Power Supply

---

##  Software Requirements

* Python 3.x
* OpenCV
* MediaPipe
* SciPy
* RPi.GPIO

### Install Dependencies

```bash
sudo apt update
sudo apt install python3-pip
pip3 install opencv-python mediapipe scipy RPi.GPIO
```

---

##  Circuit Connections

| Component                    | GPIO Pin |
| ---------------------------- | -------- |
| Buzzer                       | GPIO 18  |
| Motor IN1                    | GPIO 23  |
| Motor IN2                    | GPIO 24  |
| Motor Enable (PWM)           | GPIO 25  |
| LED                          | GPIO 26  |
| Alcohol Sensor (Digital Out) | GPIO 17  |

---

##  Running the Code

```bash
python3 main.py
```

* Press `q` to exit the program.
* Ensure the webcam is connected before running.

---

##  How It Works

1. **Drowsiness**: If the eyes are closed for more than 3 seconds, the system:

   * Sounds the buzzer.
   * Blinks the LED.
   * Reduces motor speed to 50%.

2. **Alcohol**: If alcohol is detected via the sensor:

   * Buzzer is activated continuously.
   * LED blinks.
   * Motor is stopped immediately.


---

##  Author

Made by SHASHANK M PATIL

