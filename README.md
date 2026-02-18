# DCMotor-speed-control-and-Ultrasonic-sensor-integration-with-Raspberry-Pi-4
This project demonstrates actuator and sensor interfacing using Raspberry Pi 4 Model B and Python.  The system includes:  
✅ DC Motor speed &amp; direction control using L298N Motor Driver Module  
✅ Distance measurement using HC-SR04 Ultrasonic Sensor  
✅ GPIO configuration using BCM numbering  
✅ PWM-based motor control  
✅ Real-time object detection

🚀 DCMotor Speed Control and Ultrasonic Sensor Integration with Raspberry-Pi 4


🧠 System Architecture

The Raspberry Pi acts as:

GPIO Controller

PWM Generator

Sensor Data Processor

Embedded Linux System

🔹 TASK 1 – DC Motor Speed Control using L298N
🎯 Objective

Control:

Motor Speed using PWM

Motor Direction using digital outputs

🔌 GPIO Connections (BCM Mode)
L298N Pin	Raspberry Pi GPIO
IN1	GPIO 23
IN2	GPIO 24
ENA	GPIO 18 (PWM)
GND	GND
12V	External Supply
⚙️ Working Principle

IN1 = HIGH, IN2 = LOW → Forward

IN1 = LOW, IN2 = HIGH → Backward

PWM Duty Cycle → Speed Control

💻 motor_control.py
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

IN1 = 23
IN2 = 24
ENA = 18

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

pwm = GPIO.PWM(ENA, 1000)
pwm.start(0)

def motor_forward(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(speed)

def motor_backward(speed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm.ChangeDutyCycle(speed)

def motor_stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(0)

try:
    while True:
        print("Forward 30%")
        motor_forward(30)
        time.sleep(3)

        print("Forward 70%")
        motor_forward(70)
        time.sleep(3)

        print("Backward 50%")
        motor_backward(50)
        time.sleep(3)

        print("Stop")
        motor_stop()
        time.sleep(2)

except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()

🔹 TASK 2 – Ultrasonic Sensor Integration (HC-SR04)
🎯 Objective

Measure object distance

Display detection message in terminal

🔌 GPIO Connections (BCM Mode)
HC-SR04 Pin	Raspberry Pi GPIO
VCC	5V
GND	GND
TRIG	GPIO 23
ECHO	GPIO 24

⚠️ Important: Use a voltage divider for ECHO (5V → 3.3V safe level).

📡 Distance Formula
Distance = (Pulse Duration × 17150)

💻 ultrasonic_sensor.py
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

TRIG = 23
ECHO = 24

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

GPIO.output(TRIG, False)
time.sleep(2)

DETECTION_DISTANCE = 50

try:
    while True:
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()

        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = round(pulse_duration * 17150, 2)

        if distance <= DETECTION_DISTANCE:
            print(f"Object detected at {distance} cm")
        else:
            print("No object nearby")

        time.sleep(1)

except KeyboardInterrupt:
    GPIO.cleanup()







▶️ How to Run
Run DC Motor Program
python3 motor_control.py

Run Ultrasonic Sensor Program
python3 ultrasonic_sensor.py

📊 Key Learning Outcomes

GPIO configuration using BCM numbering

PWM implementation on Raspberry Pi

H-Bridge motor driver control

Ultrasonic time-of-flight distance calculation

Embedded Linux programming using Python

📝 Conclusion

This project successfully demonstrates:

Hardware-software integration

Real-time actuator control

Sensor-based object detection

Embedded systems development using Raspberry Pi

The integration of DC motor control and ultrasonic sensing provides a strong foundation for robotics and IoT applications.
