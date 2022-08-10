# dolpin.py
# Author: Troy Dutton
# Date Modfied: August 10, 2022

from socket import *
import select
import RPi.GPIO as GPIO
import GPIO.LOW as LOW
import GPIO.HIGH as HIGH
import time, sys

HOST = "192.168.0.27"
PORT = 12000
SPEED_OF_SOUND = 34300
ULTRASONIC_OUT_PINS = [3]
ULTRASONIC_IN_PINS = [5]
MOTOR_DIR_PINS = []
MOTOR_STEP_PINS = []

# TODO Remove

AIN1 = 7
AIN2 = 8
PWA = 12

FREQ = 500
dc = 50

def wifiInit():
    server_socket = socket(AF_INET, SOCK_STREAM)
    server_socket.connect((HOST, PORT))
    return server_socket

def GPIOInit():
    GPIO.setmode(GPIO.BOARD)

    # Setup ultrasonic sensors
    for pin in ULTRASONIC_OUT_PINS:
        GPIO.setup(pin, GPIO.OUT)
    
    for pin in ULTRASONIC_IN_PINS:
        GPIO.setup(pin, GPIO.IN)

    # Setup motors
    for pin in MOTOR_DIR_PINS:
        GPIO.setup(pin, GPIO.OUT)
    
    for pin in MOTOR_STEP_PINS:
        GPIO.setup(pin, GPIO.OUT)

def measureDistance():
    distances = []
    for trig, echo in zip(ULTRASONIC_OUT_PINS, ULTRASONIC_IN_PINS):
        # Pulse trigger pin
        GPIO.output(trig, HIGH)
        time.sleep(0.00001)
        GPIO.output(trig, LOW)

        # Wait for low signal on input
        while (GPIO.input(echo)):
            pass
        
        # Start timer and wait for signal to be recieved
        start_t = time.perf_counter()
        GPIO.wait_for_edge(echo, GPIO.RISING)

        # Stop timer and calculate distance traveled (cm)
        elapsed_t = time.perf_counter() - start_t
        distance = (elapsed_t * SPEED_OF_SOUND) / 2
        distances.append(distance)
    return distances

def setMotorDirection(dir):
    if (dir == "W"):
        for motor_pins in MOTOR_DIR_PINS:
            GPIO.output(motor_pins[0], HIGH)
            GPIO.output(motor_pins[1], LOW)
    elif (dir == "S"):
        for dir_pin in MOTOR_DIR_PINS:
            GPIO.output(dir_pin, LOW)
    elif (dir == "A"):
        MOTOR_DIR_PINS[0] = LOW
        MOTOR_DIR_PINS[1] = HIGH
    elif (dir == "D"):
        MOTOR_DIR_PINS[0] = HIGH
        MOTOR_DIR_PINS[1] = LOW
    

def main():
    #GPIOInit()
    server_socket = wifiInit()

    while True:
        # TODO Remove
        GPIO.setup(AIN1, GPIO.OUTPUT)
        GPIO.setup(AIN2, GPIO.OUTPUT)
        GPIO.setup(PWA, GPIO.OUTPUT)

        pwm = GPIO.PWM(PWA, FREQ)
        pwm.start(dc)

        GPIO.output(AIN1, LOW)
        GPIO.output(AIN2, HIGH)

        r, w, e = select.select([server_socket], [], [], .01)
        if (r): # If there is data available
            msg = server_socket.recv(1024).decode()
            print(f"Message Recieved: {msg}")
            if msg == "W":
                dc += 5
            elif msg == "S":
                dc -= 5
            pwm.ChangeDutyCycle(dc)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup()
        sys.exit(0)