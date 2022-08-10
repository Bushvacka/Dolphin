# dolpin.py
# Author: Troy Dutton
# Date Modfied: August 10, 2022

from socket import *
import select
import RPi.GPIO as GPIO
from RPi.GPIO import LOW, HIGH
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
PWA = 10

BIN1 = 11
BIN2 = 12
PWB = 14

FREQ = 500
DUTY_CYCLE = 50

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
        
    GPIO.setup(AIN1, GPIO.OUT)    
    GPIO.setup(AIN2, GPIO.OUT)
    GPIO.setup(PWA, GPIO.OUT)
    return GPIO.PWM(PWA, FREQ)

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

def setMotorDirection(dirs):
    GPIO.output(AIN1, dirs[0])
    GPIO.output(AIN2, dirs[1])

def generateTask(cmd):
    if (cmd == "W"):
        task = [[HIGH, LOW], time.time()]
    elif (cmd == "S"):
        task = [[LOW, HIGH], time.time()]
    return task
    

def main():
    pwm = GPIOInit()    
    server_socket = wifiInit()
    tasks = []

    while True:
        # End any out-of-time tasks
        if len(tasks) > 0:
            if (time.time() - tasks[0][1]) > 0.5:
                tasks.pop(0)
                # Check if there are any more tasks
                if len(tasks) > 0:
                    setMotorDirection(tasks[0][0])
                    tasks[0][1] = time.time() # Update start time
                else:
                    pwm.stop()
        
        # Check for commands
        r, w, e = select.select([server_socket], [], [], .01)
        if (r): # If there is data available
            cmd = server_socket.recv(1024).decode()
            tasks.append(generateTask(cmd))
            if len(tasks) == 1:
                setMotorDirection(tasks[0][0])
                pwm.start(DUTY_CYCLE)
            print(f"Message Recieved: {cmd}")
            print(f"Tasks: {tasks}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup()
        sys.exit(0)
