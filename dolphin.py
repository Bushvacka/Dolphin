# dolpin.py
# Author: Troy Dutton
# Date Modfied: August 10, 2022

from http import server
from socket import *
import select
import RPi.GPIO as GPIO
from RPi.GPIO import LOW, HIGH
import time, sys, math

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

    GPIO.setup(BIN1, GPIO.OUT)    
    GPIO.setup(BIN2, GPIO.OUT)
    GPIO.setup(PWB, GPIO.OUT)
    return [GPIO.PWM(PWA, FREQ), GPIO.PWM(PWA, FREQ)]

def measureDistance():
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
    return distance

def setMotorDirection(dirs):
    GPIO.output(AIN1, dirs[0])
    GPIO.output(AIN2, dirs[1])
    GPIO.output(BIN1, dirs[2])
    GPIO.output(BIN2, dirs[3])


def generateTask(cmd):
    if (cmd == "W"):
        task = [[HIGH, LOW, HIGH, LOW], time.time(), [5, 0]]
    elif (cmd == "S"):
        task = [[LOW, HIGH, LOW, HIGH], time.time(), [-5, 0]]
    return task
    

def main():
    pwm = GPIOInit()    
    server_socket = wifiInit()
    pos = [0, 0]
    angle = 0
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
                    pos[0] += tasks[0][2][0]
                    angle += tasks[0][2][1]
                else:
                    pwm.stop()
        
        # Check for commands
        r, w, e = select.select([server_socket], [], [], .01)
        if (r): # If there is data available
            cmd = server_socket.recv(1024).decode()
            if cmd in ["W", "A", "S", "D"]:
                tasks.append(generateTask(cmd))
                if len(tasks) == 1:
                    setMotorDirection(tasks[0][0])
                    pwm.start(DUTY_CYCLE)
            elif cmd == "G":
                dist = measureDistance()
                point = [pos[0] + dist*math.cos(math.radians(angle)), pos[1] + dist*math.sin(math.radians(angle))]
                msg = str(point[0]) + "," + str(point[1])
                server_socket.send(msg.encode())
            print(f"Message Recieved: {cmd}")
            print(f"Pos: {pos}\nAngle: {angle}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup()
        sys.exit(0)
