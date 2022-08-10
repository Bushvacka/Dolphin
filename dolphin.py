# dolpin.py
# Author: Troy Dutton
# Date Modfied: August 1, 2022

from socket import *
import select
import RPi.GPIO as gpio
import time, sys

HOST = "192.168.0.27"
PORT = 12000
SPEED_OF_SOUND = 34300
ULTRASONIC_OUT_PINS = [3]
ULTRASONIC_IN_PINS = [5]
MOTOR_DIR_PINS = []
MOTOR_STEP_PINS = []
LOW = gpio.LOW
HIGH = gpio.HIGH

def wifiInit():
    server_socket = socket(AF_INET, SOCK_STREAM)
    server_socket.connect((HOST, PORT))
    return server_socket

def gpioInit():
    gpio.setmode(gpio.BOARD)

    # Setup ultrasonic sensors
    for pin in ULTRASONIC_OUT_PINS:
        gpio.setup(pin, gpio.OUT)
    
    for pin in ULTRASONIC_IN_PINS:
        gpio.setup(pin, gpio.IN)

    # Setup stepper motors
    for pin in MOTOR_DIR_PINS:
        gpio.setup(pin, gpio.OUT)
    
    for pin in MOTOR_STEP_PINS:
        gpio.setup(pin, gpio.OUT)

def measureDistance():
    distances = []
    for trig, echo in zip(ULTRASONIC_OUT_PINS, ULTRASONIC_IN_PINS):
        # Pulse trigger pin
        gpio.output(trig, HIGH)
        time.sleep(0.00001)
        gpio.output(trig, LOW)

        # Wait for low signal on input
        while (gpio.input(echo)):
            pass
        
        # Start timer and wait for signal to be recieved
        start_t = time.perf_counter()
        gpio.wait_for_edge(echo, gpio.RISING)

        # Stop timer and calculate distance traveled (cm)
        elapsed_t = time.perf_counter() - start_t
        distance = (elapsed_t * SPEED_OF_SOUND) / 2
        distances.append(distance)
    return distances

def setMotorDirection(dir):
    if (dir == "W"):
        for dir_pin in MOTOR_DIR_PINS:
            gpio.output(dir_pin, HIGH)
    elif (dir == "S"):
        for dir_pin in MOTOR_DIR_PINS:
            gpio.output(dir_pin, LOW)
    elif (dir == "A"):
        MOTOR_DIR_PINS[0] = LOW
        MOTOR_DIR_PINS[1] = HIGH
    elif (dir == "D"):
        MOTOR_DIR_PINS[0] = HIGH
        MOTOR_DIR_PINS[1] = LOW
    

def main():
    gpioInit()
    server_socket = wifiInit()

    while True:
        r, w, e = select.select([server_socket], [], [], .01)
        if (r): # If there is data available
            msg = server_socket.recv(1024).decode()
            print(f"Message Recieved: {msg}")
            setMotorDirection(msg)
            print(f"Sensor Distance: {measureDistance()}")



# TODO Remove
AIN1 = 7
AIN2 = 8
PWA = 10

FREQ = 100
DUTY_CYCLE = 50
if __name__ == "__main__":
    try:
        # TODO Remove
        gpio.setup(AIN1, gpio.OUTPUT)
        gpio.setup(AIN2, gpio.OUTPUT)
        gpio.setup(PWA, gpio.OUTPUT)

        pwm = gpio.PWM(PWA, FREQ)
        pwm.start(DUTY_CYCLE)

        gpio.output(AIN1, LOW)
        gpio.output(AIN2, HIGH)

        while True:
            time.sleep(2)
            gpio.output(AIN1, HIGH)
            gpio.output(AIN2, LOW)
            time.sleep(2)
            gpio.output(AIN1, LOW)
            gpio.output(AIN2, HIGH)

        # main()
    except KeyboardInterrupt:
        gpio.cleanup()
        sys.exit(0)