# dolpin.py
# Author: Troy Dutton
# Date Modfied: August 1, 2022

from math import dist
from socket import *
import select
import RPi.GPIO as gpio
import time

HOST = "192.168.0.27"
PORT = 12000
SPEED_OF_SOUND = 34300
ULTRASONIC_OUT_PINS = [15]
ULTRASONIC_IN_PINS = [13]
MOTOR_DIR_PINS = []
MOTOR_STEP_PINS = []

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
        gpio.output(trig, gpio.HIGH)
        time.sleep(0.00001)
        gpio.output(trig, gpio.LOW)

        # Start timer and wait for signal to be recieved
        start_t = time.perf_counter()
        gpio.wait_for_edge(echo, gpio.RISING)

        # Stop timer and calculate distance traveled (cm)
        elapsed_t = time.perf_counter() - start_t
        distance = (elapsed_t * SPEED_OF_SOUND) / 2
        distances.append(distance)
    return distances



def main():
    gpioInit()
    server_socket = wifiInit()

    while True:
        r, w, e = select.select([server_socket], [], [], .01)
        if (r): # If there is data available
            print(server_socket.recv(1024).decode())



if __name__ == "__main__":
    main()