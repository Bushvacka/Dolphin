# Controller.py
# Author: Troy Dutton
# Date Modfied: August 1, 2022

from socket import *
import select
import pygame, sys, time, math

PORT = 12000
FPS_CAP = 60
SCAN_COLOR = (0, 255, 0)
DOLPHIN_COLOR = (0, 0, 255)

def displayInit():
    # Initialize pygame
    pygame.init()
    pygame.font.init()

    # Initialize display
    w = pygame.display.Info().current_w if pygame.display.Info().current_w <= 1920 else 1920
    h = pygame.display.Info().current_h if pygame.display.Info().current_h <= 1080 else 1080
    screen = pygame.display.set_mode((w, h))
    pygame.display.set_caption("Dolphin")
    return (screen, w, h)

def drawScan(screen, w, h):
    """Draw up-to-date scan information to the screen"""
    for point in scan:
        # Adjust for camera position
        x = point[0] - camera[0] + w/2
        y = point[1] - camera[1] + h/2
        pygame.draw.circle(screen, SCAN_COLOR, (x, y), 7)

def drawDolphin(screen, w, h):
    """Indicate the position and direction of the dolphin"""
    x = dolphin[0] - camera[0] + w/2
    y = dolphin[1] - camera[1] + h/2
    dolphin_surface = pygame.Surface((50, 20))
    pygame.draw.circle(dolphin_surface, DOLPHIN_COLOR, (25, 10), 10)
    pygame.draw.line(dolphin_surface, DOLPHIN_COLOR, (25, 10), (45, 10), 5)
    pygame.draw.polygon(dolphin_surface, DOLPHIN_COLOR, ((45, 5), (45, 15), (50, 10)))
    dolphin_surface.convert_alpha() # Enable alpha
    dolphin_surface.set_colorkey((0, 0, 0)) # Black alpha
    # Rotate about the center of the circle
    rotated_surface = pygame.transform.rotate(dolphin_surface, dolphin_angle)
    dolphin_rect = rotated_surface.get_rect(center = dolphin_surface.get_rect().center)
    screen.blit(rotated_surface, (dolphin_rect[0] + x - 25, dolphin_rect[1] + y - 10))

def updateDisplay(screen, w, h):
    """Draw to and refresh the display"""
    screen.fill((0, 0, 0)) # Clear display
    drawScan(screen, w, h)
    drawDolphin(screen, w, h)
    pygame.display.update() # Push changes to the screen


def wifiInit():
    # Connect to dolphin
    s = socket(AF_INET, SOCK_STREAM)
    s.bind(('', PORT))
    s.listen(1)
    dolphin_socket, addr = s.accept()
    return dolphin_socket


screen, w, h = displayInit()
dolphin_socket = wifiInit()

# Position information
dolphin = [0, 0]
dolphin_angle = 0
scan = []

# Display information
camera = [0, 0]
camera_drag = False

# Timing 
last_update = 0
last_frame = 0

while True:
    # Check for new events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if pygame.mouse.get_pressed()[0]:
                camera_drag = True
        elif event.type == pygame.MOUSEBUTTONUP:
            if pygame.mouse.get_pressed()[0] == False:
                camera_drag = False
        elif event.type == pygame.MOUSEMOTION:
            if camera_drag: # Update camera position
                camera = [old - change for (old, change) in zip(camera, list(pygame.mouse.get_rel()))]
            else: # Clear motion from queue
                pygame.mouse.get_rel()
        
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                dolphin[0] += math.cos(math.radians(dolphin_angle))
                dolphin[1] -= math.sin(math.radians(dolphin_angle))
                dolphin_socket.send("W".encode())
            elif event.key == pygame.K_a:
                dolphin_angle -= 1
                dolphin_socket.send("A".encode())
            elif event.key == pygame.K_s:
                dolphin[0] -= math.cos(math.radians(dolphin_angle))
                dolphin[1] += math.sin(math.radians(dolphin_angle))
                dolphin_socket.send("S".encode())
            elif event.key == pygame.K_d:
                dolphin_angle += 1
                dolphin_socket.send("D".encode())

    read, write, error = select.select([dolphin_socket], [], [], .01)
    if (read): # If there is data available
        print(dolphin_socket.recv(1024).decode())
        
    # Update Display
    if (int(round(time.time() * 1000)) - last_frame) > (1000/FPS_CAP):
        updateDisplay(screen, w, h)
        last_frame = int(round(time.time() * 1000))
