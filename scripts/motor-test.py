#!/usr/bin/env python3
# coding: utf-8

import time
from Rosmaster_Lib import Rosmaster
import pygame
import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="pygame.pkgdata")

BG_COLOR = (25, 28, 35)              # Hintergrundfarbe
lineNum = 0
flagTotzonenkompensation = False

def ClearScreen():
    global lineNum
    screen.fill(BG_COLOR)
    lineNum = 0

def Print(txt):
    global lineNum
    screen.blit(font.render(txt, True, (220, 220, 220)), (10, 10+20*lineNum))
    lineNum += 1

def SetMotors(motor, v):
    if flagTotzonenkompensation:
        if v==0: s = 0
        elif v>0: s = 16 + (100 - 16) * v/100
        else: s = -20 + (100 - 20) * v/100
    else: s = v
    bot.set_motor(speed_1=s*motor[0], speed_2=s*motor[1], speed_3=s*motor[2], speed_4=s*motor[3])


bot = Rosmaster(com="/dev/ttyCH341USB0", debug=False)
bot.create_receive_threading()  # Empfangsthread für Statuswerte starten

# Control PWM pulse of motor to control speed (speed measurement without encoder). speed_X=[-100, 100]
# Motoren: 1=links-vorne, 2=links-hinten, 3=rechts-vorne, 4=rechtshinten

pygame.init()
screen = pygame.display.set_mode((200, 200))
pygame.display.set_caption("Motor Test")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 18)

s=14
motor=[1,1,1,1]
SetMotors(motor,s)

running = True
while running:
    ClearScreen()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                s += 1
            elif event.key == pygame.K_DOWN:
                s -= 1
            elif event.key == pygame.K_9:
                s = 90
            elif event.key == pygame.K_8:
                s = 80
            elif event.key == pygame.K_7:
                s = 70
            elif event.key == pygame.K_6:
                s = 60
            elif event.key == pygame.K_5:
                s = 50
            elif event.key == pygame.K_4:
                s = 40
            elif event.key == pygame.K_3:
                s = 30
            elif event.key == pygame.K_2:
                s = 20
            elif event.key == pygame.K_1:
                s = 10
            elif event.key == pygame.K_0:
                s = 0
            elif event.key == pygame.K_F1:
                motor[0] = 1-motor[0]
            elif event.key == pygame.K_F2:
                motor[1] = 1-motor[1]
            elif event.key == pygame.K_F3:
                motor[2] = 1-motor[2]
            elif event.key == pygame.K_F4:
                motor[3] = 1-motor[3]
            elif event.key == pygame.K_a:
                motor=[1,1,1,1]
            elif event.key == pygame.K_s:
                motor=[0,0,0,0]
            elif event.key == pygame.K_t:
                flagTotzonenkompensation = not flagTotzonenkompensation            
            elif event.key == pygame.K_q:
                running = False            

    SetMotors(motor,s)
    Print(f"Duty cycle: {s}%")
    for i in range(4):
        onOff = "on" if motor[i] else "off"
        Print(f"Motor{i+1}: {onOff}")

    Print(f"Totzonenkompensation: {flagTotzonenkompensation}")
    # Framerate begrenzen
    clock.tick(10)  # 10 FPS
    fps = clock.get_fps()
    Print(f"{fps:.0f} fps")
    
    pygame.display.flip()
    
SetMotors(motor, 0)
print("Motor test stopped")
pygame.quit()

#try:
#    while True:
#        if keyboard.is_pressed('+'):
#        elif keyboard.is_pressed('-'):
#            s -= 1
#            print("Duty cycle: {s}%")
#        elif keyboard.is_pressed('q'):
#            break
#        time.sleep(0.1)
#except KeyboardInterrupt:
#    bot.set_motor(speed_1=0, speed_2=0, speed_3=0, speed_4=0)
#    print("Motor test stopped")
