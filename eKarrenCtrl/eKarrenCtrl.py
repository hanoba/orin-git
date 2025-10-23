#!/usr/bin/env python3
# coding: utf-8

import cv2
import time
from Rosmaster_Lib import Rosmaster
from AsyncHandPoseYolo11 import HandPoseApp
from pid import PID


    
    
if __name__ == '__main__':
    bot = Rosmaster(com="/dev/ttyCH341USB0", debug=False)
    bot.create_receive_threading()
    
    time.sleep(.1)
    for i in range(3):
        bot.set_beep(60)
        time.sleep(.2)

    print("Version:", bot.get_version())
    print("Vbat:", bot.get_battery_voltage())
    
    # Effect =[0, 6], 0: stop light effect, 1: running light, 2: running horse light,
    #                 3: breathing light, 4: gradient light, 5: starlight, 6: power display 
    #                 Speed =[1, 10], the smaller the value, the faster the speed changes
    # bot.set_colorful_effect(effect=2, speed=5)

    printCnt = 0
    angular = 0
    linear = 0
    wantedHandSize = 200
    #pidLinear = pid(Kp=1/25, Ki=0, Kd=0, setpoint=ymThreshold, (-0.5,0.5))
    #pidAngular = pid(Kp=1/20, Ki=0, Kd=0, setpoint=0, (-2.5,2.5))

    Ku = 0.027 / 2      # Kritische Verstärkung (engl. ultimate gain)
    Tu = 2.0            # Kritische Periodendauer (engl. ultimate period) in sec
    KpLinear = 0.5*Ku
    KiLinear = 0.01 / 2
    pAngular = 1/150    # 1/20
    dt = 0.03           # sec

    pidLinear = PID(KpLinear, KiLinear, 0, setpoint=wantedHandSize, output_limits=(-0.5, 0.5))
    ###cam = Cam(CAM_ID)
    
    CAM_ID = 2
    handPoseApp = HandPoseApp(CAM_ID)

    lastTime = time.time()

    try:
        while True:
            newTime = time.time()
            deltaTime_ms = (newTime - lastTime)*1000
            lastTime = newTime
            if deltaTime_ms > 1000: deltaTime_ms = 0

            exitFlag, handSize, handPos = handPoseApp.ProcessFrame()
            if exitFlag: break
            
            if handSize < 0: break
            elif handSize > 0:
                linear = pidLinear.update(handSize, dt)
                
                #eHandSize = wantedHandSize - handSize
                #linear = -eHandSize * pLinear
                #linear = max(-0.5, linear)
                #linear = min(0.5, linear)
                
                eHandPos = 0 - handPos
                angular = eHandPos * pAngular
                angular = max(-2.5, angular)
                angular = min(2.5, angular)                
            else:
                angular = 0
                linear = 0
            # v_x=[-0.7, 0.7] m/s, v_y=[-0.7, 0.7] m/s, v_z=[-3.2, 3.2] rad/sec
            bot.set_car_motion(v_x=-linear, v_y=0, v_z=-angular)

            printCnt += 1
            if printCnt >= 10:
                printCnt=0
                Vbat = bot.get_battery_voltage()
                print(f"{deltaTime_ms:.0f} ms lin={linear:.3f}, ang={angular:.3f}, {handSize=}, {handPos=}, {Vbat=}")

    except KeyboardInterrupt:
        print("\nScript terminated")

    bot.set_colorful_effect(effect=0)
    bot.set_car_motion(v_x=0, v_y=0, v_z=0)
    handPoseApp.Exit()
