#!/usr/bin/env python3
# coding: utf-8

import time
from Rosmaster_Lib import Rosmaster
from HandPoseLib import HandPose
#from pid import PID


if __name__ == '__main__':
    handPose = HandPose()
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
    ymThreshold = 29
    #pidLinear = pid(Kp=1/25, Ki=0, Kd=0, setpoint=ymThreshold, (-0.5,0.5))
    #pidAngular = pid(Kp=1/20, Ki=0, Kd=0, setpoint=0, (-2.5,2.5))

    pLinear = 1/25
    pAngular = 1/20

    try:
        while True:
            handFound, x, ym = handPose.ProcessFrame()
            if handFound:
                e = ym - ymThreshold

                linear = e * pLinear
                linear = max(-0.5, linear)
                linear = min(0.5, linear)
                
                ex = -x
                angular = ex * pAngular
                angular = max(-2.5, angular)
                angular = min(2.5, angular)
                
                printCnt += 1
                if printCnt == 10:
                    printCnt=0
                    print(f"{linear=}, {angular=}, {ex=}, {ym=}")
            else:
                angular = 0
                linear = 0
            # v_x=[-0.7, 0.7] m/s, v_y=[-0.7, 0.7] m/s, v_z=[-3.2, 3.2] rad/sec
            #bot.set_car_motion(v_x=linear, v_y=0, v_z=angular)

    except KeyboardInterrupt:
        print("\nScript terminated")
        del handPose
        bot.set_colorful_effect(effect=0)
        bot.set_car_motion(v_x=0, v_y=0, v_z=0)

