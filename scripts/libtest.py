#!/usr/bin/env python3
# coding: utf-8

import time
from Rosmaster_Lib import Rosmaster

def main():
    bot = Rosmaster(com="/dev/ttyCH341USB0", debug=False)
    bot.create_receive_threading()
    
    time.sleep(.1)
    for i in range(3):
        bot.set_beep(60)
        time.sleep(.2)

    print("Version:", bot.get_version())
    print("Vbat:", bot.get_battery_voltage())
    
    # Control the car forward, backward, left, right and other movements.
    # State =[0~6],=0 stop,=1 forward,=2 backward,=3 left,=4 right,=5 spin left,=6 spin right
    # Speed =[-100, 100], =0 Stop.
    # Adjust =True Activate the gyroscope auxiliary motion direction.  If =False, the function is disabled.(This function is not enabled)
    bot.set_car_run(state=1, speed=20)

    # X3PLUS: v_x=[-0.7, 0.7] m/s, v_y=[-0.7, 0.7] m/s, v_z=[-3.2, 3.2] rad/sec
    bot.set_car_motion(v_x=0, v_y=0, v_z=0.314)
    #bot.set_car_motion(v_x=0, v_y=-0.1, v_z=0)
    #bot.set_car_motion(v_x=-0.1, v_y=0, v_z=0)

    # Effect =[0, 6], 0: stop light effect, 1: running light, 2: running horse light, 3: breathing light, 4: gradient light, 5: starlight, 6: power display 
    # Speed =[1, 10], the smaller the value, the faster the speed changes
    # Parm, left blank, as an additional argument.  Usage 1: The color of breathing lamp can be modified by the effect of breathing lamp [0, 6]
    bot.set_colorful_effect(effect=2, speed=5)
    
    try:
        time.sleep(10)
    except KeyboardInterrupt:
        pass

    bot.set_car_run(state=0, speed=0)
    bot.set_colorful_effect(effect=0)

if __name__ == '__main__':
    main()
    exit(0)
