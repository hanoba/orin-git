#!/usr/bin/env python3
# coding: utf-8

import cv2
import time
from Rosmaster_Lib import Rosmaster
from HandPoseYolo11 import HandPose, GetScreenSize
from pid import PID

class Cam():
    def __init__(self, CAM_ID):
        # === Main Loop ===
        self.cap = cv2.VideoCapture(CAM_ID)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {CAM_ID}")

        GetScreenSize()
        
        # Get frame width and height from camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        width  = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"Camera {width=} {height=}")

        self.win = "YOLO11n Pose (TensorRT)"
        #cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
        #cv2.setWindowProperty(self.win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        return

    def ProcessFrame(self):
        ret, frame = self.cap.read()
        if not ret:
            return -1, 0

        frame, handSize, handPos = handPose.ProcessFrame(frame)

        #cv2.putText(frame, f"{deltaTime_ms:.0f} ms {handSize}, {handPos}", (10, 30),
        #            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)


        #cv2.imshow(self.win, frame)
        #if cv2.waitKey(1) & 0xFF == 27:  # ESC
        #    return -1, 0
            
        #cv2.putText(frame, f"{(t1 - t0)*1000:.1f} ms", (10, 30),
        #            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        return handSize, handPos
        
    def Release(self):
        self.cap.release()
    
    
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
    wantedHandSize = 170
    #pidLinear = pid(Kp=1/25, Ki=0, Kd=0, setpoint=ymThreshold, (-0.5,0.5))
    #pidAngular = pid(Kp=1/20, Ki=0, Kd=0, setpoint=0, (-2.5,2.5))

    Ku = 0.027          # Kritische Verstärkung (engl. ultimate gain)
    Tu = 2.0            # Kritische Periodendauer (engl. ultimate period) in sec
    KpLinear = 0.5*Ku
    KiLinear = 0.01
    pAngular = 1/150    # 1/20
    CAM_ID = 2
    dt = 0.03           # sec

    pidLinear = PID(KpLinear, KiLinear, 0, setpoint=wantedHandSize, output_limits=(-0.5, 0.5))
    cam = Cam(CAM_ID)
    
    lastTime = time.time()

    try:
        while True:
            newTime = time.time()
            deltaTime_ms = (newTime - lastTime)*1000
            lastTime = newTime
            if deltaTime_ms > 1000: deltaTime_ms = 0

            handSize, handPos = cam.ProcessFrame()
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
                print(f"{deltaTime_ms:.0f} ms lin={linear:.3f}, ang={angular:.3f}, {handSize=}, {handPos=}")

    except KeyboardInterrupt:
        print("\nScript terminated")

    bot.set_colorful_effect(effect=0)
    bot.set_car_motion(v_x=0, v_y=0, v_z=0)
    cam.Release()
