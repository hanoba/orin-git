import time

printCnt = 0
printMax = 20
t_prev = time.time()

def Print():
    global printCnt, t_prev
    printCnt += 1
    if printCnt >= printMax:
        printCnt=0
        # CPU-FPS messen (reines GUI/Loop-Tempo)
        now = time.time()
        dt = (now - t_prev)
        deltaTime_ms = dt * 1000 / printMax
        t_prev = now
        #fps_cpu = ema(fps_cpu, 1.0 / dt if dt > 0 else 0.0)
        print(f"{deltaTime_ms:.0f} ms")
