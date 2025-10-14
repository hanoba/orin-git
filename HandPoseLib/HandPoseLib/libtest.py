#!/usr/bin/env python
# coding: utf-8
# File: libtest.py
# Test of HandPoseLib package

from HandPoseLib import HandPose

def libtest():
    handPose = HandPose()
    printCnt = 0
    try:
        while True:
            handFound, x, y = handPose.ProcessFrame()
            if handFound:
                printCnt += 1
                if printCnt == 10:
                    printCnt=0
                    print(f"{x=}, {y=}")
    except KeyboardInterrupt:
        print("\nScript terminated")
        del handPose


if __name__ == '__main__':
    libtest()