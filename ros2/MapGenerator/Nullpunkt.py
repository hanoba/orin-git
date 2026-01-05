# Berechnung des Nullpunkt im .pgm file
# HB 2026-01-04

import math

# ZaunAL (68,42)   (777,42)
# ZaunRB (795,42)   (955,42)
# ZaunBC (955,42)   (913,528)
# ZaunCD (913,528)   (50,198)
# ZaunDA (50,198)   (68,42)

#  garten=[(0.3, 0), (44.65, 0), (42.53852065760767, -24.283199587811236), (-0.5923901597179264, -7.809176640519705)]
#  Zaunelement /World/ZaunAL created a=array([-21.7,  12. ]), b=array([13.73, 12.  ])
#  Zaunelement /World/ZaunRB created a=array([14.65, 12.  ]), b=array([22.65, 12.  ])
#  Zaunelement /World/ZaunBC created a=array([22.65, 12.  ]), b=array([ 20.53852066, -12.28319959])
#  Zaunelement /World/ZaunCD created a=array([ 20.53852066, -12.28319959]), b=array([-22.59239016,   4.19082336])
#  Zaunelement /World/ZaunDA created a=array([-22.59239016,   4.19082336]), b=array([-21.7,  12. ])
#  /World/Schuppen      Unten links: (-15.80,  6.70), Oben rechts: (-11.80, 11.20)
#  /World/Gartenhaus    Unten links: (  5.17,  6.74), Oben rechts: ( 12.40, 11.41)
#  /World/Terrasse      Unten links: (  6.32,  2.29), Oben rechts: ( 12.40,  6.74)
#  /World/Bassin        Unten links: ( 12.90,  2.29), Oben rechts: ( 15.30,  5.09)


centerX =  22
centerY = -12


# Koordinatentransformation von isaac-sim nach map.pgm (integer result)
def cInt(A, res):
    x, y = A
    x += centerX
    y += centerY
    pt = (int((x+3)/res+2.5), int((-y+2)/res+2.5))
    print(f"{pt=}")
    return pt

# Koordinatentransformation von isaac-sim nach map.pgm (float result)
def cFloat(A, res):
    x, y = A
    x += centerX
    y += centerY
    pt = ((x+3)+2*res, (-y+2)+2*res)
    print(f"{pt=}")
    return pt

# Nullpunkt für map.yaml
def Nullpunkt(res):
    x, y = cInt((0,0), res)
    nx = round(-x*res, 3)
    ny = round(-y*res, 3)
    return nx, ny

def main():
    res = 0.10
    cInt((-21.7,  12.), res)                  # pt=(68, 42)   
    cInt((20.53852066, -12.28319959), res)    # pt=(913, 528)
    nx, ny = Nullpunkt(res)                   # pt=(502, 282)  Nullpunkt
    
    # Nullpunkt für .yaml: (-25.1, -14.100000000000001)   c2((0,0))=(25.1, 14.1)
    print(f"Nullpunkt für .yaml: ({nx}, {ny})   {cFloat((0,0), res)=}")    
    
if __name__ == "__main__":    
    main()
    