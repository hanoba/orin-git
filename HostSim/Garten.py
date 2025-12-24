# Erzeugt isaac-sim Simulationsumgebung für meinen Garten.
# Die Gartendaten sind von Moonlight/amcp übernommen.
# HB 2025-12-20

import numpy as np
import math
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
#from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
#from isaacsim.robot.wheeled_robots.robots import WheeledRobot
#from isaacsim.sensors.physx import RotatingLidarPhysX
#from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema, Usd, UsdLux, Vt

import omni.usd
import omni.kit.viewport.window as vp_win
import omni.kit.viewport.window as viewport_api
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.prims import XFormPrim

from Fence import CreateFence, CreateWalls, CreateCube, CreateCylinder, Fence, CreateRotatedCube

def Move(poly, mx, my):
    length = len(poly)
    for i in range(length):
        (x,y) = poly[i]
        poly[i] = (x+mx,y+my)
    return


def Gartenzaun():
    # Messwerte Bosch Laserentfernungsmesser
    a0 = 18.73
    b0 = 15.16
    c0 =  7.86 
    m1 = 30.54
    b2 = 44.35
    c2 = 24.25
    b3 =  0.47
    
    # Dreieck D0
    wa0 = math.acos((b0*b0 + c0*c0 - a0*a0)/(2*b0*c0))
    #print("wa0 =", wa0/math.pi*180)
    
    # Dreieck D1
    wa1 = wa0
    b1 = b0 + m1
    c1 = c0
    a1 = math.sqrt(b1*b1 + c1*c1 - 2*b1*c1*math.cos(wa1))
    #print("a1 =", a1)
    wc1 = math.acos((a1*a1 + b1*b1 - c1*c1)/(2*a1*b1))
    #print("wc1 =", wc1/math.pi*180)
    
    # Dreieck D2
    a2 = a1
    wa2 = math.acos((b2*b2 + c2*c2 - a2*a2)/(2*b2*c2))
    #print("wa2 =", wa2/math.pi*180)
    wb2 = math.acos((a2*a2 + c2*c2 - b2*b2)/(2*a2*c2))
    #print("wb2 =", wb2/math.pi*180)
    
    # Dreieck D3
    c3 = c2
    wa3 = math.pi - wc1 - wb2
    a3 = math.sqrt(b3*b3 + c3*c3 - 2*b3*c3*math.cos(wa3))
    #print("a3 =", a3)
    wb3 = math.acos((a3*a3 + c3*c3 - b3*b3)/(2*a3*c3))
    #print("wb3 =", wb3/math.pi*180)
    
    # Viereck
    wpb = wa2 + wb3
    #print("wpb =", wpb/math.pi*180)
    wpa = 2*math.pi - wa0 - wpb - (math.pi - wa3 - wb3)
    #print("wpa =", wpa/math.pi*180)
    
    
    xo = 0.3
    PA=(xo,0)
    PB=(xo+b2,0)
    
    wb = wpb - math.pi/2
    PC=(xo+b2+a3*math.sin(wb), -a3*math.cos(wb))
    
    wa = wpa - math.pi/2
    PD=(xo-c0*math.sin(wa), -c0*math.cos(wa))

    check=math.sqrt((PD[0]-PC[0])**2+(PD[1]-PC[1])**2)
    #print(check, b1+b3)

    garten=[PA, PB, PC, PD]
    print(f"{garten=}")
    return garten

def Gartenhaus():
    haus = [(0,0),(7.23,0),(7.23,4.67),(0,4.67)]
    mx = 44.70 - 10.30 - 7.23 # 5.10 + 4.00 + 16.60
    my = -4.76-0.5
    Move(haus,mx,my)
    return haus

def Terrasse():
    xx = 7.23-6.08
    #yy = 4.67
    mx = 44.70 - 10.30 - 7.23 # 5.10 + 4.00 + 16.60
    my = -4.76-0.5
    t = [(xx,0),(xx+6.08,0),(xx+6.08,-4.45),(xx,-4.45)]
    Move(t, mx,my)
    return [t[3], t[2], t[1], t[0]]
    
def Schuppen():
    schuppen = [(0,0),(4.00,0),(4.00,4.50),(0,4.50)]
    mx = 5.10 + 1.6 - 0.5
    my = -4.50-0.50 - 0.3
    Move(schuppen,mx,my)
    return schuppen


# -------------------------------------------------------
# Garten
# -------------------------------------------------------
def CreateGarten(stage):
    centerX =  22
    centerY = -12
    def ZaunElement(stage, primName, A, B):
        center = np.array([centerX, centerY])
        a = np.array(A) - center
        b = np.array(B) - center
        m = (a + b)/2 
        c = b - a
        angle = math.degrees(math.atan2(c[1], c[0]))
        dist = math.dist(a, b)
    
        #fence = Fence(stage, primName, width=dist, height=H, radius=R, step=S)
        CreateRotatedCube(stage, primName, posX=m[0], posY=m[1], lenX=dist, lenY=0.1, height=1.6, angle_deg=angle)
        print(f"Zaunelement {primName} created {a=}, {b=}")
        #f = UsdGeom.XformCommonAPI(stage.GetPrimAtPath(primName))
        #f.SetRotate((0, 0, angle))
        #f.SetTranslate((m[0], m[1], 0.0))
        
    PA, PB, PC, PD = Gartenzaun()
    # Gartentor hinzufügen
    torAbstand = 8.00
    torBreite = 0.92
    PL = (PB[0]-torAbstand-torBreite, PB[1])    # linker Torpfosten
    PR = (PB[0]-torAbstand, PB[1])              # rechter Torpfosten
    ZaunElement(stage, "/World/ZaunAL", PA, PL)
    ZaunElement(stage, "/World/ZaunRB", PR, PB)
    ZaunElement(stage, "/World/ZaunBC", PB, PC)
    ZaunElement(stage, "/World/ZaunCD", PC, PD)
    ZaunElement(stage, "/World/ZaunDA", PD, PA)
    
    def Haus(stage, prim_path, eckPunkte):
        posX1, posY1 = eckPunkte[0]
        posX2, posY2 = eckPunkte[2]
        lenX = posX2 - posX1
        lenY = posY2 - posY1
        posX = posX1 - centerX
        posY = posY1 - centerY
        print(f"{prim_path:<20} Unten links: ({posX:6.2f},{posY:6.2f}), Oben rechts: ({posX+lenX:6.2f},{posY+lenY:6.2f})")
        CreateCube(stage, prim_path, posX, posY, lenX, lenY, height=1.6)
    
    Haus(stage, "/World/Schuppen", Schuppen())
    Haus(stage, "/World/Gartenhaus", Gartenhaus())
    terrasse = Terrasse()
    Haus(stage, "/World/Terrasse", terrasse)
    
    def Bassin(terrasseUntenRechts):
        lenX = 2.40
        lenY = 2.80
        (x, y) = terrasseUntenRechts
        x += 0.50
        return [(x, y), (x+lenX, y), (x+lenX, y+lenY), (x, y+lenY)]
    
    Haus(stage, "/World/Bassin", Bassin(terrasse[1]))
    
    def Strauch(stage, prim_path, mittelPunkt, dm):
        posX, posY = mittelPunkt
        CreateCylinder(stage, prim_path, posX-centerX, posY-centerY, dm, height=1.6)
        # Farbe als RGB definieren (Werte von 0.0 bis 1.0)
        # Hier ein Grün für den Strauch:
        farbe = Gf.Vec3f(0.0, 1.0, 0.0)

        # Das Attribut erstellen oder holen und setzen
        # Wir müssen es in eine Liste [] packen, da displayColor ein Array ist
        prim = stage.GetPrimAtPath(prim_path)
        prim.CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(Vt.Vec3fArray([farbe]))

    
    tx, ty = terrasse[0]
    grasMP = (tx, ty-8.0)
    grasDM = 1.50
    Strauch(stage, "/World/Gras", grasMP, grasDM)

    strauchMP = (tx+13.00, ty-8.00)
    strauchDM = 2.00    
    Strauch(stage, "/World/Strauch", strauchMP, strauchDM)
