# Erzeugt pygame Simulationsumgebung für meinen Garten.
# Die Gartendaten sind von Moonlight/amcp übernommen.
# HB 2025-12-20

import numpy as np
import math
import pygame
from dataclasses import dataclass

# Weltgeometrie
BORDER_MARGIN = 10      # 40                   
WALL_X = 500
WIN_W, WIN_H = 1500, 1000              # Fenstergröße in Pixeln
WIN_W_METER = 50.0

CenterX =  22
CenterY = -12
Center = np.array([CenterX, CenterY])


WALL_COLOR = (180, 180, 180)         # Farbe der Wände

MetersPerPixel = WIN_W_METER / WIN_W
PixelsPerMeter = 1 / MetersPerPixel
PixelOffsetX = WIN_W/2 + BORDER_MARGIN
PixelOffsetY = WIN_H/2 - BORDER_MARGIN

# Linien (Walls), die für die Positionsbestimmung verwendet werden, 
# werden wie folgt gespeichert: 0 = a*x + b*y + c 
# wobei:  a = lineMatrix[i,0] 
#         b = lineMatrix[i,1]
#         c = lineMatrix[i,2]
# Für horizontale Linien gilt: a=0, b=-1, c=Ax=Bx
# Für vertikale Linien gilt: a=-1, b=0, c=Ax=Bx
# 
LineLengthMargin = 0.50
LineMatrixRows = 12
LineMatrixCols =  3
lineMatrix = np.zeros((LineMatrixRows, LineMatrixCols))
linePosition = [0]*LineMatrixRows
lineMin = [0.0]*LineMatrixRows
lineMax = [0.0]*LineMatrixRows
lineTheta = [0.0]*LineMatrixRows
lineMaxDist = [np.inf]*LineMatrixRows
lineNames = ["ZaunN","ZaunO","ZaunS","ZaunW","SchuppenW","SchuppenS","SchuppenO","TerrasseW","TerrasseS","BassinO","BassinN","HausO"]
#            [ZN,ZO,ZS,ZW,SW,SS,SO,TW,TS,BO,BN,HO]
lineIgnore = [ 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1]

# Wall Position relative to robot
P_NORTH = 0
P_EAST  = 1
P_SOUTH = 2
P_WEST  = 3
P_TEXT = ["North", "East", "South", "West"]
P_SIGN = [ 1, 1, -1, -1 ]


def GetWallPosY(wallNum, x=0):
    """ Berechnet den Y-Wert für die Wand/Linie wallNum """
    a=lineMatrix[wallNum][0]
    b=lineMatrix[wallNum][1]
    c=lineMatrix[wallNum][2]
    return -(a*x + c)/b

def GetWallPosX(wallNum, y=0):
    """ Berechnet den Y-Wert für die Wand/Linie wallNum """
    a=lineMatrix[wallNum][0]
    b=lineMatrix[wallNum][1]
    c=lineMatrix[wallNum][2]
    return -(b*y + c)/a

def AddLineForPositioning(num, A, B, pos, minLength=None, maxDist=np.inf):
    a = np.array(A) - Center
    b = np.array(B) - Center
    if a[0] == b[0]:
        # Vertikale Linie
        lineMatrix[num, 0] = -1.0
        lineMatrix[num, 1] =  0.0
        lineMatrix[num, 2] = a[0]
    elif a[1] == b[1]:
        # Horizontale Linie
        lineMatrix[num, 0] =  0.0
        lineMatrix[num, 1] = -1.0
        lineMatrix[num, 2] = b[1]
    elif pos==P_EAST or pos==P_WEST:
        m, t = LineEquation(a, b)
        lineMatrix[num, 0] = -1.0
        lineMatrix[num, 1] = 1/m
        lineMatrix[num, 2] = -t/m
    else:
        m, t = LineEquation(a, b)
        lineMatrix[num, 0] = m
        lineMatrix[num, 1] = -1.0
        lineMatrix[num, 2] = t
    if minLength == None:
        vecLen = np.linalg.norm(a-b)
        lineMin[num] = vecLen - LineLengthMargin*2.0
        lineMax[num] = vecLen + LineLengthMargin*0.3
    else:
        lineMin[num] = minLength
        lineMax[num] = math.inf
    lineTheta[num] = Angle(a, b)
    linePosition[num] = pos
    lineMaxDist[num] = maxDist

def LineMatrixRowText(num):
    pos = linePosition[num]
    theta_deg = np.rad2deg(lineTheta[num])
    return (
        f"{num:2d}. {lineNames[num]:<12}:  "
        f"a={lineMatrix[num, 0]:9.5f}  b={lineMatrix[num, 1]:9.5f}  c={lineMatrix[num, 2]:10.5f}  "
        f"min={lineMin[num]:6.2f}  max={lineMax[num]:<6.2f}  "
        f"theta={theta_deg:4.0f}°  "
        f"{P_TEXT[pos]}")

def PrintLineMatrix():
    print("Line Matrix for Positioning")
    for num in range(LineMatrixRows):
        print(f"{LineMatrixRowText(num)}")

def X(meter):
    return meter*PixelsPerMeter + PixelOffsetX

def Y(meter):
    return PixelOffsetY - meter*PixelsPerMeter

def Xm(pixel):
    return (pixel - PixelOffsetX)*MetersPerPixel

def Ym(pixel):
    return (PixelOffsetY - pixel) * MetersPerPixel

def V(meterPerSec):
    return meterPerSec*WIN_W/WIN_W_METER

def NormalizeAngle(angle_rad):
    return (angle_rad + math.pi) % math.tau - np.pi

def CheckAngle(num, angle_rad):
    value_rad = lineTheta[num]
    MaxDiff = np.deg2rad(2.5)
    diff1 = NormalizeAngle(angle_rad - value_rad)
    diff2 = NormalizeAngle(angle_rad + math.pi - value_rad)
    return abs(diff1) < MaxDiff or abs(diff2) < MaxDiff 

def CheckAngleResult_deg(num, angle_rad):
    value_rad = lineTheta[num]
    MaxDiff = np.deg2rad(5.0)
    diff1 = NormalizeAngle(angle_rad - value_rad)
    diff2 = NormalizeAngle(angle_rad + math.pi - value_rad)
    return np.rad2deg(min(abs(diff1), abs(diff2)))

def CheckLength(num, vektorLen):
    if vektorLen < lineMin[num]: return False
    if vektorLen > lineMax[num]: return False
    return True

def IsHorizontal(num):
    return lineMatrix[num, 0] ==  0.0

def IsVertikal(num):
    return lineMatrix[num, 1] ==  0.0
    
def LineEquation(A, B):
    """ Berechne die Parameter m und t der Geraden y=m*x+t, die durch die Punkte A und B geht """
    m = (B[1] - A[1]) / (B[0] - A[0])
    t = A[1] - m*A[0]
    return m, t
    
def Distance(start, end):
    """ Berechne den Abstand in Y-Richtung von der Geraden, die durch die Punkte start und end geht """
    m, t = LineEquation(start, end)
    distX = -t/m
    distY = t
    return distX, distY

def Localization(theta, all_detected_walls, A, b, lineNumbers, ignore=lineIgnore, debug=True):
    """ Prüft alle Linien in den Lidardaten und berechnet aus den gültigen Linien die Roboter-Position """
    theta_deg = np.rad2deg(theta)
    c, s = np.cos(theta), np.sin(theta)
    # 3. Rotationsmatrix erstellen
    # Formel: [[cos, -sin], [sin, cos]]
    # Da unsere Punkte Zeilenvektoren sind (N,2), müssen wir 
    # die Matrix für die Multiplikation transponieren oder die Formel anpassen.
    rotation_matrix = np.array([
        [c, -s],
        [s,  c]
    ])
    isDetectedWallValid = []
    #if numTest>=0: print(f"{LineMatrixRowText(numTest)}")
    for start, end in np.array(all_detected_walls):  
        # Drehung durch Matrix-Multiplikation (Dot Product)
        # Wir transponieren die Matrix (.T), damit (N,2) * (2,2) funktioniert        
        start = np.dot(start, rotation_matrix.T)
        end = np.dot(end, rotation_matrix.T)
        vektor = end - start
        vektorLen = np.linalg.norm(vektor)
        angle_rad = np.arctan2(vektor[1], vektor[0])
        lineValid = False
        for num in range(LineMatrixRows):  # [0, 2, 5, 6]:
            #if num==1: print(f"{(start,end)} {CheckAngle(num, angle_rad)} {CheckLength(num, vektorLen)} {CheckPos(num, start, end)}")
            if CheckAngle(num, angle_rad) and CheckLength(num, vektorLen) and not ignore[num]:
                s = P_SIGN[linePosition[num]]
                if IsVertikal(num):     
                    bVal = (start[0] + end[0]) / 2
                    dist = s*bVal
                    xy = "x"
                elif IsHorizontal(num): 
                    bVal = (start[1] + end[1]) / 2
                    dist = s*bVal
                    xy = "y"
                else:                   
                    dx, dy = Distance(start, end)
                    distX = s*dx; distY = s*dy
                    if linePosition[num] == P_EAST or linePosition[num] == P_WEST:
                        xy = "x" 
                        dist = distX
                        bVal = dx
                    else:
                        xy = "y"
                        dist = distY
                        bVal = dy
                if dist>0 and dist<lineMaxDist[num]:
                    A.append([lineMatrix[num, 0], lineMatrix[num, 1]])
                    b.append(bVal - lineMatrix[num, 2])
                    lineNumbers.append(num)
                    if debug: print(
                        f"=== [Localization] {lineNames[num]} ({vektorLen:5.2f}m breit) erkannt {dist:5.2f}m {xy}-Abstand vom Robotor, Theta={theta_deg:.0f}°")
                    lineValid = True
        isDetectedWallValid.append(lineValid)
    return isDetectedWallValid
        
@dataclass
class Segment:
    """Liniensegment (Wand/Begrenzung) in Weltkoordinaten."""
    x1: float
    y1: float
    x2: float
    y2: float

    def draw(self, surf, color):
        pygame.draw.line(surf, color, (self.x1, self.y1), (self.x2, self.y2), 2)
    
    # Neu für NumPy: Segment als Vektor zurückgeben
    def to_numpy(self):
        return np.array([self.x1, self.y1]), np.array([self.x2, self.y2])


class World:
    """Enthält alle kollidierenden Liniensegmente und zeichnet die Szene."""
    ZaunN     = 0
    ZaunO     = 1
    ZaunS     = 2
    ZaunW     = 3
    SchuppenW = 4
    SchuppenS = 5
    SchuppenO = 6
    TerrasseW = 7
    TerrasseS = 8
    BassinO   = 9
    BassinN   = 10
    HausO     = 11

    def __init__(self):
        self.segments = []

        # Außenrahmen erzeugen
        x0, y0 = BORDER_MARGIN, BORDER_MARGIN
        x1, y1 = WIN_W - BORDER_MARGIN, WIN_H - BORDER_MARGIN
        x1_wall = WALL_X      
        
        self.CreateGarten()
        
        # Tor Segmente
        #self.segments.append(Segment(WALL_X, y0, WALL_X, GATE_Y1))
        #self.segments.append(Segment(WALL_X, GATE_Y2, WALL_X, y1))

    def draw(self, surf):
        for s in self.segments:
            s.draw(surf, WALL_COLOR)
        # Tor Visualisierung
        # pygame.draw.line(surf, GATE_COLOR, (WALL_X, GATE_Y1), (WALL_X, GATE_Y2), 3)

    def Line(self, A, B):
        a = np.array(A) - Center
        b = np.array(B) - Center
        self.segments.append(Segment(X(a[0]), Y(a[1]), X(b[0]), Y(b[1])))
        #print(f"Line created {a=}, {b=}")

    def Haus(self, name, eckPunkte):
        #print(f"{eckPunkte=}")
        Length(name+"South", eckPunkte[0], eckPunkte[1])
        Length(name+"East",  eckPunkte[1], eckPunkte[2])
        posX1, posY1 = eckPunkte[0]
        posX2, posY2 = eckPunkte[2]
        lenX = posX2 - posX1
        lenY = posY2 - posY1
        
        posX = posX1    #- CenterX
        posY = posY1    #- CenterY
        print(f"Creating {name:<12}: Unten links = ({posX:6.2f},{posY:6.2f}), Oben rechts = ({posX+lenX:6.2f},{posY+lenY:6.2f})")
        self.Line( (posX,       posY     ), (posX+lenX, posY     ) )
        self.Line( (posX+lenX,  posY     ), (posX+lenX, posY+lenY) )
        self.Line( (posX+lenX,  posY+lenY), (posX,      posY+lenY) )
        self.Line( (posX,       posY+lenY), (posX,      posY     ) )

    # -------------------------------------------------------
    # Garten
    # -------------------------------------------------------
    def CreateGarten(self):
        CenterX =  22
        CenterY = -12
            
        PA, PB, PC, PD = Gartenzaun()
        # Gartentor hinzufügen
        torAbstand = 8.00
        torBreite = 0.92
        PL = (PB[0]-torAbstand-torBreite, PB[1])    # linker Torpfosten
        PR = (PB[0]-torAbstand, PB[1])              # rechter Torpfosten
        self.Line(PA, PL)
        self.Line(PR, PB)
        self.Line(PB, PC)
        self.Line(PC, PD)
        self.Line(PD, PA)
        AddLineForPositioning(0, PA, PB, P_NORTH,  7.0)
        AddLineForPositioning(1, PB, PC, P_EAST,   4.0)
        AddLineForPositioning(2, PC, PD, P_SOUTH,  7.0)
        AddLineForPositioning(3, PD, PA, P_WEST, 7.0)

        schuppen = Schuppen()
        self.Haus("Schuppen", schuppen)
        AddLineForPositioning(4, schuppen[3], schuppen[0], P_EAST)
        AddLineForPositioning(5, schuppen[0], schuppen[1], P_NORTH, maxDist=7.0)
        AddLineForPositioning(6, schuppen[1], schuppen[2], P_WEST)
        
        terrasse = Terrasse()
        self.Haus("Terrasse", terrasse)
        AddLineForPositioning(7, terrasse[3], terrasse[0], P_EAST)
        AddLineForPositioning(8, terrasse[0], terrasse[1], P_NORTH)
        
        def Bassin(terrasseUntenRechts):
            lenX = 2.10 #2.40
            lenY = 2.50 #2.80
            (x, y) = terrasseUntenRechts
            x += 0.50
            y += 0.40   # 0.0
            return [(x, y), (x+lenX, y), (x+lenX, y+lenY), (x, y+lenY)]
        
        bassin = Bassin(terrasse[1])
        self.Haus("Bassin", bassin)
        AddLineForPositioning( 9, bassin[1], bassin[2], P_WEST, maxDist=8.0)
        AddLineForPositioning(10, bassin[2], bassin[3], P_SOUTH, maxDist=7.0)
        
        gartenhaus = Gartenhaus()
        self.Haus("Gartenhaus", gartenhaus)
        specialMinLength = 5.50     # wegen angrenzender Terrasse
        AddLineForPositioning(11, gartenhaus[1], gartenhaus[2], P_WEST, specialMinLength)
        
        def Strauch(name, mittelPunkt, dm):
            print(f"Creating {name}")
            mx, my = mittelPunkt
            r = dm/2
            N = 12
            dphi = 2*math.pi/N
            for i in range(N):
                phi0 = i*dphi
                phi1 = (i+1)*dphi
                x0 = mx + r*math.cos(phi0)
                y0 = my + r*math.sin(phi0)
                x1 = mx + r*math.cos(phi1)
                y1 = my + r*math.sin(phi1)
                self.Line((x0,y0), (x1,y1))       
        
        tx, ty = terrasse[0]
        grasMP = (tx, ty-8.0)
        grasDM = 1.50
        Strauch("Gras", grasMP, grasDM)
        
        strauchMP = (tx+13.00, ty-8.00)
        strauchDM = 2.00    
        Strauch("Strauch1", strauchMP, strauchDM)
        Strauch("Strauch2", (22-7.0, 8.0-12), 1.0)
        Strauch("Strauch3", (22+1.0, 8.0-12), 3.0)
        Strauch("Baum",     (22+3.0, 9.0-12), 3.0)
        Strauch("Lampe",    (22-3.0, 8.0-12), 0.05)
        


def Move(poly, mx, my):
    length = len(poly)
    for i in range(length):
        (x,y) = poly[i]
        poly[i] = (x+mx,y+my)
    return

def Angle(A, B):
    vektor = np.array(B) - np.array(A)
    angle_rad = np.arctan2(vektor[1], vektor[0])
    #winkel_deg = np.rad2deg(winkel_rad)
    #print(f"Theta{text}_rad = {winkel_rad:8.5f}    # {winkel_deg:6.1f}°")
    return angle_rad

def Length(text, A, B):
    vektor = np.array(B) - np.array(A)
    dist = np.linalg.norm(vektor)
    print(f"Len{text} = {dist:5.2f}")
    return dist

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
    schuppen = [(0.00,0.00),(4.00,0.00),(4.00,4.50),(0.00,4.50)]
    mx = 5.10 + 1.6 - 0.5
    my = -4.50-0.50 - 0.3
    Move(schuppen,mx,my)
    return schuppen

# if __name__ == '__main__':
world = World()
PrintLineMatrix()