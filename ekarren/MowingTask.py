import numpy as np
import math
from GartenWorld import Localization, lineNames        #, lineNames, World, GetWallPosX, GetWallPosY
from params import TaskState

def G(x):
    return np.rad2deg(x)

def NormalizeAngle(angle_rad):
    return (angle_rad + math.pi) % math.tau - np.pi


def RemoveEquations(A, b, lineNumbers, debug=True):
    """ Verwende nur Zäune zur Lokalisierung, wenn mindesten zwei Zäune erkannt wurden """
    """ (dient zur Reduzierung von Fehlerkennungen) """
    numEq = len(b)
    zaun = [0, 0, 0]
    for i in range(numEq):
        if 0 <= lineNumbers[i] <= 2: zaun[lineNumbers[i]] = 1
    if debug: print(f"{zaun=}")
    if zaun[0] + zaun[1] + zaun[2] < 2:
        return np.array(A), np.array(b), lineNumbers
    Anew = []
    bnew = []
    infoNew = []
    n = 0
    for i in range(numEq):
        if 0 <= lineNumbers[i] <= 2:
            Anew.append(A[i])
            bnew.append(b[i])
            infoNew.append(lineNumbers[i])
        else: n += 1
    if debug: print(f"{n} equations removed")
    return np.array(Anew), np.array(bnew), infoNew


def FindWalls(node, detectedWalls, wallNum):
    A = []
    b = []
    wallNumbers = []
    #            [ZN,ZO,ZS,ZW,SW,SS,SO,TW,TS,BO,BN,HO]
    ignoreList = [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    assert 0 <= wallNum < 12
    ignoreList[wallNum] = 0
    detectedWallsValid = Localization(node.theta, detectedWalls, A, b, wallNumbers, ignore=ignoreList, debug=False)
    return detectedWallsValid

 
def CheckAngle(value_rad, angle_rad):
    MaxDiff = np.deg2rad(2.5)
    diff1 = NormalizeAngle(angle_rad - value_rad)
    diff2 = NormalizeAngle(angle_rad + math.pi - value_rad)
    return abs(diff1) < MaxDiff or abs(diff2) < MaxDiff 


class MowingTask:
    def Init(self, node, params, retvals=None):
        self.StateAlignTheta = 0
        self.StateMow = 1
        self.StateIdle = 2
        self.followRight = True
        self.vLinear = 0.5  #0.5
        self.K_lat = 0.5 # 0.5          # 0.3       # lateral error (Abstand)
        self.K_head = 0.8 # 0.7          # 0.5      # heading error (Winkel)
        self.node = node
        self.laneDist = 0.2
        self.startDist = 2.6*0+0.6*1
        self.wantedDist = self.startDist
        self.wallAngle = np.deg2rad(85)
        self.wantedTheta = self.wallAngle
        self.node.SetWantedTheta(self.wantedTheta)
        self.state = self.StateAlignTheta
        self.subState = 0
        self.errorCounter = 0
        self.debug = False
        self.moterTimeOutValue = 10
        self.moterTimeOutCounter = self.moterTimeOutValue

    def DistAngleWallToFollow(self, walls, wallToFollow):
        """ Berechnet den Abstand zur Wand, der gefolgt werden soll """
        minDist = np.inf
        minAngle = 0.0
        minWorldAngle = 0.0
        
        wallFound = FindWalls(self.node, walls, wallToFollow)
        # A und B sind die Endpunkte der Wand
        for i, (A, B) in enumerate(walls):
            if wallFound[i]:
                # Richtungsvektor der Geraden (AB) und Vektor zum Nullpunkt (AP)
                ab = B - A
                ap = -A
                abLen = np.linalg.norm(ab)
                
                # Berechnung des Abstands
                # Wir berechnen die Norm des Kreuzprodukts geteilt durch die Norm von AB
                # Für 2D simulieren wir das Kreuzprodukt durch eine Hilfsfunktion
                dist = np.abs(np.cross(ab, ap)) / abLen
                if dist < minDist and abLen > 3.0:    # 3.0
                    angle = np.arctan2(ab[1], ab[0])
                    worldAngle = angle + self.node.theta
                    #if CheckAngle(self.wallAngle, worldAngle):       
                    minDist = dist
                    minAngle = angle
                    minWorldAngle = worldAngle
        return float(minDist), float(minAngle), float(minWorldAngle)

    def ComputePosition(self, detectedWalls):
        A = []
        b = []
        wallNumbers = []
        #            [ZN,ZO,ZS,ZW,SW,SS,SO,TW,TS,BO,BN,HO]
        ignoreList = [ 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        #ignoreList = [ 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0]
        detectedWallsValid = Localization(self.node.theta, detectedWalls, A, b, wallNumbers, ignore=ignoreList, debug=False)
        if len(detectedWalls) < 3: 
            self.errorCounter += 1
            print(f"ERROR LocalizationTask failed {len(detectedWalls)=}")
            return False, 0.0, 0.0, 0.0
        self.node.PublishMarkers(detectedWalls, detectedWallsValid)
        A, b, wallNumbers = RemoveEquations(A, b, wallNumbers, debug=False)
        numEq = len(wallNumbers)
        if numEq < 2:
            self.errorCounter += 1
            print(f"ERROR LocalizationTask failed {numEq=}")
            return False, 0.0, 0.0, 0.0
        if A.ndim <= 1:
            self.errorCounter += 1
            print(f"ERROR LocalizationTask failed {A.ndim=}")
            return False, 0.0, 0.0, 0.0
        x, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
        # Falls residuals nicht leer ist, nimm das erste Element, sonst berechne es manuell
        res = float(residuals[0]) if residuals.size > 0 else 0.0
        if rank < 2:
            self.errorCounter += 1
            print(f"ERROR LocalizationTask failed {rank=}")
            return False, 0.0, 0.0, 0.0
        if res > 0.5:
            self.errorCounter += 1
            print(f"ERROR LocalizationTask failed {res=}")
            return False, 0.0, 0.0, 0.0
        if self.debug:
            print(f"Lösung: ({x[0]:.2f}, {x[1]:.2f})    Rang:{rank}   Fehlerquadratsumme: {residuals}")                
            for i in range(numEq):
                err = x[0]*A[i,0] + x[1]*A[i,1] - b[i]
                print(f"{b[i]:6.2f} = {A[i,0]:6.2f}*x + {A[i,1]:6.2f}*y   {err=:6.2f}  # {lineNames[wallNumbers[i]]}")

        return True, float(x[0]), float(x[1]), res

    def Step(self, ranges):
        if self.state == self.StateAlignTheta:
            if self.node.wantedThetaReached:
                self.state = self.StateMow
                self.node.ResetDirection()
                self.node.RvizPrint("Mowing up")
                
        elif self.state == self.StateMow:
            s = -1.0 if self.followRight else 1.0
            all_detected_walls = self.node.Walldetector(ranges) 
            walls = np.array(all_detected_walls)
            #print(f"{walls.shape}")

            # Differenz-Vektoren berechnen (Endpunkt - Startpunkt)
            # walls hat die Form (N, 2, 2)
            diff = walls[:, 1, :] - walls[:, 0, :]

            # Quadrierte Längen berechnen (x^2 + y^2)
            # np.sum(..., axis=1) summiert x- und y-Quadrate für jede Wand einzeln
            minLen = 2.0
            minLenSquare = minLen*minLen
            squaredLen = np.sum(diff**2, axis=1)
            mask = squaredLen > minLenSquare

            # Filtern der Wände, die die Bedingung erfüllen
            matchingWalls = walls[mask]        
            
            # Berechne Abstand und Winkel zur Wand, der gefolgt werden soll
            ZAUN_OST = 1
            dist, angle, worldAngle = self.DistAngleWallToFollow(matchingWalls, ZAUN_OST)
            
            # Berechne Position
            ok, x, y, res = self.ComputePosition(walls)
            
            if (0.0 < dist < np.inf) and ok:
                lateralError = dist - self.wantedDist
                angle = NormalizeAngle(angle)
                headingError = 0.0 - angle
                omega = s*self.K_lat*lateralError - self.K_head*headingError
                print(f"{dist=:.2f}  angle={G(angle):.0f}/{G(worldAngle):.0f}°  LatErr={lateralError:.2f}"
                      f"  HeadErr={G(headingError):.0f}°  {omega=:.3f}  {x=:.2f} {y=:.2f}   {res=:.3f}")
                self.node.SetVelocities(omega, -s*self.vLinear)           
                self.moterTimeOutCounter = self.moterTimeOutValue
            else:
                print(f"{dist=:.2f}")
                self.moterTimeOutCounter -= 1
                if self.moterTimeOutCounter <= 0:
                    self.node.SetVelocities(0.0, 0.0)                

            if ok:
                pos = np.array([x, y])   
                self.node.odom.SetPos(pos, self.node.theta)
                yMin =  2.0   #-2.0
                yMax =  10.5
                RepeatDist = 2.0
                repeatMode = True
                if x < 16.0:
                    self.state = self.StateIdle
                    self.node.ResetDirection()
                    self.node.RvizPrint("Mowing completed")
                elif repeatMode:
                    if self.subState==0 and y>yMax:
                            self.followRight = False
                            self.wantedDist += self.laneDist
                            self.subState = 1
                            self.node.RvizPrint(f"Mowing Down SubState={self.subState}")
                    elif self.subState==1 and y<yMax-RepeatDist:
                            self.followRight = True
                            self.subState = 2
                            self.node.RvizPrint(f"Mowing Down/Repeat SubState={self.subState}")
                    elif self.subState==2 and y>yMax:
                            self.followRight = False
                            self.subState = 3
                            self.node.RvizPrint(f"Mowing Down SubState={self.subState}")
                    elif self.subState==3 and y<yMin:
                            self.wantedDist += self.laneDist  
                            self.followRight = True
                            self.subState = 4
                            self.node.RvizPrint(f"Mowing Up SubState={self.subState}")
                    elif self.subState==4 and y>yMin+RepeatDist:
                            self.followRight = False
                            self.subState = 5
                            self.node.RvizPrint(f"Mowing Up/Repeat SubState={self.subState}")
                    elif self.subState==5 and y<yMin:
                            self.followRight = True
                            self.subState = 0
                            self.node.RvizPrint(f"Mowing Up SubState={self.subState}")
                else:
                    if self.followRight:
                        if y > yMax:
                            self.node.RvizPrint("Mowing Down")
                            self.followRight = False
                            self.wantedDist += self.laneDist
                    elif y < yMin:
                            self.node.RvizPrint("Mowing Up")
                            self.followRight = True
                            self.wantedDist += self.laneDist  
            else:
                print(f"No position {res=}")
        elif self.state == self.StateIdle:
            self.node.ResetDirection()
            return TaskState.Ready, None

        return TaskState.Running, None
