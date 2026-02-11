import numpy as np
import math
from Ransac import PublishMarkers
from GartenWorld import Localization        #, lineNames, World, GetWallPosX, GetWallPosY

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

 
def ComputePosition(node, detectedWalls):
    A = []
    b = []
    wallNumbers = []
    #            [ZN,ZO,ZS,ZW,SW,SS,SO,TW,TS,BO,BN,HO]
    ignoreList = [ 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    #ignoreList = [ 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0]
    detectedWallsValid = Localization(node.theta, detectedWalls, A, b, wallNumbers, ignore=ignoreList, debug=False)
    PublishMarkers(node.marker_pub, detectedWalls, detectedWallsValid)
    A, b, wallNumbers = RemoveEquations(A, b, wallNumbers, debug=False)
    numEq = len(wallNumbers)
    if numEq > 1:
        x, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
        # Falls residuals nicht leer ist, nimm das erste Element, sonst berechne es manuell
        res = float(residuals[0]) if residuals.size > 0 else 0.0
        if res < 0.5 and rank == 2:
            return True, float(x[0]), float(x[1]), res
    return False, 0.0, 0.0, 0.0

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
        self.startDist = 2.6+0.6*0
        self.wantedDist = self.startDist
        self.wallAngle = np.deg2rad(85)
        self.wantedTheta = self.wallAngle
        self.node.SetWantedTheta(self.wantedTheta)
        self.state = self.StateAlignTheta
        self.subState = 0

    def DistAngleClosestWall(self, walls):
        """ Berechnet den Abstand zur nähesten Wand """
        minDist = np.inf
        minAngle = 0.0
        minWorldAngle = 0.0
        # A und B sind die Endpunkte der Wand
        for A, B in walls:
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
                if CheckAngle(self.wallAngle, worldAngle):       
                    minDist = dist
                    minAngle = angle
                    minWorldAngle = worldAngle
        return float(minDist), float(minAngle), float(minWorldAngle)

    def Step(self, scan_msg):
        if self.state == self.StateAlignTheta:
            if self.node.wantedThetaReached:
                self.state = self.StateMow
                self.node.ResetDirection()
                self.node.RvizPrint("Mowing up")
                
        elif self.state == self.StateMow:
            s = -1.0 if self.followRight else 1.0
            all_detected_walls = self.node.Walldetector(scan_msg) 
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
            dist, angle, worldAngle = self.DistAngleClosestWall(matchingWalls)
            ok, x, y, res = ComputePosition(self.node, walls)
            
            if 0.0 < dist < np.inf:
                lateralError = dist - self.wantedDist
                angle = NormalizeAngle(angle)
                headingError = 0.0 - angle
                omega = s*self.K_lat*lateralError - self.K_head*headingError
                print(f"{dist=:.2f}  angle={G(angle):.0f}/{G(worldAngle):.0f}°  LatErr={lateralError:.2f}"
                      f"  HeadErr={G(headingError):.0f}°  {omega=:.3f}  {x=:.2f} {y=:.2f}   {res=:.3f}")
                self.node.SetVelocities(omega, -s*self.vLinear)                
            else:
                print(f"{dist=:.2f}")
                self.node.SetVelocities(0.0, 0.0)                

            if ok:
                yMin = -2.0
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
                print("No position")
        elif self.state == self.StateIdle:
            self.node.ResetDirection()
            return 0

        return None

