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
    ignoreList = [ 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0]
    detectedWallsValid = Localization(node.theta, detectedWalls, A, b, wallNumbers, ignore=ignoreList, debug=False)
    PublishMarkers(node.marker_pub, detectedWalls, detectedWallsValid)
    A, b, wallNumbers = RemoveEquations(A, b, wallNumbers, debug=False)
    numEq = len(wallNumbers)
    if numEq > 1:
        x, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
        if not (residuals.size > 0 and residuals > 0.5) and rank == 2:
            return True, float(x[0]), float(x[1])
    return False, 0.0, 0.0

def DistAngleClosestWall(walls):
    """ Berechnet den Abstand zur nähesten Wand """
    minDist = np.inf
    minAngle = 0.0
    # A und B sind die Endpunkte der Wand
    for A, B in walls:
        # Richtungsvektor der Geraden (AB) und Vektor zum Punkt (AP)
        ab = B - A
        ap = -A
        abLen = np.linalg.norm(ab)
        
        # Berechnung des Abstands
        # Wir berechnen die Norm des Kreuzprodukts geteilt durch die Norm von AB
        # Für 2D simulieren wir das Kreuzprodukt durch eine Hilfsfunktion
        #assert len(A) == 2
        dist = np.abs(np.cross(ab, ap)) / abLen
        if dist < minDist and abLen > 3.0:
            angle = np.arctan2(ab[1], ab[0])
            if abs(angle) < np.pi*0.4:   #/4:
                minDist = dist
                minAngle = angle
    return float(minDist), float(minAngle)


class MowingTask:
    def Init(self, node, params, retvals=None):
        self.StateAlignTheta = 0
        self.StateMow = 1
        self.StateIdle = 2
        self.followRight = True
        self.K_lat = 0.5          # 0.3       # lateral error (Abstand)
        self.K_head = 0.7          # 0.5      # heading error (Winkel)
        self.vLinear = 0.5
        self.node = node
        self.laneDist = 0.2
        self.startDist = 0.6
        self.endDist = 2.5
        self.wantedDist = self.startDist
        self.wantedTheta = np.deg2rad(85)
        self.node.SetWantedTheta(self.wantedTheta)
        self.state = self.StateAlignTheta

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
            dist, angle = DistAngleClosestWall(matchingWalls)
            
            if 0.0 < dist < np.inf:
                lateralError = dist - self.wantedDist
                angle = NormalizeAngle(angle)
                headingError = 0.0 - angle
                omega = s*self.K_lat*lateralError - self.K_head*headingError
                print(f"{dist=:.2f}  angle={G(angle):.0f}°  {lateralError=:.2f}  headingError={G(headingError):.0f}°")
                self.node.SetVelocities(omega, -s*self.vLinear)                
            else:
                print(f"{dist=:.2f}")
                self.node.SetVelocities(0.0, 0.0)                

            ok, x, y = ComputePosition(self.node, walls)
            if ok:
                if x < 16.0:
                    self.state = self.StateIdle
                    self.node.ResetDirection()
                    self.node.RvizPrint("Mowing completed")
                elif self.followRight:
                    if y > 9.0:
                        self.node.RvizPrint("Mowing Down")
                        self.followRight = False
                        self.wantedDist += self.laneDist
                elif y < -2.0:
                        self.node.RvizPrint("Mowing Up")
                        self.followRight = True
                        self.wantedDist += self.laneDist                    

        elif self.state == self.StateIdle:
            self.node.ResetDirection()
            return 0

        return None

