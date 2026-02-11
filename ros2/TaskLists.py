import sys
import numpy as np
from Ransac import PublishMarkers
from GartenWorld import Localization, lineNames, World, GetWallPosX, GetWallPosY
from MowingTask import MowingTask
import params

# Lidar sector width (in deg) for distance check
# nur der Bereich von -LS_xxx° bis +LS_xxx° wird verwendet
LsAngle = 5
LsFront = 0
LsFrontL = LsAngle
LsFrontR = -LsAngle
LocXLT = 0
LocXGE = 1
LocYLT = 2
LocYGE = 3
LocMask = 3
LocRevDrv = 4

# Koordinaten zur Zonenbestimmung
yZaunN = GetWallPosY(World.ZaunN)
xTerrasseW = GetWallPosX(World.TerrasseW)
yTerrasseS = GetWallPosY(World.TerrasseS)
xSchuppenO = GetWallPosX(World.SchuppenO)
xSchuppenW = GetWallPosX(World.SchuppenW)
ySchuppenS = GetWallPosY(World.SchuppenS)
xBassinO = GetWallPosX(World.BassinO)
yBassinN = GetWallPosY(World.BassinN)
xHausO = GetWallPosX(World.HausO)
yHausS = 7.0
dxSchuppenO = 2.0
xZ23 = xSchuppenO + dxSchuppenO
dyZ23 = 4.0
xZ45 = xBassinO + 1.5
dxHausO = 3.0
dyZaunN1 = 2.5
dyZaunN7 = 3.0
xWald = xHausO + dxHausO
yWald = yZaunN - dyZaunN7

def d2r(angle_deg):
    return np.deg2rad(angle_deg)

def Zone(x, y):
    """ Bestimmt in welcher Zone sich der Roboter beim Start befindet """
    if x >= xSchuppenO and y >= ySchuppenS and x < xHausO:
        return 1
    elif x <= xZ23 and y <= ySchuppenS:
        return 2
    elif y <= yHausS and x <= xTerrasseW and x >= xZ23:    
        return 3
    elif y <= yTerrasseS and x >= xTerrasseW and x <= xZ45: 
        return 4
    elif y <= yTerrasseS and x >= xZ45: 
        return 5
    elif y >= yTerrasseS and y <= yBassinN and x >= xBassinO: 
        return 6
    elif y >= yBassinN and x >= xHausO: 
        return 7
    return 0


def sign(x):
    return 1.0 if x >= 0 else -1.0

def PathFinder(x, y, target):
    zone = Zone(x, y)
    print(f"Zone {zone} detected")
    path = []
    if zone == 0: 
        return path
    if target=="Wald":
        #   ignoreList: [ZN,ZO,ZS,ZW,SW,SS,SO,TW,TS,BO,BN,HO]
        loc = (LocXGE, [ 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1])
        if zone == 1:
            dy1 = dyZaunN1*sign(yZaunN - y - dyZaunN1)
            dx1 = dxSchuppenO * sign(x - xZ23)
            path = [
                ( np.pi/2,         dy1,  LsFront),     # nach Norden
                (  -np.pi,         dx1, LsFrontR),     # nach Westen/Osten bis vor dem Schuppen
                (-np.pi/2,       dyZ23,  LsFront),     # nach Süden
                ( -d2r(21),        xZ45,     loc),     # nach Osten
                ( np.pi/2,    dyZaunN7,  LsFront),     # nach Norden
                (  -np.pi,     dxHausO,  LsFront)      # nach Westen
            ]
        elif zone == 2:
            #   ignoreList:  [ZN,ZO,ZS,ZW,SW,SS,SO,TW,TS,BO,BN,HO]
            loc1 = (LocXGE, [ 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1])
            dy1 = y - GetWallPosY(World.ZaunS, xZ23)
            dy2 = sign(dy1-dyZ23)*dyZ23
            path = [
                (       0,     xZ23,    loc1),     # nach Osten bis östlich vom Schuppen
                (-np.pi/2,      dy2, LsFront),     # nach Süden/Norden
                (-d2r(21),     xZ45,     loc),     # nach Osten
                ( np.pi/2, dyZaunN7, LsFront),     # nach Norden
                (  -np.pi,  dxHausO, LsFront)      # nach Westen
            ]
        elif zone in [3, 4]:
            dy1 = y - GetWallPosY(World.ZaunS, x)
            dy2 = sign(dy1-dyZ23)*dyZ23
            path = [
                (-np.pi/2,      dy2, LsFrontL),    # nach Süden/Norden
                (-d2r(21),     xZ45,      loc),    # nach Osten
                ( np.pi/2, dyZaunN7,        0),    # nach Norden
                (  -np.pi,  dxHausO,        0)     # nach Westen
            ]
        elif zone in [5, 6, 7]:
            dx1 = (GetWallPosX(World.ZaunO, y) - xZ45) * sign(xZ45 - x)
            dy1 = dyZaunN7 * sign(yWald - y)
            path = [
                (     0.0,      dx1, LsFront),     # nach Westen/Osten
                ( np.pi/2,      dy1, LsFront),     # nach Norden/Süden
                (  -np.pi,  dxHausO, LsFront)      # nach Westen
            ]
    elif target=="Schuppen":
        if zone == 1:
            dx1 = dxSchuppenO * sign(x - xZ23)
            y1 =  yZaunN - dyZaunN1
            dy1 = dyZaunN1 * sign(y1 - y)
            path = [
                ( np.pi/2, dy1, LsFront),     # nach Norden/Süden
                (  -np.pi, dx1, LsFront)      # nach Westen/Osten bis vor dem Schuppen
            ]
        elif zone == 2:
            #   ignoreList:  [ZN,ZO,ZS,ZW,SW,SS,SO,TW,TS,BO,BN,HO]
            loc1 = (LocXGE, [ 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1])
            path = [
                (       0,     xZ23,     loc1),     # nach Osten bis östlich vom Schuppen
                ( np.pi/2, dyZaunN1,  LsFront),     # nach Norden
            ]
        elif zone in [3, 4]:
            dy1 = (y - GetWallPosY(World.ZaunS, x))
            dy2 = sign(dy1 - dyZ23) * dyZ23
            #   ignoreList: [ZN,ZO,ZS,ZW,SW,SS,SO,TW,TS,BO,BN,HO]
            loc = (LocXLT, [ 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1])
            path = [
                (-np.pi/2,      dy2,  LsFrontR),  # nach Süden/Norden
                (d2r(159),     xZ23,       loc),  # nach Westen
                ( np.pi/2, dyZaunN1,  LsFrontR),  # nach Norden
            ]
        elif zone in [5, 6, 7]:
            dx1 = (GetWallPosX(World.ZaunO, y) - xZ45) * sign(xZ45 - x)
            y1 = GetWallPosY(World.ZaunS, xZ45) + dyZ23
            locMode = LocYLT if y > y1 else LocYGE | LocRevDrv
            #    ignoreList: [ZN,ZO,ZS,ZW,SW,SS,SO,TW,TS,BO,BN,HO]
            loc1 = (locMode, [ 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1])
            loc2 = (LocXLT,  [ 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1])
            path = [
                (     0.0,     dx1,         0),  # nach Westen/Osten
                (-np.pi/2,      y1,      loc1),  # nach Norden/Süden
                (d2r(159),    xZ23,      loc2),  # nach Westen
                ( np.pi/2, dyZaunN1, LsFrontR),  # nach Norden
            ]
    return path

class FollowPathTask:
    def __init__(self):
        self.StateAlignTheta = 0
        self.StateGotoWall = 1
        self.StateWallReached = 2
        self.StateFollowWall = 3
        self.StateIdle = 4
        
    def Init(self, node, target, retvals):
        self.node = node
        self.pathIndex = 0
        self.target = target
        xv, A, b, wallNumbers = retvals
        x = xv[0]
        y = xv[1]
        self.path = PathFinder(x, y, target)
        if len(self.path) > 0:
            self.theta, self.dist, self.lidarSector = self.path[self.pathIndex]
            self.pathIndex += 1
            self.node.get_logger().info(f"Starting FollowPathTask for {target}. First node: Theta={self.theta:.3f}  Dist={self.dist}m")
            self.node.SetWantedTheta(self.theta)
            self.State = self.StateAlignTheta
        else:
            self.node.get_logger().error(f"No path found for target {target}")
            self.State = self.StateIdle
        
    def Step(self, scan_msg):
        if self.State == self.StateAlignTheta:
            if self.node.wantedThetaReached:
                self.State = self.StateGotoWall
                if type(self.lidarSector) == int:
                    vLinear = params.LinearVelocity*sign(self.dist)
                else:
                    mode, _ = self.lidarSector
                    if mode & LocRevDrv:
                        vLinear = -params.LinearVelocity
                    else:
                        vLinear = params.LinearVelocity
                self.node.SetDirection(self.theta, vLinear)
        elif self.State == self.StateGotoWall:
            ranges = np.array(scan_msg.ranges)
            if type(self.lidarSector) == int:
                start_deg = params.LidarMaxAngle + self.lidarSector - LsAngle
                end_deg   = params.LidarMaxAngle + self.lidarSector + LsAngle
                dist = np.min(ranges[start_deg:end_deg]) 
                #print(f"{dist=}")
                targetReached = sign(self.dist)*dist < self.dist
            else:
                A = []
                b = []
                wallNumbers = []
                mode, ignoreList = self.lidarSector
                locMode = mode & LocMask
                revDrive = mode & LocRevDrv
                detectedWalls = self.node.Walldetector(scan_msg)        
                detectedWallsValid = Localization(self.node.theta, detectedWalls, A, b, wallNumbers, ignore=ignoreList, debug=False)
                PublishMarkers(self.node.marker_pub, detectedWalls, detectedWallsValid)
                A, b, wallNumbers = RemoveEquations(A, b, wallNumbers, debug=False)
                targetReached = False
                numEq = len(wallNumbers)
                if numEq > 1:
                    x, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
                    #print(f"Lösung: ({x[0]:.2f}, {x[1]:.2f})    Rang:{rank}   Fehlerquadratsumme: {residuals}")                
                    #for i in range(numEq):
                    #    err = x[0]*A[i,0] + x[1]*A[i,1] - b[i]
                    #    print(f"{b[i]:6.2f} = {A[i,0]:6.2f}*x + {A[i,1]:6.2f}*y   {err=:6.2f}  # {lineNames[wallNumbers[i]]}")
                    #print(f"{rank=}   {x=}   {residuals=}")
                    if not (residuals.size > 0 and residuals > 0.5) and rank == 2:
                        err = 0.0
                        if residuals.size > 0: 
                            err = float(residuals)
                        print(f"x={x[0]:.2f}  y={x[1]:.2f}  {err=:.3f}")
                        if locMode == LocXLT:
                            targetReached = x[0] < self.dist
                        if locMode == LocXGE:
                            targetReached = x[0] >= self.dist
                        if locMode == LocYLT:
                            targetReached = x[1] < self.dist
                        if locMode == LocYGE:
                            targetReached = x[1] >= self.dist
            if targetReached: 
                self.node.ResetDirection()
                if self.pathIndex >= len(self.path):
                    self.State = self.StateIdle
                    self.node.RvizPrint(f"Target {self.target} reached")
                    return 0
                self.theta, self.dist, self.lidarSector = self.path[self.pathIndex]
                text = f"Waypoint {self.pathIndex} reached. Next waypoint: Theta={np.rad2deg(self.theta):.0f} Grad  Dist={self.dist:.2f}m"
                #self.node.get_logger().info(text)
                self.node.RvizPrint(text)
                self.node.SetWantedTheta(self.theta)
                self.State = self.StateAlignTheta
                self.pathIndex += 1
        elif self.State == self.StateIdle:
            return 0
            
        return None

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
    

class LocalizationTask:
    def Init(self, node, params, retvals=None):
        self.node = node
        self.A = []
        self.b = []
        self.wallNumbers = []
        self.stateCounter = 0
        self.thetaStep = np.pi/2
        self.wantedTheta = int(self.node.theta/self.thetaStep + 1)*self.thetaStep
        self.node.SetWantedTheta(self.wantedTheta)

    def Step(self, scan_msg):
        #self.simTimeSec = self.node.get_clock().now().nanoseconds / 1e9
        detectedWalls = self.node.Walldetector(scan_msg)        
        if self.node.wantedThetaReached:
            detectedWallsValid = Localization(self.node.theta, detectedWalls, self.A, self.b, self.wallNumbers)
            PublishMarkers(self.node.marker_pub, detectedWalls, detectedWallsValid)
            self.stateCounter += 1          
            self.node.RvizPrint(f"Localization Task Step {self.stateCounter}")
            if self.stateCounter >= 4:
                A, b, wallNumbers = RemoveEquations(self.A, self.b, self.wallNumbers)
                numEq = len(wallNumbers)
                x, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
                print(f"Lösung: ({x[0]:.2f}, {x[1]:.2f})    Rang:{rank}   Fehlerquadratsumme: {residuals}")                
                for i in range(numEq):
                    err = x[0]*A[i,0] + x[1]*A[i,1] - b[i]
                    print(f"{b[i]:6.2f} = {A[i,0]:6.2f}*x + {A[i,1]:6.2f}*y   {err=:6.2f}  # {lineNames[wallNumbers[i]]}")
                if (residuals.size > 0 and residuals > 0.5) or rank < 2:
                    self.node.get_logger().error("LocalizationTask failed")
                return x, A, b, wallNumbers

            # Nächste Richtung ansteuern
            self.wantedTheta += self.thetaStep
            self.node.SetWantedTheta(self.wantedTheta)
        return None


class GotoTask:
    def Init(self, node, taskIndex, retvals=None):
        node.GotoTask(taskIndex)

LocalizationTaskList = [
    (LocalizationTask(),     None),
    (GotoTask(), 0)
]

FahreInDenWaldTaskList = [ (MowingTask(), None) ]


XXXFahreInDenWaldTaskList = [
    (LocalizationTask(),     None),
    (FollowPathTask(),       "Schuppen"),
    #(LocalizationTask(),     None),
    #(FollowPathTask(),       "Wald"),
    #(GotoTask(), 0)
]

