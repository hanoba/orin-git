import sys
import numpy as np
from Ransac import PublishMarkers
from GartenWorld import Localization, lineNames, World, GetWallPosX, GetWallPosY

sys.path.append('../HostSim')
from params import LidarMaxAngle, LinearVelocity
from WallFollower import WallFollower

# Lidar sector width (in deg) for distance check
# nur der Bereich von -LS_xxx° bis +LS_xxx° wird verwendet
LS_FRONT =   0
LS_FRONT2 = 10
LS_RIGHT = -80
LS_LEFT  =  80

# Koordinaten zur Zonenbestimmung
xTerrasseW = GetWallPosX(World.TerrasseW)
yTerrasseS = GetWallPosY(World.TerrasseS)
xSchuppenO = GetWallPosX(World.SchuppenO)
xSchuppenW = GetWallPosX(World.SchuppenW)
ySchuppenS = GetWallPosY(World.SchuppenS)
xBassinO = GetWallPosX(World.BassinO)
yBassinN = GetWallPosY(World.BassinN)
xHausO = GetWallPosX(World.HausO)
yHausS = 7.0
xZ23 = xSchuppenO + 2.5
xZ45 = xBassinO + 1.5


def Zone(x, y):
    if x >= xSchuppenO and y >= ySchuppenS:
        return 1
    elif x >= xZ23 and y <=ySchuppenS:
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
    path = []
    if target=="Wald":
        if (y < 7.0 and x < 6.0) or x < -7.0:
            y1 = 1.0 - GetWallPosY(World.ZaunS, x)
            path = [
                (-np.pi/2,   y1, LS_FRONT2),    # nach Süden
                (     0.0,  3.5, LS_FRONT),     # nach Osten
                ( np.pi/2,  3.0, LS_FRONT),     # nach Norden
                (  -np.pi,  3.0, LS_FRONT)      # nach Westen
            ]
        elif (y > 8.0 and x < 6.0):
            x1 = GetWallPosX(World.SchuppenO, y) + 2.0
            y1 = 1.0 - GetWallPosY(World.ZaunS, x1)
            #print(f"{x1=}  {y1=}")
            path = [
                (  -np.pi, 2.0, LS_FRONT),     # nach Westen bis 2m vor dem Schuppen
                (-np.pi/2,  y1, LS_FRONT),     # nach Süden
                (     0.0, 3.5, LS_FRONT),     # nach Osten
                ( np.pi/2, 3.0, LS_FRONT),     # nach Norden
                (  -np.pi, 3.0, LS_FRONT)      # nach Westen
            ]
    elif target=="Schuppen":
        yBassinN = GetWallPosY(World.BassinN, x)
        xHausO = GetWallPosX(World.HausO, y)
        theta1 = 0.0
        if x > xHausO and y > yBassinN:
            x1 = GetWallPosX(World.BassinO, y) + 1.0
            dx1 = GetWallPosX(World.ZaunO, y) - x1
            if dx1 < 0:
                dx1 = x1 - GetWallPosX(World.HausO, y)
                theta1 = -np.pi
            y1 = GetWallPosY(World.TerrasseS, 0) - 2.0
            #dy1 = y1 - GetWallPosY(World.ZaunS, x1)
            dy1 = GetWallPosY(World.ZaunN, x1) - y1
            x2 = GetWallPosX(World.SchuppenO, 0) + 2.0
            dy2 = y1 - GetWallPosY(World.ZaunS, x2)
            path = [
                (  theta1,   dx1, LS_FRONT),     # nach Osten oder Westen
                ( np.pi/2,  -dy1, LS_FRONT),     # nach Süden
                (  -np.pi,   dy2, LS_LEFT),      # nach Westen
                ( np.pi/2,   3.0, LS_FRONT)      # nach Norden
            ]
            print(dx1,dy1,dy2)
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
                vLinear = LinearVelocity*sign(self.dist)
                self.node.SetDirection(self.theta, vLinear)
        elif self.State == self.StateGotoWall:
            ranges = np.array(scan_msg.ranges)
            start_deg = LidarMaxAngle + self.lidarSector - 10
            end_deg   = LidarMaxAngle + self.lidarSector + 10
            dist = np.min(ranges[start_deg:end_deg]) 
            #print(f"{dist=}")
            if sign(self.dist)*dist < self.dist: 
                self.node.ResetDirection()
                if self.pathIndex >= len(self.path):
                    self.State = self.StateIdle
                    self.node.get_logger().info(f"Target {self.target} reached")
                    return 0
                self.theta, self.dist, self.lidarSector = self.path[self.pathIndex]
                self.node.SetWantedTheta(self.theta)
                self.State = self.StateAlignTheta
                self.node.get_logger().info(f"Node {self.pathIndex} reached. Next node: Theta={self.theta:.3f}  Dist={self.dist}m")
                self.pathIndex += 1
        elif self.State == self.StateIdle:
            return 0
            
        return None


class FollowWallTask:
    def __init__(self):
        self.StateAlignTheta = 0
        self.StateGotoWall = 1
        self.StateWallReached = 2
        self.StateFollowWall = 3
        self.StateIdle = 4
        
    def Init(self, node, params, retvals):
        self.targetDist = 2.0
        self.wallFollower = WallFollower(target_dist=self.targetDist, base_speed=0.5, debug=True)
        self.node = node
        xv, A, b, wallNumbers = retvals
        x = xv[0]
        y = xv[1]
        if y < 7.0 and x < 6.0:
            self.node.SetWantedTheta(-np.pi/2)
            self.State = self.StateAlignTheta
        else:
            self.State = self.StateIdle
        
    def Step(self, scan_msg):
        if self.State == self.StateAlignTheta:
            if self.node.wantedThetaReached:
                self.State = self.StateGotoWall
                self.node.SetDirection(-np.pi/2, 0.5)
        elif self.State == self.StateGotoWall:
            ranges = np.array(scan_msg.ranges)
            start_deg = LidarMaxAngle - 10
            end_deg = LidarMaxAngle + 10
            dist = np.min(ranges[start_deg:end_deg]) 
            #print(f"{dist=}")
            if dist < 2.0: 
                from GartenWorld import lineTheta
                wallSouthTheta = lineTheta[2] - np.pi
                self.node.ResetDirection()
                self.node.SetWantedTheta(wallSouthTheta)
                self.State = self.StateWallReached
        elif self.State == self.StateWallReached:
            if self.node.wantedThetaReached:
                self.State = self.StateFollowWall
                return 0
        elif self.State == self.StateFollowWall:
            ranges_np = np.array(scan_msg.ranges)
            vLinear, omega = self.wallFollower.step(ranges_np)    
            self.node.SetVelocities(omega, vLinear)
        elif self.State == self.StateIdle:
            return 0
            
        return None

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

    def RemoveEquations(self, A, b, lineNumbers):
        """ Verwende nur Zäune zur Lokalisierung, wenn mindesten zwei Zäune erkannt wurden """
        """ (dient zur Reduzierung von Fehlerkennungen) """
        numEq = len(self.b)
        zaun = [0, 0, 0]
        for i in range(numEq):
            if 0 <= lineNumbers[i] <= 2: zaun[lineNumbers[i]] = 1
        print(f"{zaun=}")
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
        print(f"{n} equations removed")
        return np.array(Anew), np.array(bnew), infoNew
    
    def Step(self, scan_msg):
        #self.simTimeSec = self.node.get_clock().now().nanoseconds / 1e9
        detectedWalls = self.node.Walldetector(scan_msg)        
        if self.node.wantedThetaReached:
            detectedWallsValid = Localization(self.node.theta, detectedWalls, self.A, self.b, self.wallNumbers)
            PublishMarkers(self.node.marker_pub, detectedWalls, detectedWallsValid)
            self.stateCounter += 1            
            if self.stateCounter >= 4:
                A, b, wallNumbers = self.RemoveEquations(self.A, self.b, self.wallNumbers)
                numEq = len(wallNumbers)
                x, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
                print(f"Lösung: ({x[0]:.2f}, {x[1]:.2f})    Rang:{rank}   Fehlerquadratsumme: {residuals}")                
                for i in range(numEq):
                    err = x[0]*A[i,0] + x[1]*A[i,1] - b[i]
                    print(f"{b[i]:6.2f} = {A[i,0]:6.2f}*x + {A[i,1]:6.2f}*y   {err=:6.2f}  # {lineNames[wallNumbers[i]]}")
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

FahreInDenWaldTaskList = [
    (LocalizationTask(),     None),
    (FollowPathTask(),       "Schuppen"),
    (LocalizationTask(),     None),
    (FollowPathTask(),       "Wald"),
    (GotoTask(), 0)
]

