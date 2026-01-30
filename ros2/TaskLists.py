import numpy as np
from Ransac import PublishMarkers
from GartenWorld import Localization, lineNames

class LocalizationTask:
    def __init__(self):
        self.STATE_INIT        = 0
        self.STATE_CHECK_NORTH = 1
        self.STATE_CHECK_EAST  = 2
        self.STATE_CHECK_SOUTH = 3
        self.STATE_CHECK_WEST  = 4

    def Init(self, node, params, retvals=None):
        self.node = node
        self.state = self.STATE_CHECK_NORTH
        self.node.SetWantedTheta(np.pi/2)
        self.A = []
        self.b = []
        self.wallNumbers = []

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
        self.simTimeSec = self.node.get_clock().now().nanoseconds / 1e9
        detectedWalls = self.node.Walldetector(scan_msg)        
        if self.state == self.STATE_CHECK_NORTH:
            if self.node.wantedThetaReached: # and self.node.simTimeSec > self.node.wantedThetaReachedTime + 0.0:
                detectedWallsValid = Localization(self.node.theta, detectedWalls, self.A, self.b, self.wallNumbers)
                PublishMarkers(self.node.marker_pub, detectedWalls, detectedWallsValid)
                # Nach Osten ausrichten
                self.node.SetWantedTheta(0.0)
                self.state = self.STATE_CHECK_EAST
            
        elif self.state == self.STATE_CHECK_EAST:
            if self.node.wantedThetaReached: # and self.node.simTimeSec > self.node.wantedThetaReachedTime + 0.0:
                detectedWallsValid = Localization(self.node.theta, detectedWalls, self.A, self.b, self.wallNumbers)
                PublishMarkers(self.node.marker_pub, detectedWalls, detectedWallsValid)
                # Nach Süden ausrichten
                self.node.SetWantedTheta(-np.pi/2)
                self.state = self.STATE_CHECK_SOUTH
            
        elif self.state == self.STATE_CHECK_SOUTH:
            if self.node.wantedThetaReached: # and self.node.simTimeSec > self.node.wantedThetaReachedTime + 0.0:
                detectedWallsValid = Localization(self.node.theta, detectedWalls, self.A, self.b, self.wallNumbers)
                PublishMarkers(self.node.marker_pub, detectedWalls, detectedWallsValid)
                # Nach Westen ausrichten
                self.node.SetWantedTheta(-np.pi)
                self.state = self.STATE_CHECK_WEST
                
        elif self.state == self.STATE_CHECK_WEST:
            if self.node.wantedThetaReached: # and self.node.simTimeSec > self.node.wantedThetaReachedTime + 0.0:
                detectedWallsValid = Localization(self.node.theta, detectedWalls, self.A, self.b, self.wallNumbers)
                PublishMarkers(self.node.marker_pub, detectedWalls, detectedWallsValid)
                
                A, b, wallNumbers = self.RemoveEquations(self.A, self.b, self.wallNumbers)
                numEq = len(wallNumbers)
                #print(f"--> {A.shape=}  {b.shape=}")

                # rcond=None unterdrückt eine Warnung und nutzt den Standard-Schwellenwert
                x, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)

                print(f"Lösung x={x[0]:6.2f}  y={x[1]:6.2f}    Rang:{rank}   Fehlerquadratsumme: {residuals}")                

                for i in range(numEq):
                    err = x[0]*A[i,0] + x[1]*A[i,1] - b[i]
                    print(f"{b[i]:6.2f} = {A[i,0]:6.2f}*x + {A[i,1]:6.2f}*y   {err=:6.2f}  # {lineNames[wallNumbers[i]]}")

                return x, A, b, wallNumbers

        return None

class GotoTask:
    def Init(self, node, taskIndex, retvals=None):
        node.GotoTask(taskIndex)

LocalizationTaskList = [
    (LocalizationTask(),     None),
    (GotoTask(), 0)
]
#taskListFahreInDenWald = [
#    (LocalizationTask(),     None),
#    (GotoWallTask(),        "Right"),
#    (FollowWallTask(),      "Right"),
#    (PassThroughGateTask(), "Wald"),
#]

