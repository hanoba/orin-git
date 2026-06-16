import numpy as np
import sys
from sim import Simulation
from Navigator import Navigator
import params
from params import Udp
from UdpReceive import UdpReceive
import TaskLists


lidarCounter = 0
        


def main():
    udp_rx = UdpReceive(Udp.PORT_TELEOP)
    
    # Tasklist dictionary
    TaskListDict = {
        "Localization":           (TaskLists.Localization_TaskList,        15.00,  9.00,  np.pi  ), # Im Garten beim Gartentor
        "FastLocalization":       (TaskLists.FastLocalization_TaskList,    15.00,  9.00,  np.pi  ), # Im Garten beim Gartentor
        "Mowing":                 (TaskLists.Mowing_TaskList,              18.00,  0.00,   0.0   ), # für Mow Test
        "Fahre_zum_Schuppen":     (TaskLists.Fahre_zum_Schuppen_TaskList,  15.00,  9.00,  np.pi  ), # Im Garten beim Gartentor
        "Fahre_in_den_Wald":      (TaskLists.Fahre_in_den_Wald_TaskList,   15.00,  9.00,  np.pi  ), # Im Garten beim Gartentor
        "Fahre_in_den_Garten":    (TaskLists.Fahre_in_den_Garten_TaskList, 19.00, 15.00, -np.pi/2), # Im Wald beim Gartentor
        "Fahre_hinters_Haus":     (TaskLists.Fahre_hinters_Haus_TaskList,  -2.00, 10.50,   0.0   ), # rechts vom Schuppen 
        "Bestimme_YawOffset":     (TaskLists.Bestimme_YawOffset_TaskList,  -9.00,  9.00,  np.pi/2), # vor der Schuppentür
        "Test":                   (TaskLists.Test_TaskList,                12.00, -3.00,   0.0   )  # unterhalb der Terrasse
    }


    def Usage():
        print("Usage: ekarren <taskName>")
        print("<taskName>:")
        index = 0
        for taskName in TaskListDict:
            print(f"    {index} {taskName}")
            index += 1
        sys.exit(1)

    def SendLidarData(dist):
        dist = np.clip(dist, params.LidarRangeMin, params.LidarRangeMax)
        cm = 100.0
        dist_cm = dist*cm
        dist_cm = dist_cm.astype(np.int16)
        navigator.udp.Send(Udp.LIDAR_DATA, dist_cm.tolist())

                
    def SendPositionAndTime(posX, posY, theta):
        global lidarCounter
        if not params.PublishEstimatedPosition:
            # POSE an Visualizer senden
            cm = 100.0
            theta_deg = np.degrees(theta)
            udp_header = Udp.POSE
            udp_data = [
                # round(x) gibt in Python 3 automatisch einen Integer zurück
                round(posX*cm),       # X-Koordinate in cm
                round(posY*cm),       # Y-Koordinate in cm
                round(theta_deg)      # Yaw in Grad
            ]
            navigator.udp.Send(udp_header, udp_data)

        # Kleine Erfolgsmeldung alle 100 Pakete
        # if lidarCounter % 100 == 0:
        #     theta_deg = int(np.rad2deg(theta))
        #     #print(f"[{self.sim.sim_time_sec:.3f}] Sende Position & Time #  {posX=:6.2f} {posY=:6.2f} {theta_deg}°")
        #     print(f" Sende Position & Time #  {posX=:6.2f} {posY=:6.2f} {theta_deg}°")
        # lidarCounter += 1


    # command line parameter handling
    argc = len(sys.argv)
    (taskList, x, y, yaw) = (None, 0.0, 0.0, 0.0)
    if argc == 2:
        arg = sys.argv[1]
        if arg.isdigit():
            parameters = list(TaskListDict.values())[int(arg)]
            print(parameters)
        else:
            parameters = TaskListDict.get(arg)
        if parameters is None: Usage()
        (taskList, x, y, yaw) = parameters
    elif argc > 2: Usage()

    sim = Simulation(x, y, yaw)
    navigator = Navigator()

    if taskList is not None:
        navigator.NewTaskList(taskList)
    
    try:
        while sim.running:
            x, y, theta, radius = sim.Step()        # Simulations-Schritt
            if not sim.pause:
                vLinear, omega = navigator.CompassCallback(theta)   # Publish Theta
                vLinear, omega = udp_rx.ReceiveTeleop(vLinear, omega)   # check for teleop command
                
                sim.SetRobotSpeed(vLinear, omega)
                SendPositionAndTime(x, y, theta)    # Send Pose to viz.py
                if len(radius) > 0: 
                    navigator.ScanCallback(radius)  # Publish lidar data
                    SendLidarData(radius)           # Send lidar data to viz.py
    except KeyboardInterrupt:
        # Wird ausgelöst, wenn du Ctrl+C drückst
        print("Simulation wird durch Benutzer abgebrochen...")
    except Exception as e:
        # Fängt unerwartete Fehler ab
        print(f"Unerwarteter Fehler: {e}")
    finally:
        # Dieser Block wird IMMER ausgeführt, egal ob Fehler oder Ctrl+C
        print("Bereinige Ressourcen...")
        sim.Quit() 
        navigator.trace.Print()
        
        # Optional: Komplettes Beenden erzwingen (hilft bei WSL2-Hängern)
        sys.exit(0)

    
if __name__ == '__main__':
    main()