import numpy as np
import sys
from sim import Simulation
from Navigator import Navigator
import params
from params import Udp
from UdpSend import UdpSend


lidarCounter = 0
        

def SendLidarData(dist):
    # # --- PUBLISH SCAN ---
    # scan = LaserScan()
    # # 2. Zur Sicherheit: time_increment nullen
    # scan.time_increment = 0.0
    # #current_sim_time = Time(seconds=self.sim.sim_time_sec)
    # #scan.header.stamp = current_sim_time.to_msg()
    # scan.header.stamp = self.get_clock().now().to_msg()
    # scan.header.frame_id = 'lidar'      # wenn Lidar vor der Achsenmitte montiert ist
    # #scan.header.frame_id = 'base_link'  # wenn Lidar direkt in Achsenmitte montiert ist
    # #scan.time_increment = self.scanTimeInc
    # scan.angle_min = math.radians(1-params.LidarMaxAngle)
    # scan.angle_max = math.radians(params.LidarMaxAngle)
    # #scan.angle_min = math.radians(1-params.LidarMaxAngle)  HB old version
    # #scan.angle_max = math.radians(params.LidarMaxAngle)
    # num_readings = 2*params.LidarMaxAngle
    # scan.angle_increment = (scan.angle_max - scan.angle_min) / (num_readings - 1)
    # scan.angle_max = scan.angle_min + (scan.angle_increment * (num_readings - 1))
    # 
    # scan.range_min = params.LidarRangeMin
    # scan.range_max = params.LidarRangeMax
    
    dist = np.clip(dist, params.LidarRangeMin, params.LidarRangeMax)
    # scan.ranges = dist.tolist()
    
    cm = 100.0
    dist_cm = dist*cm
    dist_cm = dist_cm.astype(np.int16)
    UdpSend(Udp.LIDAR_DATA, dist_cm.tolist())

            
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
        UdpSend(udp_header, udp_data)

    # Kleine Erfolgsmeldung alle 100 Pakete
    if lidarCounter % 100 == 0:
        theta_deg = int(np.rad2deg(theta))
        #print(f"[{self.sim.sim_time_sec:.3f}] Sende Position & Time #  {posX=:6.2f} {posY=:6.2f} {theta_deg}°")
        print(f" Sende Position & Time #  {posX=:6.2f} {posY=:6.2f} {theta_deg}°")
    lidarCounter += 1


def main():
    sim = Simulation()
    navigator = Navigator(sim.SetRobotSpeed)
    try:
        # Die Schleife läuft nur, solange sim.running UND ROS okay ist
        while sim.running:
            x, y, theta, radius = sim.Step()        # Simulations-Schritt
            if not sim.pause:
                navigator.CompassCallback(theta)    # Publish Theta
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
        
        # Optional: Komplettes Beenden erzwingen (hilft bei WSL2-Hängern)
        sys.exit(0)

    
if __name__ == '__main__':
    main()