import ydlidar
import time

# 1. LIDAR initialisieren
laser = ydlidar.CYdLidar()
laser.setlidaropt(ydlidar.LidarPropSerialPort, "/dev/ttyUSB0")
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 512000)  
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
laser.setlidaropt(ydlidar.LidarPropSampleRate, 9)
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)

# 2. Gerät öffnen
if laser.initialize():
    print("TG30 Initialisierung erfolgreich")
else:
    print("TG30 Initialisierung fehlgeschlagen.")
laser.turnOff()
laser.disconnecting()
