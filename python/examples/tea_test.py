import os
import ydlidar
import time
import sys

if __name__ == "__main__":
    ydlidar.os_init();
    port = "192.168.0.11";
    laser = ydlidar.CYdLidar();
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 8090);
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF);
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_TCP);
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 20.0);
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 20);
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, False);
    laser.setlidaropt(ydlidar.LidarPropIntenstiy, True);
    laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0);
    laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0);
    laser.setlidaropt(ydlidar.LidarPropMaxRange, 50.0);
    laser.setlidaropt(ydlidar.LidarPropMinRange, 0.01);

    ret = laser.initialize();
    if ret:
        ret = laser.turnOn();
        scan = ydlidar.LaserScan()
        while ret and ydlidar.os_isOk() :
            r = laser.doProcessSimple(scan);
            if r:
                print("Scan received [", scan.points.size(), "] points");
            else :
                print("Failed to get Lidar Data.")
            time.sleep(0.05);
        laser.turnOff();
    laser.disconnecting();
