import os
import ydlidar
import time
import sys

if __name__ == "__main__":
    ydlidar.os_init();
    ports = ydlidar.lidarPortList();
    port = "/dev/ydlidar";
    for key, value in ports.items():
        port = value;
        print(port);
    laser = ydlidar.CYdLidar();
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 921600);
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_GS);
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL);
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 30.0);
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
                print("Scan received[",scan.stamp,"]:",scan.points.size());
            else :
                print("Failed to get Lidar Data.")
            time.sleep(0.05);
        laser.turnOff();
    laser.disconnecting();
