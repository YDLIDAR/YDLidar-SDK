import os
import ydlidar
import time

if __name__ == "__main__":
    ydlidar.os_init();
    laser = ydlidar.CYdLidar();
    laser.setlidaropt(ydlidar.LidarPropSerialPort, "192.168.1.11");
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 8000);
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF_NET);
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_TCP);
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 20.0);
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 20);
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, False);
    laser.setlidaropt(ydlidar.LidarPropIntenstiy, True);

    ret = laser.initialize();
    if ret:
        ret = laser.turnOn();
        scan = ydlidar.LaserScan();
        while ret and ydlidar.os_isOk() :
            r = laser.doProcessSimple(scan);
            if r:
                print("Scan received[",scan.stamp,"]:",scan.points.size(),"ranges is [",1.0/scan.config.scan_time,"]Hz");
            else :
                print("Failed to get Lidar Data")
            time.sleep(0.05);
        laser.turnOff();
    laser.disconnecting();
