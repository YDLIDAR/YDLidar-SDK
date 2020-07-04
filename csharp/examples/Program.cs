using System;

namespace LidarTest
{
    class Program
    { 
        static void Main(string[] args)
        {
            ydlidar.os_init();
            CYdLidar lidarClass = new CYdLidar();
            string port = "COM7";
            int optname = (int)LidarProperty.LidarPropSerialPort;
            lidarClass.setlidaropt(optname, port);
            optname = (int)LidarProperty.LidarPropSerialBaudrate;
            lidarClass.setlidaropt(optname, 512000);

            optname = (int)LidarProperty.LidarPropLidarType;

            int lidarType = (int)LidarTypeID.TYPE_TOF;
            lidarClass.setlidaropt(optname, lidarType);


            bool ret  = lidarClass.initialize();
            if(ret)
            {
                ret = lidarClass.turnOn();
            } else
            {
                Console.WriteLine("error:" + lidarClass.DescribeError());
            }
            LaserScan scan = new LaserScan();
            while (ret && ydlidar.os_isOk())
            {
                if(lidarClass.doProcessSimple(scan))
                {
                    Console.WriteLine("stamp: "+ scan.stamp + ", size: " + scan.points.Count);
                }
            }
            lidarClass.turnOff();
            lidarClass.disconnecting();
            lidarClass.Dispose();

        }
    }
}
