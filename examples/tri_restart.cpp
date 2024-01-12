#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include "core/base/timer.h"

using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

int main(int argc, char *argv[])
{
  printf("__   ______  _     ___ ____    _    ____  \n");
  printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
  printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
  printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
  printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
  printf("\n");
  fflush(stdout);

  ydlidar::os_init();

  bool ret = false;
 
  CYdLidar lidarS2; //S2雷达
  {
    bool isSingleChannel = false;
    float frequency = 8.0;
    std::string port = "/dev/ttyUSB0";
    int baudrate = 115200;
    //////////////////////string property/////////////////
    /// lidar port
    lidarS2.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
    //////////////////////int property/////////////////
    /// lidar baudrate
    lidarS2.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
    /// tof lidar
    int optval = TYPE_TRIANGLE;
    lidarS2.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
    /// device type
    optval = YDLIDAR_TYPE_SERIAL;
    lidarS2.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
    /// sample rate
    optval = isSingleChannel ? 3 : 4;
    lidarS2.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
    /// abnormal count
    optval = 4;
    lidarS2.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
    /// Intenstiy bit count
    optval = 10;
    lidarS2.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));
    //////////////////////bool property/////////////////
    /// fixed angle resolution
    bool b_optvalue = false;
    lidarS2.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
    b_optvalue = false;
    /// rotate 180
    lidarS2.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
    /// Counterclockwise
    lidarS2.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
    b_optvalue = true;
    lidarS2.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
    /// one-way communication
    lidarS2.setlidaropt(LidarPropSingleChannel, &isSingleChannel, sizeof(bool));
    /// intensity
    b_optvalue = true;
    lidarS2.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
    /// Motor DTR
    b_optvalue = false;
    lidarS2.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));
    /// HeartBeat
    b_optvalue = false;
    lidarS2.setlidaropt(LidarPropSupportHeartBeat, &b_optvalue, sizeof(bool));
    //////////////////////float property/////////////////
    /// unit: °
    float f_optvalue = 180.0f;
    lidarS2.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
    f_optvalue = -180.0f;
    lidarS2.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
    /// unit: m
    f_optvalue = 64.f;
    lidarS2.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
    f_optvalue = 0.05f;
    lidarS2.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
    /// unit: Hz
    lidarS2.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));
  }

  LaserScan scanGs; //GS2点云数据
  LaserScan scanS2; //S2雷达点云数据
  while (ydlidar::os_isOk())
  {
    ret = lidarS2.initialize();
    if (!ret)
    {
      fprintf(stderr, "Fail to initialize %s\n", lidarS2.DescribeError());
      fflush(stderr);
      return -1;
    }
    //启动S2
    ret = lidarS2.turnOn();
    if (!ret)
    {
      fprintf(stderr, "Fail to turn on S2 %s\n", lidarS2.DescribeError());
      fflush(stderr);
      return -1;
    }
    //启动后运行5秒然后停止扫描
    uint64_t t = getms();
    while (getms() - t < 5000)
    {
      //获取S2点云数据
      if (lidarS2.doProcessSimple(scanS2))
      {
        printf("[%u] points inc [%f]\n",
               (unsigned int)scanS2.points.size(),
               scanS2.config.angle_increment);
        fflush(stdout);
      }
      else
      {
        fprintf(stderr, "Failed to get S2 ldiar data\n");
        fflush(stderr);
        static int s_errorCount = 0;
        if (s_errorCount++ > 10)
          return -1;
      }
    }

    //停止S2
    lidarS2.turnOff();
  }

  lidarS2.turnOff();
  lidarS2.disconnecting();

  return 0;
}
