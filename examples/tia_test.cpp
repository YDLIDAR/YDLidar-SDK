#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include "CYdLidar.h"
#include "core/common/ydlidar_help.h"

using namespace std;
using namespace ydlidar;
using namespace ydlidar::core::common;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif


int main(int argc, char *argv[])
{
  printLogo();
  os_init();

  std::string port = "192.168.0.11";
  int baudrate = 8090;
  bool isSingleChannel = false;
  float frequency = 30.0f; //TIA(10~30Hz),TIA-H(100Hz,150Hz)
  float samplerate = 20.0f; //TIA(10~30Hz),TIA-H(200Hz,300Hz)

  /// instance
  CYdLidar laser;
  //////////////////////string property/////////////////
  /// lidar port
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  /// ignore array
  std::string ignore_array;
  ignore_array.clear();
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                    ignore_array.size());

  //////////////////////int property/////////////////
  /// lidar baudrate
  laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
  /// tof lidar
  int optval = TYPE_TIA;
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  optval = YDLIDAR_TYPE_TCP;
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = samplerate;
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
//  optval = 16;
//  laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = false;
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  b_optvalue = false;
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  b_optvalue = false;
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  b_optvalue = isSingleChannel;
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
  /// intensity
  b_optvalue = true;
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  b_optvalue = false;
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  /// unit: m
  f_optvalue = 64.f;
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.05f;
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

  //启用调试
  // laser.setEnableDebug(true);

  /// initialize SDK and LiDAR.
  bool ret = laser.initialize();
  if (ret) 
  {
    /// Start the device scanning routine which runs on a separate thread and enable motor.
    ret = laser.turnOn();
  } 
  if (!ret)
  {
    error("Error %s", laser.DescribeError());
    return -1;
  }

  LaserScan scan; //点云
  while (ret && ydlidar::os_isOk())
  {
    if (laser.doProcessSimple(scan))
    {
      info("Scan received at [%.02f]Hz [%u] points in [%.03f]s",
          scan.scanFreq,
          int(scan.points.size()),
          scan.config.scan_time);
    }
    else
    {
      error("Failed to get lidar data");
    }
  }

  laser.turnOff();
  laser.disconnecting();

  return 0;
}
