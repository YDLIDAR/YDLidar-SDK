﻿/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, EAIBOT, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include "CYdLidar.h"
#include "filters/NoiseFilter.h"
#include "core/common/ydlidar_help.h"

using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

/**
 * @brief tof Lidar test
 * @param argc
 * @param argv
 * @return
 * @par Flow chart
 * Step1: instance CYdLidar.\n
 * Step2: set paramters.\n
 * Step3: initialize SDK and LiDAR.(::CYdLidar::initialize)\n
 * Step4: Start the device scanning routine which runs on a separate thread and enable motor.(::CYdLidar::turnOn)\n
 * Step5: Get the LiDAR Scan Data.(::CYdLidar::doProcessSimple)\n
 * Step6: Stop the device scanning thread and disable motor.(::CYdLidar::turnOff)\n
 * Step7: Uninitialize the SDK and Disconnect the LiDAR.(::CYdLidar::disconnecting)\n
 */
int main(int argc, char *argv[]) {
  printf("__   ______  _     ___ ____    _    ____  \n");
  printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
  printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
  printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
  printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
  printf("\n");
  fflush(stdout);
  std::string port;
  ydlidar::os_init();

  std::map<std::string, std::string> ports = ydlidar::lidarPortList();
  std::map<std::string, std::string>::iterator it;

  if (ports.size() == 1) {
    port = ports.begin()->second;
  } else {
    int id = 0;//

    for (it = ports.begin(); it != ports.end(); it++) {
      printf("[%d] %s %s\n", id, it->first.c_str(), it->second.c_str());
      id++;
    }

    if (ports.empty()) {
      printf("Not Lidar was detected. Please enter the lidar serial port:");
      std::cin >> port;
    } else {
      while (ydlidar::os_isOk()) {
        printf("Please select the lidar port:");
        std::string number;
        std::cin >> number;

        if ((size_t)atoi(number.c_str()) >= ports.size()) {
          continue;
        }

        it = ports.begin();
        id = atoi(number.c_str());

        while (id) {
          id--;
          it++;
        }

        port = it->second;
        break;
      }
    }
  }

  int baudrate = 230400;
  std::map<int, int> baudrateList;
  baudrateList[0] = 115200;
  baudrateList[1] = 230400;
  baudrateList[2] = 460800;
  baudrateList[3] = 512000;

  printf("Baudrate:\n");

  for (std::map<int, int>::iterator it = baudrateList.begin();
       it != baudrateList.end(); it++) {
    printf("%d. %d\n", it->first, it->second);
  }

  while (ydlidar::os_isOk()) {
    printf("Please select the lidar baudrate:");
    std::string number;
    std::cin >> number;

    if ((size_t)atoi(number.c_str()) > baudrateList.size()) {
      continue;
    }

    baudrate = baudrateList[atoi(number.c_str())];
    break;
  }

  if (!ydlidar::os_isOk()) {
    return 0;
  }

  bool isSingleChannel = false;
  std::string input_channel;
  printf("Whether the Lidar is one-way communication[yes/no]:");
  std::cin >> input_channel;
  std::transform(input_channel.begin(), input_channel.end(),
                 input_channel.begin(),
  [](unsigned char c) {
    return std::tolower(c);  // correct
  });

  if (input_channel.find("y") != std::string::npos) {
    isSingleChannel = true;
  }

  if (!ydlidar::os_isOk()) {
    return 0;
  }

  std::string input_frequency;
  float frequency = 8.0f;
  while (ydlidar::os_isOk() && !isSingleChannel) {
    printf("Please enter the lidar scan frequency[3-15.7]:");
    std::cin >> input_frequency;
    frequency = atof(input_frequency.c_str());

    if (frequency <= 15.7 && frequency >= 3.0) {
      break;
    }

    fprintf(stderr, "Invalid scan frequency,"
      "The scanning frequency range is 3 to 15.7 HZ, Please re-enter.\n");
  }

  if (!ydlidar::os_isOk()) {
    return 0;
  }

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
  int optval = TYPE_TOF;
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  optval = YDLIDAR_TYPE_SERIAL;
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = isSingleChannel ? 4 : 20;
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
  b_optvalue = false;
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
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

  /// initialize SDK and LiDAR.
  bool ret = laser.initialize();

  if (ret) {//success
    /// Start the device scanning routine which runs on a separate thread and enable motor.
    ret = laser.turnOn();
  } else {//failed
    fprintf(stderr, "%s\n", laser.DescribeError());
    fflush(stderr);
  }

  LaserScan scan;
  NoiseFilter filter;
  filter.setStrategy(NoiseFilter::FS_TailWeek);

  //main loop
  uint64_t stamp = 0;
  while (ret && ydlidar::os_isOk())
  {
    if (laser.doProcessSimple(scan))
    {
      // core::common::info("Scan received at [%f]Hz %u points is [%f]s\n",
      //         scan.scanFreq,
      //         (unsigned int)scan.points.size(),
      //         scan.config.scan_time);
      // 使用拖尾滤波器
      //  filter.filter(scan, 0, 0, scan);
      // 打印点云
      // for (size_t i = 0; i < scan.points.size(); ++i)
      // {
      //   const LaserPoint &p = scan.points.at(i);
      //   core::common::info("%d a %.02f d %.03f i %.0f\n",
      //     i, p.angle * 180 / M_PI, p.range, p.intensity);
      // }
      //打印首点和尾点时间戳
      if (scan.points.size())
      {
        //如果上一圈末点时间戳大于当前圈首点时间戳则打印警告信息，否则打印正常信息
        uint64_t lastStamp = scan.stamp + 
          uint64_t((scan.points.size() - 1) * 1e9 * scan.config.time_increment);
        if (scan.stamp < stamp)
          core::common::warn("first %llu last %llu size %lu",
            scan.stamp, 
            lastStamp,
            scan.points.size());
        else
          core::common::info("first %llu last %llu size %lu",
            scan.stamp, 
            lastStamp,
            scan.points.size());
        stamp = lastStamp;
      }
    }
    else
    {
      core::common::error("Failed to get Lidar Data");
    }
  }

  laser.turnOff();
  laser.disconnecting();

  return 0;
}
