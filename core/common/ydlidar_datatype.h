/*********************************************************************
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
#pragma once
#include <core/base/datatype.h>
#include <vector>
#include "ydlidar_def.h"

#define MAX_DEBUG_INDEX 14

/**
 * @brief The Laser Debug struct
 */
typedef struct  {
  uint8_t     cVer;
  uint8_t     debug2;
  uint8_t     hfVer;
  uint8_t     fVer;
  uint8_t     month;
  uint8_t     day;
  uint8_t     year;
  uint8_t     numH;
  uint8_t     numL;
  uint8_t     health;
  uint8_t     model; //雷达型号
  uint8_t     maxIndex;
} LaserDebug;


/**
 * @brief The Laser Scan Data struct
 * @par usage
 * @code
 * LaserScan data;
 * for(int i = 0; i < data.points.size(); i++) {
 *  //current LiDAR angle
 *  float angle = data.points[i].angle;
 *  //current LiDAR range
 *  float range = data.points[i].range;
 *  //current LiDAR intensity
 *  float intensity = data.points[i].intensity;
 *  //current LiDAR point stamp
 *  uint64_t timestamp = data.stamp + i * data.config.time_increment * 1e9;
 * }
 * LaserScanDestroy(&data);
 * @endcode
 * @par convert to ROS sensor_msgs::LaserScan
 * @code
 * LaserScan scan;
 * sensor_msgs::LaserScan scan_msg;
 * std::string frame_id = "laser_frame";
 * ros::Time start_scan_time;
 * start_scan_time.sec = scan.stamp/1000000000ul;
 * start_scan_time.nsec = scan.stamp%1000000000ul;
 * scan_msg.header.stamp = start_scan_time;
 * scan_msg.header.frame_id = frame_id;
 * scan_msg.angle_min =(scan.config.min_angle);
 * scan_msg.angle_max = (scan.config.max_angle);
 * scan_msg.angle_increment = (scan.config.angle_increment);
 * scan_msg.scan_time = scan.config.scan_time;
 * scan_msg.time_increment = scan.config.time_increment;
 * scan_msg.range_min = (scan.config.min_range);
 * scan_msg.range_max = (scan.config.max_range);
 * int size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;
 * scan_msg.ranges.resize(size);
 * scan_msg.intensities.resize(size);
 * for(int i=0; i < scan.points.size(); i++) {
 *  int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
 *  if(index >=0 && index < size) {
 *      scan_msg.ranges[index] = scan.points[i].range;
 *      scan_msg.intensities[index] = scan.points[i].intensity;
 *  }
 * }
 * @endcode
 */

struct LaserScan {
  // System time when first range was measured in nanoseconds
  uint64_t stamp = 0;
  // rpm
  float scanFreq = .0;
  // sampling rate
  float sampleRate = .0;
  // Array of lidar points
  std::vector<LaserPoint> points;
  // Actual number of points (when fixed resolution, the number of points does not match the actual number of points)
  int size = 0;
  // Configuration of scan
  LaserConfig config;
  int moduleNum = 0;
  // Environmental flags (currently only applicable to GS2)
  uint16_t envFlag = 0; 
} ;
