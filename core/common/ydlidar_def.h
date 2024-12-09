//
// The MIT License (MIT)
//
// Copyright (c) 2020 EAIBOT. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef YDLIDAR_DEF_H_
#define YDLIDAR_DEF_H_

#include <stdint.h>
#include <stdlib.h>
#include <core/base/typedef.h>
#include <core/base/utils.h>
#ifdef __cplusplus
extern "C" {
#endif

/** Device Type ID */
typedef enum {
  YDLIDAR_TYPE_SERIAL = 0x0,/**< serial type.*/
  YDLIDAR_TYPE_TCP = 0x1,/**< socket tcp type.*/
  YDLIDAR_TYPC_UDP = 0x2,/**< socket udp type.*/
} DeviceTypeID;

/** Lidar Type ID */
typedef enum {
  TYPE_TOF = 0, //TG系列雷达
  TYPE_TRIANGLE = 1, //S2、S2 Pro、S4、G4、G6、G2、Tmini等三角协议雷达
  TYPE_TOF_NET = 2, //T系列雷达
  TYPE_GS = 3, //GS系列雷达（目前只有GS2）
  TYPE_SCL = 4, //SCL雷达
  TYPE_SDM = 5, //SDM15单点雷达
  TYPE_SDM18 = 6, //SDM18单点雷达
  TYPE_TIA = 7, //TIA系列雷达
  TYPE_Tail,
} LidarTypeID;


/** Lidar Properties,Lidar Can set and get parameter property index.\n
 * float properties must be float type, not double type.
*/
typedef enum {
  /* char* properties */
  LidarPropSerialPort = 0,/**< Lidar serial port or network ipaddress */
  LidarPropIgnoreArray,/**< Lidar ignore angle array */
  /* int properties */
  LidarPropSerialBaudrate = 10,/**< lidar serial baudrate or network port */
  LidarPropLidarType,/**< lidar type code */
  LidarPropDeviceType,/**< lidar connection type code */
  LidarPropSampleRate,/**< lidar sample rate */
  LidarPropAbnormalCheckCount,/**< abnormal maximum check times */
  LidarPropIntenstiyBit,/**< lidar intensity bit count */
  /* float properties */
  LidarPropMaxRange = 20,/**< lidar maximum range */
  LidarPropMinRange,/**< lidar minimum range */
  LidarPropMaxAngle,/**< lidar maximum angle */
  LidarPropMinAngle,/**< lidar minimum angle */
  LidarPropScanFrequency,/**< lidar scanning frequency */
  /* bool properties */
  LidarPropFixedResolution = 30,/**< fixed angle resolution flag */
  LidarPropReversion,/**< lidar reversion flag */
  LidarPropInverted,/**< lidar inverted flag */
  LidarPropAutoReconnect,/**< lidar hot plug flag */
  LidarPropSingleChannel,/**< lidar single-channel flag */
  LidarPropIntenstiy,/**< lidar intensity flag */
  LidarPropSupportMotorDtrCtrl,/**< lidar support motor Dtr ctrl flag */
  LidarPropSupportHeartBeat,/**< lidar support heartbeat flag */
} LidarProperty;

/// lidar instance
typedef struct {
  void *lidar;///< CYdLidar instance
} YDLidar;

typedef enum {
  NoError = 0,
  DeviceNotFoundError,
  PermissionError,
  UnsupportedOperationError,
  UnknownError,
  TimeoutError,
  NotOpenError,
  BlockError,
  NotBufferError,
  TrembleError,
  LaserFailureError,
} DriverError;

#pragma pack(1)

/**
 * @brief The Laser Point struct
 * @note angle unit: rad.\n
 * range unit: meter.\n
 */
typedef struct {
  /// lidar angle. unit(rad)
  float angle;
  /// lidar range. unit(m)
  float range;
  /// lidar intensity
  float intensity;
} LaserPoint;

/**
 * @brief A struct for returning configuration from the YDLIDAR
 * @note angle unit: rad.\n
 * time unit: second.\n
 * range unit: meter.\n
 */
typedef struct {
  /// Start angle for the laser scan [rad].  0 is forward and angles are measured clockwise when viewing YDLIDAR from the top.
  float min_angle;
  /// Stop angle for the laser scan [rad].   0 is forward and angles are measured clockwise when viewing YDLIDAR from the top.
  float max_angle;
  /// angle resoltuion [rad]
  float angle_increment;
  /// Scan resoltuion [s]
  float time_increment;
  /// Time between scans 扫描时长，单位秒
  float scan_time;
  /// Minimum range [m]
  float min_range;
  /// Maximum range [m]
  float max_range;
} LaserConfig;


/**
 * @brief The Laser Scan Data struct
 * @par usage
 * @code
 * LaserScan data;
 * for(int i = 0; i < data.npoints; i++) {
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
 * for(int i=0; i < scan.npoints; i++) {
 *  int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
 *  if(index >=0 && index < size) {
 *      scan_msg.ranges[index] = scan.points[i].range;
 *      scan_msg.intensities[index] = scan.points[i].intensity;
 *  }
 * }
 * LaserScanDestroy(&scan);
 * @endcode
 */

typedef struct {
  /// System time when first range was measured in nanoseconds
  uint64_t stamp;///< ns
  /// Array of lidar points
  uint32_t npoints;
  LaserPoint *points;
  /// Configuration of scan
  LaserConfig config;
} LaserFan;

/**
  * @brief c string
  */
typedef struct {
  /// data
  char data[50];
} string_t;

/**
  * @brief lidar ports
  */
typedef struct {
  string_t port[8];
} LidarPort;

/** The numeric version information struct.  */
typedef struct {
  uint8_t hardware;   /**< Hardware version*/
  uint8_t soft_major;      /**< major number */
  uint8_t soft_minor;      /**< minor number */
  uint8_t soft_patch;      /**< patch number */
  uint8_t sn[16];     /**< serial number*/
} LidarVersion;

#pragma pack()

/**
 * @brief initialize LaserFan
 * @param to_init
 */
YDLIDAR_API void LaserFanInit(LaserFan *to_init);
/**
 * Destroy an instance of LaserFan points
 */
YDLIDAR_API void LaserFanDestroy(LaserFan *to_destroy);

#ifdef __cplusplus
}
#endif


#endif  // YDLIDAR_DEF_H_
