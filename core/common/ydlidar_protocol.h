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
#include <core/base/v8stdint.h>
#include <vector>
#include <functional>

/// Count the number of elements in a statically allocated array.
#if !defined(_countof)
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

/**
 * @name PI constant
 * @{
 */
#ifndef M_PI
#define M_PI 3.1415926
#endif
/** @}
*/

/**
 * @name sun noise flag constant
 * @{
 */
#define SUNNOISEINTENSITY 0x03
/** @}
*/

/**
 * @name glass noise flag constant
 * @{
 */
#define GLASSNOISEINTENSITY 0x02
/** @}
*/

/**@name LIDAR CMD Protocol
* @brief LiDAR request and response CMD
* @{
*/
#define LIDAR_CMD_STOP                      0x65
#define LIDAR_CMD_SCAN                      0x60
#define LIDAR_CMD_FORCE_SCAN                0x61
#define LIDAR_CMD_RESET                     0x80
#define LIDAR_CMD_FORCE_STOP                0x00
#define LIDAR_CMD_GET_EAI                   0x55
#define LIDAR_CMD_GET_DEVICE_INFO           0x90
#define LIDAR_CMD_GET_DEVICE_HEALTH         0x92
#define LIDAR_ANS_TYPE_DEVINFO              0x4
#define LIDAR_ANS_TYPE_DEVHEALTH            0x6
#define LIDAR_CMD_SYNC_BYTE                 0xA5
#define LIDAR_CMDFLAG_HAS_PAYLOAD           0x80
#define LIDAR_ANS_SYNC_BYTE1                0xA5
#define LIDAR_ANS_SYNC_BYTE2                0x5A
#define LIDAR_ANS_TYPE_MEASUREMENT          0x81
#define LIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define LIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1
#define LIDAR_RESP_MEASUREMENT_DISTANCE_SHIFT  2
#define LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT 8

#define LIDAR_CMD_RUN_POSITIVE             0x06
#define LIDAR_CMD_RUN_INVERSION            0x07
#define LIDAR_CMD_SET_AIMSPEED_ADDMIC      0x09
#define LIDAR_CMD_SET_AIMSPEED_DISMIC      0x0A
#define LIDAR_CMD_SET_AIMSPEED_ADD         0x0B
#define LIDAR_CMD_SET_AIMSPEED_DIS         0x0C
#define LIDAR_CMD_GET_AIMSPEED             0x0D

#define LIDAR_CMD_SET_SAMPLING_RATE        0xD0
#define LIDAR_CMD_GET_SAMPLING_RATE        0xD1
#define LIDAR_STATUS_OK                    0x0
#define LIDAR_STATUS_WARNING               0x1
#define LIDAR_STATUS_ERROR                 0x2

#define LIDAR_CMD_ENABLE_LOW_POWER         0x01
#define LIDAR_CMD_DISABLE_LOW_POWER        0x02
#define LIDAR_CMD_STATE_MODEL_MOTOR        0x05
#define LIDAR_CMD_ENABLE_CONST_FREQ        0x0E
#define LIDAR_CMD_DISABLE_CONST_FREQ       0x0F

#define LIDAR_CMD_GET_OFFSET_ANGLE          0x93
#define LIDAR_CMD_SAVE_SET_EXPOSURE         0x94
#define LIDAR_CMD_SET_LOW_EXPOSURE          0x95
#define LIDAR_CMD_ADD_EXPOSURE       	    0x96
#define LIDAR_CMD_DIS_EXPOSURE       	    0x97

#define LIDAR_CMD_SET_HEART_BEAT            0xD9

//GS2命令
#define GS_LIDAR_CMD_GET_ADDRESS               0x60
#define GS_LIDAR_CMD_GET_PARAMETER             0x61
#define GS_LIDAR_CMD_GET_VERSION               0x62
#define GS_LIDAR_CMD_SCAN                      0x63
#define GS_LIDAR_ANS_SCAN                      0x63
#define GS_LIDAR_CMD_STOP                      0x64
#define GS_LIDAR_CMD_RESET                     0x67
#define GS_LIDAR_CMD_SET_MODE                  0x69
#define GS_LIDAR_CMD_SET_BIAS                  0xD9
#define GS_LIDAR_CMD_SET_DEBUG_MODE            0xF0

/** @} LIDAR CMD Protocol */

//GS2
#define Angle_Px   1.22
#define Angle_Py   5.315
#define Angle_PAngle   22.5
#define PackageMaxModuleNums  0x03
#define MaxPointsPerPackge_GS2 160  //GS2固定160个点
#define MaxPointsPerPackge_GS1 216  //GS1固定216个点
#define PackagePaidBytes_GS 8
#define NORMAL_PACKAGE_SIZE 331

/// Maximuum number of samples in a packet
#define PackageSampleMaxLngth 0x100
#define MaximumNumberOfPackages 765 //= 255 * 3

/// CT Package Type
typedef enum {
  CT_Normal = 0,///< Normal package
  CT_RingStart  = 1,///< Starting package
  CT_Tail,
} CT;

/// Default Node Quality
#define Node_Default_Quality (10)
/// Starting Node
#define Node_Sync 1
/// Normal Node
#define Node_NotSync 2
/// Package Header Size
#define PackagePaidBytes 10
/// Package Header
#define PH 0x55AA
#define PH1 0xAA
#define PH2 0x55 //AA55是点云数据
#define PH3 0x66 //AA66是时间戳数据

/// Normal Package size
#define TrianglePackageDataSize 40
/// TOF Normal package size
#define TOFPackageDataSize 80

#define FREINDEX 0
#define USERVERSIONNDEX 1
#define HEALTHINDEX 3

/// ET LiDAR Protocol Type
typedef enum {
  Protocol_V1 = 0,///< V1 version
  Protocol_V2  = 1,///< V2 version
} ProtocolVer;

#if defined(_WIN32)
#pragma pack(1)
#endif

//雷达节点信息
struct node_info {
  uint8_t sync_flag; //首包标记
  uint8_t is; //抗干扰标志
  uint16_t sync_quality; //信号强度
  uint16_t angle_q6_checkbit; //角度值（°）
  uint16_t distance_q2; //距离值
  uint64_t stamp; //时间戳
  uint32_t delay_time; ///< delay time
  uint8_t scan_frequence; //扫描频率
  uint8_t debugInfo; ///< debug information
  uint8_t index; //包序号
  uint8_t error_package; ///< error package state
} __attribute__((packed));

/// package node info
struct PackageNode {
  uint8_t PakageSampleQuality;///< intensity
  uint16_t PakageSampleDistance;///< range
} __attribute__((packed));

/// TOF Intensity package node info
struct TOFPackageNode {
  uint16_t PakageSampleQuality;
  uint16_t PakageSampleDistance;
} __attribute__((packed));

/// LiDAR Intensity Nodes Package
struct node_package {
  uint16_t  package_Head;///< package header
  uint8_t   package_CT;///< package ct
  uint8_t   nowPackageNum;///< package number
  uint16_t  packageFirstSampleAngle;///< first sample angle
  uint16_t  packageLastSampleAngle;///< last sample angle
  uint16_t  checkSum;///< checksum
  PackageNode  packageSample[PackageSampleMaxLngth];
} __attribute__((packed)) ;

/// TOF LiDAR Intensity Nodes Package
struct tof_node_package {
  uint16_t  package_Head;
  uint8_t   package_CT;
  uint8_t   nowPackageNum;
  uint16_t  packageFirstSampleAngle;
  uint16_t  packageLastSampleAngle;
  uint16_t  checkSum;
  TOFPackageNode  packageSample[PackageSampleMaxLngth];
} __attribute__((packed)) ;

/// LiDAR Normal Nodes package
struct node_packages {
  uint16_t  package_Head;///< package header
  uint8_t   package_CT;///< package ct
  uint8_t   nowPackageNum; ///< package number
  uint16_t  packageFirstSampleAngle;///< first sample angle
  uint16_t  packageLastSampleAngle;///< last sample angle
  uint16_t  checkSum;///< checksum
  uint16_t  packageSampleDistance[PackageSampleMaxLngth];
} __attribute__((packed)) ;

//时间戳结构体
struct stamp_package {
  uint8_t flag1; //包头标记1
  uint8_t flag2; //包头标记2
  uint8_t cs; //校验和
  uint32_t stamp; //时间戳
  uint8_t reserved; //保留字段
} __attribute__((packed));
#define SIZE_STAMPPACKAGE sizeof(stamp_package)

/// LiDAR Device Information
struct device_info {
  uint8_t   model; ///< LiDAR model
  uint16_t  firmware_version; ///< firmware version
  uint8_t   hardware_version; ///< hardare version
  uint8_t   serialnum[16];    ///< serial number
} __attribute__((packed)) ;

/// LiDAR Health Information
struct device_health {
  uint8_t   status; ///< health state
  uint16_t  error_code; ///< error code
} __attribute__((packed))  ;

/// LiDAR sampling Rate struct
struct sampling_rate {
  uint8_t rate;	///< sample rate
} __attribute__((packed))  ;

/// LiDAR scan frequency struct
struct scan_frequency {
  uint32_t frequency;	///< scan frequency
} __attribute__((packed))  ;

struct scan_rotation {
  uint8_t rotation;
} __attribute__((packed))  ;

/// LiDAR Exposure struct
struct scan_exposure {
  uint8_t exposure;	///< low exposure
} __attribute__((packed))  ;

/// LiDAR Heart beat struct
struct scan_heart_beat {
  uint8_t enable;	///< heart beat
} __attribute__((packed));

struct scan_points {
  uint8_t flag;
} __attribute__((packed))  ;

struct function_state {
  uint8_t state;
} __attribute__((packed))  ;

/// LiDAR Zero Offset Angle
struct offset_angle {
  int32_t angle;
} __attribute__((packed))  ;

/// LiDAR request command packet
struct cmd_packet {
  uint8_t syncByte;
  uint8_t cmd_flag;
  uint8_t size;
  uint8_t data;
} __attribute__((packed)) ;

/// LiDAR response Header
struct lidar_ans_header {
  uint8_t  syncByte1;
  uint8_t  syncByte2;
  uint32_t size: 30;
  uint32_t subType: 2;
  uint8_t  type;
} __attribute__((packed));

//GS1
struct GS1_Multi_Package {
    int frameNum;
    int moduleNum;
    bool left = false;
    bool right = false;
    node_info all_points[MaxPointsPerPackge_GS1];
} __attribute__((packed));
//GS2
struct GS2_Multi_Package {
    int frameNum;
    int moduleNum;
    bool left = false;
    bool right = false;
    node_info all_points[MaxPointsPerPackge_GS2];
} __attribute__((packed));

struct GS2PackageNode {
  uint16_t PakageSampleDistance : 9;
  uint16_t PakageSampleQuality : 7;
} __attribute__((packed));

struct gs1_node_package {
  uint32_t  package_Head;
  uint8_t   address;
  uint8_t   package_CT;
  uint16_t  size;
  uint16_t  BackgroudLight;
  GS2PackageNode  packageSample[MaxPointsPerPackge_GS1];
  uint8_t  checkSum;
} __attribute__((packed));

struct gs2_node_package {
  uint32_t  package_Head;
  uint8_t   address;
  uint8_t   package_CT;
  uint16_t  size;
  uint16_t  BackgroudLight;
  GS2PackageNode  packageSample[MaxPointsPerPackge_GS2];
  uint8_t  checkSum;
} __attribute__((packed));

struct gs_lidar_ans_header {
    uint8_t  syncByte0;
    uint8_t  syncByte1;
    uint8_t  syncByte2;
    uint8_t  syncByte3;
    uint8_t  address;
    uint8_t  type;
    uint16_t size;
} __attribute__((packed));

struct gs_device_para {
    uint16_t u_compensateK0;
    uint16_t u_compensateB0;
    uint16_t u_compensateK1;
    uint16_t u_compensateB1;
    int8_t  bias;
    uint8_t  crc;
} __attribute__((packed));

struct cmd_packet_gs {
    uint8_t syncByte0;
    uint8_t syncByte1;
    uint8_t syncByte2;
    uint8_t syncByte3;
    uint8_t address;
    uint8_t cmd_flag;
    uint16_t size;
} __attribute__((packed));
//GS系列设备信息
struct gs_device_info {
    uint8_t hardware_version; //硬件版本号
    uint16_t firmware_version; //固件版本号
    uint8_t serialnum[16]; //序列号
} __attribute__((packed));

#if defined(_WIN32)
#pragma pack()
#endif

/**
 * @brief UDP Data format
 */
typedef struct _dataFrame {
  uint16_t frameHead;
  uint8_t deviceType;
  uint8_t frameType;
  uint8_t dataIndex;
  uint8_t frameIndex;
  uint32_t timestamp;
  uint8_t headFrameFlag;
  uint8_t dataFormat;
  uint8_t disScale;
  uint32_t startAngle;
  uint32_t dataNum;
  uint32_t frameCrc;
  uint8_t frameBuf[2048];
} dataFrame;

/**
 * @class lidarConfig
 * @brief Structure containing scan configuration.
 *
 * @author jzhang
 */
typedef struct _lidarConfig {
  /**
   * @brief Scanning enable.
   */
  int laser_en;

  /**
   * @brief rotate enable.
   */
  int motor_en;

  /**
   * @brief motor RPM.
   */
  int motor_rpm;

  /**
   * @brief start FOV angle.
   */
  int fov_start;

  /**
   * @brief end FOV angle.
   */
  int fov_end;

  /**
   * @brief data receive interface, USB or Ethernet.
   */
  int trans_sel;

  /**
   * @brief data receive IP.
   */
  char dataRecvIp[16];

  /**
   * @brief data receive PORT.
   */
  int dataRecvPort;

  /**
   * @brief device network config, HDCP or Manual.
   */
  int dhcp_en;

  /**
   * @brief device IP.
   */
  char deviceIp[16];

  /**
   * @brief device netmask.
   */
  char deviceNetmask[16];

  /**
   * @brief device gateway ip.
   */
  char deviceGatewayIp[16];

  int laserScanFrequency;


  /**
   * @brief correction_angle
   */
  int correction_angle;

  /**
   * @brief correction_distance
   */
  int correction_distance;

} lidarConfig;


#include "ydlidar_datatype.h"
