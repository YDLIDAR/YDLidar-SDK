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

#ifndef M_PI
#define M_PI 3.1415926
#endif

//浮点型判断是否为0
#define ISZERO(v) (abs(v) < 1e-12)

#define SUNNOISEINTENSITY 0x03 //sun noise flag constant
#define GLASSNOISEINTENSITY 0x02 //glass noise flag constant

//LIDAR CMD Protocol
#define LIDAR_CMD_STOP                      0x65
#define LIDAR_CMD_SCAN                      0x60
#define LIDAR_CMD_FORCE_SCAN                0x61
#define LIDAR_CMD_RESET                     0x80
#define LIDAR_CMD_FORCE_STOP                0x00
#define LIDAR_CMD_GET_EAI                   0x55
#define LIDAR_CMD_GET_DEVICE_INFO           0x90
#define LIDAR_CMD_GET_DEVICE_HEALTH         0x92
#define LIDAR_CMD_SYNC_BYTE                 0xA5
#define LIDAR_CMDFLAG_HAS_PAYLOAD           0x80
#define LIDAR_RESP_SYNCBIT        (0x1<<0)
#define LIDAR_RESP_QUALITY_SHIFT  2
#define LIDAR_RESP_CHECKBIT       (0x1<<0)
#define LIDAR_RESP_ANGLE_SHIFT    1
#define LIDAR_RESP_DIST_SHIFT  2
#define LIDAR_RESP_ANGLE_SAMPLE_SHIFT 8

#define LIDAR_ANS_SYNC_BYTE1                0xA5
#define LIDAR_ANS_SYNC_BYTE2                0x5A
#define LIDAR_ANS_TYPE_DEVINFO              0x04
#define LIDAR_ANS_TYPE_DEVHEALTH            0x06
#define LIDAR_ANS_TYPE_PITCH                0x10
#define LIDAR_ANS_TYPE_MEASUREMENT          0x81

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
#define LIDAR_CMD_GETPITCH 0x98 //获取俯仰角

//GS命令
#define GS_LIDAR_CMD_GET_ADDRESS               0x60
#define GS_LIDAR_CMD_GET_PARAMETER             0x61
#define GS_LIDAR_CMD_GET_VERSION               0x62
#define GS_LIDAR_CMD_GET_VERSION3              0x6B
#define GS_LIDAR_CMD_SCAN                      0x63
#define GS_LIDAR_ANS_SCAN                      0x63
#define GS_LIDAR_CMD_STOP                      0x64
#define GS_LIDAR_CMD_RESET                     0x67
#define GS_LIDAR_CMD_SET_MODE                  0x69
#define GS_LIDAR_CMD_SET_BIAS                  0xD9
#define GS_LIDAR_CMD_SET_DEBUG_MODE            0xF0
//GS
#define Angle_Px 1.22
#define Angle_Py 5.315
#define Angle_PAngle 22.5 //GS2
#define Angle_PAngle2 16.0 //GS5
#define GS_PACKHEADSIZE 8
#define GS_PACKMAXNODES 160 //GS数据包中最大点云数

#define SDK_SNLEN 16 //序列号长度

/// Default Node Quality
#define Node_Default_Quality (10)
/// Starting Node
#define NODE_SYNC 1
/// Normal Node
#define NODE_UNSYNC 2
/// Package Header
#define PH 0x55AA
#define PH1 0xAA
#define PH2 0x55 //AA55是点云数据
#define PH3 0x66 //AA66是时间戳数据

//Package
#define TRI_PACKHEADSIZE 10
#define TRI_PACKMAXNODES 80 //单包最大点数（G4最大点数为80，其它三角等最大均为40）
#define TIA_PACKWIDTH 12 //TIA单包行数
#define TIA_PACKHEIGHT 16 //TIA单包行数
#define TIA_PACKMAXBUFFS (TIA_PACKWIDTH * (4 + TIA_PACKHEIGHT * 4) + 4 + 4) //TIA-H单包最大字节数
#define TIA_PACKMAXBUFFS2 (TIA_PACKMAXBUFFS + 4) //TIA单包最大字节数
#define TIA_PACKMAXNODES (TIA_PACKWIDTH * TIA_PACKHEIGHT) //TIA单包最大点数

//模组地址
#define LIDAR_MODULE_1 0x01
#define LIDAR_MODULE_2 0x02
#define LIDAR_MODULE_3 0x04
#define LIDAR_MODULE_ALL 0x00
#define LIDAR_MAXCOUNT 3 //最大模组数
#define LIDAR_PACKMAXNODES TRI_PACKMAXNODES //单包最大点数
#define LIDAR_MAXNODES 5000 //最大点数

#define FREINDEX 0
#define USERVERSIONNDEX 1
#define HEALTHINDEX 3

//超时定义
#define TIMEOUT_100 100
#define TIMEOUT_300 300
#define TIMEOUT_500 500 //500ms
#define TIMEOUT_1S 1000
#define TIMEOUT_2S 2000
#define SDK_TIMEOUT TIMEOUT_1S //默认超时时间
//角度定义
#define SDK_ANGLE360 360.0f
#define SDK_ANGLE180 180.0f
#define SDK_ANGLE90 90.0f
#define SDK_ANGLE0 0.0f

/// CT Package Type
typedef enum {
  CT_Normal = 0,///< Normal package
  CT_RingStart  = 1,///< Starting package
  CT_Tail,
} CT;

//雷达协议类型
typedef enum {
  Protocol_V1 = 0, //V1 version
  Protocol_V2 = 1, //V2 version
} ProtocolVer;

//设备所属平台类型
enum EaiPlatformType
{
  EPT_None = 0x00, //无
  EPT_Module = 0x01, //模组
  EPT_Base = 0x02, //底板
  EPT_All = (EPT_Module | EPT_Base), //所有
};

#if defined(_WIN32)
#pragma pack(1)
#endif

//雷达节点信息
struct node_info {
  uint8_t sync; //首包标记
  uint8_t is; //抗干扰标志
  uint16_t qual; //信号强度
  uint16_t angle; //角度值（°）
  uint16_t dist; //距离值
  uint64_t stamp; //时间戳
  uint32_t delayTime; //delay time
  uint8_t scanFreq; //扫描频率
  uint8_t debugInfo; //debug information
  uint8_t index; //包序号
  uint8_t error; //error package state
} __attribute__((packed));
#define SDKNODESIZE sizeof(node_info)

//package node info
struct tri_node {
  uint16_t dist; //range
} __attribute__((packed));

//package node info
struct tri_node2 {
  uint8_t qual;///< intensity
  uint16_t dist;///< range
} __attribute__((packed));

// TOF Intensity package node info
struct tof_node {
  uint16_t qual;
  uint16_t dist;
} __attribute__((packed));

//LiDAR Normal Nodes package
struct tri_node_package {
  uint16_t  head;///< package header
  uint8_t   ct;///< package ct
  uint8_t   count; ///< package number
  uint16_t  firstAngle;///< first sample angle
  uint16_t  lastAngle;///< last sample angle
  uint16_t  cs;///< checksum
  uint16_t  nodes[TRI_PACKMAXNODES];
} __attribute__((packed));

//LiDAR Intensity Nodes Package
struct tri_node_package2 {
  uint16_t  head;///< package header
  uint8_t   ct;///< package ct
  uint8_t   count;///< package number
  uint16_t  firstAngle;///< first sample angle
  uint16_t  lastAngle;///< last sample angle
  uint16_t  cs;///< checksum
  tri_node2  nodes[TRI_PACKMAXNODES];
} __attribute__((packed));

// TOF LiDAR Intensity Nodes Package
struct tof_node_package {
  uint16_t  head;
  uint8_t   ct;
  uint8_t   count;
  uint16_t  firstAngle;
  uint16_t  lastAngle;
  uint16_t  cs;
  tof_node  nodes[TRI_PACKMAXNODES];
} __attribute__((packed));

//时间戳结构体
struct stamp_package {
  uint8_t flag1; //包头标记1
  uint8_t flag2; //包头标记2
  uint8_t cs; //校验和
  uint32_t stamp; //时间戳
  uint8_t reserved; //保留字段
} __attribute__((packed));
#define SIZE_STAMPPACKAGE sizeof(stamp_package)

//设备信息结构体
struct device_info {
  uint8_t   model; //雷达型号码
  uint16_t  firmware_version; //固件版本
  uint8_t   hardware_version; //硬件版本
  uint8_t   serialnum[SDK_SNLEN]; //序列号
} __attribute__((packed));
#define DEVICEINFOSIZE sizeof(device_info)

//设备信息结构体（带模组序号）
struct device_info_ex {
  uint8_t id = 0;
  device_info di = {0};
};

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
#define TRIRESPHEADSIZE sizeof(lidar_ans_header) //定义通用响应头大小

//GS单帧数据
struct gs_packages {
    int frameNum;
    int moduleNum;
    bool left = false;
    bool right = false;
    node_info points[GS_PACKMAXNODES];
} __attribute__((packed));
struct gs_module_nodes {
  int moduleNum = 0;
  int pointCount = 0;
  node_info points[GS_PACKMAXNODES];
} __attribute__((packed));

//GS点数据结构
struct gs_node {
  // uint16_t dist : 9;
  // uint16_t qual : 7;
  uint16_t node;
} __attribute__((packed));
#define GSNODESIZE sizeof(gs_node) //定义GS点大小

//GS单包数据结构
struct gs_node_package {
  uint32_t head;
  uint8_t address;
  uint8_t ct;
  uint16_t size;
  uint16_t env;
  gs_node nodes[GS_PACKMAXNODES];
  uint8_t cs;
} __attribute__((packed));
#define GSPACKSIZE sizeof(gs_node_package) //定义GS点大小

//GS设备参数
struct gs_device_para {
    uint16_t k0;
    uint16_t b0;
    uint16_t k1;
    uint16_t b1;
    int8_t bias;
    uint8_t crc;
} __attribute__((packed));
//GS包头
struct gs_package_head {
    uint8_t syncByte0;
    uint8_t syncByte1;
    uint8_t syncByte2;
    uint8_t syncByte3;
    uint8_t address;
    uint8_t type;
    uint16_t size;
} __attribute__((packed));
#define GSPACKEGEHEADSIZE sizeof(gs_package_head)
//GS系列设备信息
struct gs_device_info {
    uint8_t hwVersion; //硬件版本号
    uint16_t fwVersion; //固件版本号
    uint8_t sn[16]; //序列号
} __attribute__((packed));
#define GSDEVINFOSIZE sizeof(gs_device_info)
//GS系列设备信息（带雷达型号）
struct gs_device_info2 {
    uint8_t hwVersion; //硬件版本号
    uint16_t fwVersion; //固件版本号
    uint8_t model; //型号
    uint8_t sn[16]; //序列号
} __attribute__((packed));
#define GSDEVINFO2SIZE sizeof(gs_device_info2)

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
