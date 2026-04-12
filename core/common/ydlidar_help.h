/*********************************************************************
* Software License Agreement (MIT License)
*
* Copyright © 2020 EAIBOT, Inc.
* Permission is hereby granted, free of charge, to any person obtaining
* a copy of this software and associated documentation
* files (the “Software”), to deal in the Software without restriction,
* including without limitation the rights to use, copy, modify, merge,
* publish, distribute, sublicense, and/or sell copies of the Software,
* and to permit persons to whom the Software is furnished to do so,
* subject to the following conditions:
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
*  @file     ydlidar_help.h                                                  *
*  @brief    LiDAR Help function                                             *
*  Details.                                                                  *
*                                                                            *
*  @author   Tony.Yang                                                       *
*  @email    chushuirurong618@eaibot.com                                     *
*  @version  1.0.0                                                           *
*  @date     2020/02/14                                                      *
*  @license  MIT                               *
*                                                                            *
*----------------------------------------------------------------------------*
*  Remark         : Description                                              *
*----------------------------------------------------------------------------*
*  Change History :                                                          *
*  <Date>     | <Version> | <Author>       | <Description>                   *
*----------------------------------------------------------------------------*
*  2020/02/14 | 1.0.0     | Tony           | Lidar Help File                 *
*----------------------------------------------------------------------------*
*                                                                            *
*********************************************************************/
#pragma once
#include <sstream>
#include <iomanip>
#include <stdarg.h>
#include "DriverInterface.h"

/**
 * @brief ydlidar
 */
namespace ydlidar {
/**
 * @brief ydlidar core
 */
namespace core {
using namespace base;
/**
 * @brief ydlidar common
 */
namespace common {

//颜色定义
#ifndef COLOR
    #define COLOFF  "\033[0m"      ///关闭所有属性
    #define RED      "\033[0;31m"   ///"\033[显示方式;字体颜色;背景颜色m"
    #define GREEN    "\033[0;32m"
    #define YELLOW   "\033[0;33m"
    #define BLUE     "\033[0;34m"
    #define PURPLE   "\033[0;35m"
#endif

//打印系统时间
#define UNIX_PRINT_TIME  \
  time_t currentTime = time(NULL); \
  struct tm *localTime = localtime(&currentTime); \
  printf("[%04d-%02d-%02d %02d:%02d:%02d]", \
    (1900 + localTime->tm_year), \
    (1 + localTime->tm_mon), \
    localTime->tm_mday, \
    localTime->tm_hour, \
    localTime->tm_min, \
    localTime->tm_sec);
#define WIN_PRINT_TIME UNIX_PRINT_TIME
#ifdef _WIN32
  #define PRINT_TIME WIN_PRINT_TIME
#else
  #define PRINT_TIME UNIX_PRINT_TIME
#endif
//格式化字符串
#define FORMAT_STDOUT \
  char buff[1024] = {0}; \
  va_list ap; \
  va_start(ap, fmt); \
  vsprintf(buff, fmt, ap); \
  va_end(ap); \
  printf(buff); \
  printf("\n");

//调试
inline void debug(char* fmt, ...)
{
  printf(GREEN); //设置绿色
  PRINT_TIME
  printf("[debug] ");
  FORMAT_STDOUT
  printf(COLOFF); //恢复默认颜色
  fflush(stdout);
}

//常规
inline void info(char* fmt, ...)
{
  PRINT_TIME
  printf("[info] ");
  FORMAT_STDOUT
  fflush(stdout);
}

//警告
inline void warn(char* fmt, ...)
{
  printf(YELLOW); //设置黄色
  PRINT_TIME
  printf("[warn] ");
  FORMAT_STDOUT
  printf(COLOFF); //恢复默认颜色
  fflush(stdout);
}

//错误
inline void error(char* fmt, ...)
{
  printf(RED); //设置红色
  PRINT_TIME
  printf("[error] ");
  FORMAT_STDOUT
  printf(COLOFF); //恢复默认颜色
  fflush(stdout);
}

//调试（16进制）
inline void debugh(const uint8_t *data, int size, const char* prefix=NULL)
{
  if (!data || !size)
    return;
  printf(GREEN); //设置绿色
  PRINT_TIME
  printf("[debug] ");
  if (prefix)
      printf(prefix);
  for (int i=0; i<size; ++i)
      printf("%02X", data[i]);
  printf("\n");
  printf(COLOFF); //恢复默认颜色
  fflush(stdout);
}

//常规（16进制）
inline void infoh(const uint8_t *data, int size, const char* prefix=NULL)
{
  if (!data || !size)
      return;
  PRINT_TIME
  printf("[info] ");
  if (prefix)
      printf(prefix);
  for (int i=0; i<size; ++i)
      printf("%02X", data[i]);
  printf("\n");
  fflush(stdout);
}

/*!
 * @brief convert lidar model to string
 * @param model lidar model
 * @return lidar model name
 */
inline std::string lidarModelToString(int model)
{
  std::string name = "unkown";

  switch (model)
  {
  case DriverInterface::YDLIDAR_F4:
    name = "F4";
    break;
  case DriverInterface::YDLIDAR_T1:
    name = "T1";
    break;
  case DriverInterface::YDLIDAR_F2:
    name = "F2";
    break;
  // case DriverInterface::YDLIDAR_S4:
  case DriverInterface::YDLIDAR_S2PRO:
    name = "S2PRO";
    break;
  case DriverInterface::YDLIDAR_G4:
    name = "G4";
    break;
  case DriverInterface::YDLIDAR_X4:
    name = "X4";
    break;
  case DriverInterface::YDLIDAR_G4PRO:
    name = "G4PRO";
    break;
  case DriverInterface::YDLIDAR_F4PRO:
    name = "F4PRO";
    break;
  case DriverInterface::YDLIDAR_R2:
    name = "R2";
    break;
  case DriverInterface::YDLIDAR_G10:
    name = "G10";
    break;
  case DriverInterface::YDLIDAR_S4B:
    name = "S4B";
    break;
  case DriverInterface::YDLIDAR_S2:
    name = "S2";
    break;
  case DriverInterface::YDLIDAR_G6:
    name = "G6";
    break;
  case DriverInterface::YDLIDAR_G2A:
    name = "G2A";
    break;
  case DriverInterface::YDLIDAR_G2B:
    name = "G2B";
    break;
  case DriverInterface::YDLIDAR_G2C:
    name = "G2C";
    break;
  case DriverInterface::YDLIDAR_G4B:
    name = "G4B";
    break;
  case DriverInterface::YDLIDAR_G4C:
    name = "G4C";
    break;
  case DriverInterface::YDLIDAR_G1:
    name = "G1";
    break;
  case DriverInterface::YDLIDAR_G5:
    name = "G5";
    break;
  case DriverInterface::YDLIDAR_G7:
    name = "G7";
    break;
  case DriverInterface::YDLIDAR_SCL:
    name = "SCL";
    break;
  case DriverInterface::YDLIDAR_R3:
    name = "R3";
    break;
  case DriverInterface::YDLIDAR_GS1:
    name = "GS1";
    break;
  case DriverInterface::YDLIDAR_GS2:
    name = "GS2";
    break;
  case DriverInterface::YDLIDAR_GS5:
    name = "GS5";
    break;
  case DriverInterface::YDLIDAR_TG15:
    name = "TG15";
    break;
  case DriverInterface::YDLIDAR_TG30:
    name = "TG30";
    break;
  case DriverInterface::YDLIDAR_TG50:
    name = "TG50";
    break;
  case DriverInterface::YDLIDAR_TEA:
    name = "TEA";
    break;
  case DriverInterface::YDLIDAR_TSA:
    name = "TSA";
    break;
  case DriverInterface::YDLIDAR_TSAPro:
    name = "TSA Pro";
    break;
  case DriverInterface::YDLIDAR_Tmini:
    return "Tmini";
  case DriverInterface::YDLIDAR_TminiPro:
    return "Tmini Pro";
  case DriverInterface::YDLIDAR_TminiPlus:
    return "Tmini Plus";
  case DriverInterface::YDLIDAR_TminiPlusSH:
    return "Tmini Plus SH";
  case DriverInterface::YDLIDAR_T15:
    return "T15";
  case DriverInterface::YDLIDAR_SDM15:
    return "SDM15";
  case DriverInterface::YDLIDAR_SDM18:
    return "SDM18";
  default:
    name = "unkown(YD-" + std::to_string(model) + ")";
    break;
  }

  return name;
}

/*!
 * @brief Get LiDAR default sampling rate.
 * @param model lidar model.
 * @return lidar sampling rate.
 */
inline std::vector<int> getDefaultSampleRate(int model) 
{
  std::vector<int> srs;

  switch (model) 
  {
    case DriverInterface::YDLIDAR_F4:
    case DriverInterface::YDLIDAR_T1:
    case DriverInterface::YDLIDAR_F2:
      srs.push_back(4);
      break;
    // case DriverInterface::YDLIDAR_S4:
    case DriverInterface::YDLIDAR_S2PRO:
      srs.push_back(3);
      srs.push_back(4);
      break;
    case DriverInterface::YDLIDAR_G4:
      srs.push_back(9);
      break;
    case DriverInterface::YDLIDAR_X4:
      srs.push_back(5);
      break;
    case DriverInterface::YDLIDAR_G4PRO:
      srs.push_back(9);
      break;
    case DriverInterface::YDLIDAR_F4PRO:
      srs.push_back(4);
      break;
    case DriverInterface::YDLIDAR_R2:
      srs.push_back(5);
      break;
    case DriverInterface::YDLIDAR_G10:
      srs.push_back(10);
      break;
    case DriverInterface::YDLIDAR_S4B:
      srs.push_back(4);
      break;
    case DriverInterface::YDLIDAR_S2:
      srs.push_back(3);
      break;
    case DriverInterface::YDLIDAR_G6:
      srs.push_back(18);
      break;
    case DriverInterface::YDLIDAR_G2A:
    case DriverInterface::YDLIDAR_G2B:
      srs.push_back(5);
      break;
    case DriverInterface::YDLIDAR_G2C:
    case DriverInterface::YDLIDAR_G4B:
    case DriverInterface::YDLIDAR_G4C:
      srs.push_back(4);
      break;
    case DriverInterface::YDLIDAR_G1:
    case DriverInterface::YDLIDAR_G5:
      srs.push_back(9);
      break;
    case DriverInterface::YDLIDAR_G7:
      srs.push_back(18);
      break;
    case DriverInterface::YDLIDAR_TG15:
      srs.push_back(20);
      break;
    case DriverInterface::YDLIDAR_TG30:
      srs.push_back(10);
      srs.push_back(20);
      break;
    case DriverInterface::YDLIDAR_TG50:
      srs.push_back(20);
      break;
    case DriverInterface::YDLIDAR_TEA:
      srs.push_back(30);
      break;
    case DriverInterface::YDLIDAR_T15:
      srs.push_back(20);
      break;

    default:
      srs.push_back(4);
      break;
  }

  return srs;
}

/*!
 * @brief Query whether the LiDAR is Octave LiDAR.
 * @param model lidar model
 * @return true if the current lidar sampling rate is octave, otherwise false
 */
inline bool isOctaveLidar(int model)
{
  bool ret = false;
  if (model == DriverInterface::YDLIDAR_G6 ||
      model == DriverInterface::YDLIDAR_G7)
  {
      ret = true;
  }
  return ret;
}

//根据雷达码判断是否是Tmini系列雷达
inline bool isTminiLidar(int model)
{
  return (model == DriverInterface::YDLIDAR_Tmini ||
          model == DriverInterface::YDLIDAR_TminiPro ||
          model == DriverInterface::YDLIDAR_TminiPlus ||
          model == DriverInterface::YDLIDAR_TSAPro);
}

//根据雷达码判断是否是SCL雷达
inline bool isSCLLidar2(int model)
{
  return model == DriverInterface::YDLIDAR_SCL;
}

//根据雷达码判断是否是TEA雷达
inline bool isTEALidar(int model)
{
  return model == DriverInterface::YDLIDAR_TEA;
}

/*!
 * @brief Supports multiple sampling rate
 * @param model   lidar model
 * @return true if THere are multiple sampling rate, otherwise false.
 */
inline bool hasSampleRate(int model) 
{
  bool ret = false;

  if (model == DriverInterface::YDLIDAR_G4 ||
      model == DriverInterface::YDLIDAR_G5 ||
      model == DriverInterface::YDLIDAR_G4PRO ||
      model == DriverInterface::YDLIDAR_F4PRO ||
      // model == DriverInterface::YDLIDAR_G6 ||
      model == DriverInterface::YDLIDAR_G7 ||
      model == DriverInterface::YDLIDAR_TG15 ||
      model == DriverInterface::YDLIDAR_TG50 ||
      model == DriverInterface::YDLIDAR_TG30 ||
      model == DriverInterface::YDLIDAR_TEA) {
    ret = true;
  }

  return ret;
}

inline bool isR3Lidar(int model)
{
  if (model == DriverInterface::YDLIDAR_R3)
  {
      return true;
  }
  return false;
}

/*!
 * @brief Is there a zero offset angle
 * @param model   lidar model
 * @return true if there are zero offset angle, otherwise false.
 */
inline bool hasZeroAngle(int model) {
  bool ret = false;

  if (model == DriverInterface::YDLIDAR_R2 ||
      model == DriverInterface::YDLIDAR_G2A ||
      model == DriverInterface::YDLIDAR_G2B ||
      model == DriverInterface::YDLIDAR_G2C ||
      model == DriverInterface::YDLIDAR_G1 ||
      model == DriverInterface::YDLIDAR_TG15 ||
      model == DriverInterface::YDLIDAR_TG30 ||
      model == DriverInterface::YDLIDAR_TG50 ||
      model == DriverInterface::YDLIDAR_TEA) {
    ret = true;
  }

  return ret;
}

/*!
 * @brief Whether to support adjusting the scanning frequency .
 * @param model   lidar model
 * @return true if supported, otherwise false.
 */
inline bool hasScanFrequencyCtrl(int model) 
{
  bool ret = true;

  if (model == DriverInterface::YDLIDAR_S4 ||
      model == DriverInterface::YDLIDAR_S4B ||
      model == DriverInterface::YDLIDAR_S2 ||
      model == DriverInterface::YDLIDAR_X4 ||
      model == DriverInterface::YDLIDAR_GS1 ||
      model == DriverInterface::YDLIDAR_GS2 ||
      model == DriverInterface::YDLIDAR_GS5) 
  {
    ret = false;
  }

  return ret;
}

/*!
 * @brief Does SDK support the LiDAR model.
 * @param model   lidar model
 * @return true if supported, otherwise false.
 */
inline bool isSupportLidar(int model)
{
  if (model > DriverInterface::YDLIDAR_None &&
      model < DriverInterface::YDLIDAR_Tail)
    return true;
  
  return false;
}

/*!
 * @brief Whether to support intensity.
 * @param model   lidar model
 * @return true if supported, otherwise false.
 */
inline bool hasIntensity(int model) 
{
  bool ret = false;

  if (model == DriverInterface::YDLIDAR_G2B ||
      model == DriverInterface::YDLIDAR_G4B ||
      model == DriverInterface::YDLIDAR_S4B ||
      model == DriverInterface::YDLIDAR_GS1 ||
      model == DriverInterface::YDLIDAR_GS2) {
    ret = true;
  }

  return ret;
}

/*!
 * @brief Whether to support serial DTR enable motor.
 * @param model   lidar model
 * @return true if support serial DTR enable motor, otherwise false.
 */
inline bool isSupportMotorCtrl(int model) {
  bool ret = false;

  if (model == DriverInterface::YDLIDAR_X4 ||
      model == DriverInterface::YDLIDAR_S2 ||
      model == DriverInterface::YDLIDAR_S4 ||
      model == DriverInterface::YDLIDAR_S4B) {
    ret = true;

  }

  return true;
}

/*!
 * @brief Whether the scanning frequency is supported
 * @param model     lidar model
 * @param frequency scanning frequency
 * @return true if supported, otherwise false.
 */
inline bool isSupportScanFrequency(int model, double frequency)
{
  bool ret = false;

  if (model >= DriverInterface::YDLIDAR_TG15)
  {
    if (1 <= frequency && frequency <= 18)
    {
      ret = true;
    }

    if (model == DriverInterface::YDLIDAR_SDM15)
    {
      if (10 <= frequency && frequency <= 1800)
        ret = true;
    }
    else if (model >= DriverInterface::YDLIDAR_T15)
    {
      if (1 <= frequency && frequency <= 50)
      {
        ret = true;
      }
    }
    else if (model == DriverInterface::YDLIDAR_TEA)
    {
      if (10 <= frequency && frequency <= 30)
      {
        ret = true;
      }
    }
    else if (model == DriverInterface::YDLIDAR_Tmini)
    {
      if (5 <= frequency && frequency <= 12)
      {
        ret = true;
      }
    }
  }
  else
  {
    if (5 <= frequency && frequency <= 16)
    {
      ret = true;
    }
  }

  return ret;
}

/**
 * @brief Whether it is a TOF Model LiDAR
 * @param model  LiDAR model
 * @return tru if it is TOF Model, otherwise false.
 */
inline bool isTOFLidarByModel(int model) {
  bool ret = false;

  if (model >= DriverInterface::YDLIDAR_TG15 &&
      model <= DriverInterface::YDLIDAR_TG50) {
    ret = true;
  }

  return ret;
}

/**
 * @brief Whether it is a Net TOF Model LiDAR
 * @param model  LiDAR model
 * @return tru if it is Net TOF Model, otherwise false.
 */
inline bool isNetTOFLidarByModel(int model) {
  bool ret = false;

  if (model >= DriverInterface::YDLIDAR_T15) {
    ret = true;
  }

  return ret;
}

/**
 * @brief Whether it is a TOF type LiDAR
 * @param type  LiDAR type
 * @return true if it is a TOF type, otherwise false.
 */
inline bool isTOFLidar(int type) {
  bool ret = false;

  if (type == TYPE_TOF) {
    ret = true;
  }

  return ret;
}

/**
 * @brief Whether it is a network hardware interface TOF type LiDAR
 * @param type  LiDAR type
 * @return true if it is a network hardware interface TOF type, otherwise false.
 */
inline bool isNetTOFLidar(int type) {
  bool ret = false;

  if (type == TYPE_TOF_NET) {
    ret = true;
  }

  return ret;
}

/**
 * @brief Whether it is a Triangle type LiDAR
 * @param type  LiDAR type
 * @return true if it is a Triangle type, otherwise false.
 */
inline bool isTriangleLidar(int type) {
  bool ret = false;

  if (type == TYPE_TRIANGLE ||
    type == TYPE_SCL) {
    ret = true;
  }

  return ret;
}

/**
 * @brief Whether it is a GS type LiDAR
 * @param type  LiDAR type
 * @return true if it is a Triangle type, otherwise false.
 */
inline bool isGSLidar(int type) 
{
  return (type == TYPE_GS);
}

/**
 * @brief Whether it is a SCL type LiDAR
 * @param type  LiDAR type
 * @return true if it is a Triangle SCL type, otherwise false.
 */
inline bool isSCLLidar(int type) 
{
  return (type == TYPE_SCL);
}

inline bool isSDMLidar(int type)
{
  return (type == TYPE_SDM);
}

inline bool isDTSLidar(int type)
{
  return (type == TYPE_SDM18);
}

inline bool isTIALidar(int type)
{
  return (type == TYPE_TIA);
}

/**
 * @brief Whether it is Old Version protocol TOF LiDAR
 * @param model     lidar model
 * @param Major     firmware Major version
 * @param Minor     firmware Minor version
 * @return true if it is old version protocol, otherwise false.
 */
inline bool isOldVersionTOFLidar(int model, int Major, int Minor) {
  bool ret = false;

  if (model == DriverInterface::YDLIDAR_TG15 ||
      model == DriverInterface::YDLIDAR_TG30 ||
      model == DriverInterface::YDLIDAR_TG50)  {
    if (Major <= 1 && Minor <= 2) {
      ret = true;
    }

  }

  return ret;
}

inline float lidarZeroOffsetAngleScale(uint8_t model, uint8_t Major,
                                       uint8_t Minor) {
  float scale = 4.f;

  if (model == DriverInterface::YDLIDAR_R2) {
    scale = 100.f;

    if ((Major == 1 && Minor <= 7) || Major < 1) {
      scale = 4.0;
    }
  }

  return scale;
}

/*!
 * @brief Whether to support Heartbeat.
 * @param model   lidar model
 * @return true if support heartbeat, otherwise false.
 */
inline bool isSupportHeartBeat(int model) {
  bool ret = false;

  if (model == DriverInterface::YDLIDAR_G4 ||
      model == DriverInterface::YDLIDAR_G4PRO) {
    ret = true;
  }

  return true;
}

/**
 * @brief Whether the sampling rate is valid
 * @param smap  sampling rate map
 * @return true if it is valid, otherwise false.
 */
inline bool isValidSampleRate(std::map<int, int> smap) 
{
  if (smap.size() < 1) {
    return false;
  }

  if (smap.size() == 1) {
    if (smap.begin()->second > 2) {
      return true;
    }

    return false;
  }

  return false;
}

/**
 * @brief convert User sampling rate code to LiDAR sampling code
 * @param model         LiDAR model
 * @param m_SampleRate  User sampling rate code
 * @param defaultRate   LiDAR Defualt sampling rate code
 * @return  LiDAR sampling rate code
 */
inline int ConvertUserToLidarSmaple(int model,
                                    int sampleRate,
                                    int defaultRate)
{
  int _samp_rate = 9;
  switch (sampleRate) 
  {
    case 10:
      _samp_rate = DriverInterface::YDLIDAR_RATE_4K;
      break;
    case 16:
      _samp_rate = DriverInterface::YDLIDAR_RATE_8K;
      break;
    case 18:
      _samp_rate = DriverInterface::YDLIDAR_RATE_9K;
      break;
    case 20:
      _samp_rate = DriverInterface::YDLIDAR_RATE_10K;
      break;
    default:
      _samp_rate = defaultRate;
      break;
  }

  if (!isOctaveLidar(model)) 
  {
    _samp_rate = 2;
    switch (sampleRate) 
    {
      case 4:
        _samp_rate = DriverInterface::YDLIDAR_RATE_4K;
        break;
      case 8:
        _samp_rate = DriverInterface::YDLIDAR_RATE_8K;
        break;
      case 9:
        _samp_rate = DriverInterface::YDLIDAR_RATE_9K;
        break;
      default:
        break;
    }
    if (model == DriverInterface::YDLIDAR_F4PRO) 
    {
      _samp_rate = 0;
      switch (sampleRate) 
      {
        case 4:
          _samp_rate = DriverInterface::YDLIDAR_RATE_4K;
          break;
        case 6:
          _samp_rate = DriverInterface::YDLIDAR_RATE_8K;
          break;
        default:
          break;
      }
    }
  }

  return _samp_rate;
}

/**
 * @brief convert LiDAR sampling rate code to User sampling code
 * @param model     LiDAR model
 * @param rate      LiDAR sampling rate code
 * @return user sampling code
 */
inline int ConvertLidarToUserSmaple(int model, int rate) 
{
  int _samp_rate = 9;

  if (!isOctaveLidar(model) && 
      !isTOFLidarByModel(model))
  {
    switch (rate)
    {
    case DriverInterface::YDLIDAR_RATE_4K:
      _samp_rate = 4;
      break;
    case DriverInterface::YDLIDAR_RATE_8K:
      _samp_rate = 8;
      if (model == DriverInterface::YDLIDAR_F4PRO)
        _samp_rate = 6;
      break;
    case DriverInterface::YDLIDAR_RATE_9K:
      _samp_rate = 9;
      break;
    case DriverInterface::YDLIDAR_RATE_10K:
      _samp_rate = 10;
      break;
    default:
      //修改默认为当前获取到采样率值
      _samp_rate = rate;
      break;
    }
  }
  else
  {
    switch (rate)
    {
    case DriverInterface::YDLIDAR_RATE_4K:
      _samp_rate = 10;
      break;
    case DriverInterface::YDLIDAR_RATE_8K:
      _samp_rate = 16;
      break;
    case DriverInterface::YDLIDAR_RATE_9K:
      _samp_rate = 18;
      break;
    case DriverInterface::YDLIDAR_RATE_10K:
      _samp_rate = 20;
      break;
    case 4:
      _samp_rate = 10;
      break;
    default:
      //修改默认为当前获取到采样率值
      _samp_rate = rate;
      break;
    }
  }

  return _samp_rate;
}

/**
 * @brief Whether the Value is valid.
 * @param value LiDAR CT Byte information
 * @return true if it is valid, otherwise false
 */
inline bool isValidValue(uint8_t value) {
  if (value & 0x80) {
    return false;
  }

  return true;
}

/**
 * @brief Whether the Version is valid.
 * @param info  the LiDAR LaserDebug information
 * @return true if it is valid, otherwise false.
 */
inline bool isVersionValid(const LaserDebug &info) {
  bool ret = false;

  if (isValidValue(info.cVer) &&
      isValidValue(info.debug2) &&
      isValidValue(info.hfVer) &&
      isValidValue(info.month)) {
    ret = true;
  }

  return ret;
}

/**
 * @brief Whether the serial number is valid.
 * @param info LiDAR LaserDebug information
 * @return true if it is valid, otherwise false.
 */
inline bool isSerialNumbValid(const LaserDebug &info) {
  bool ret = false;

  if (isValidValue(info.day) &&
      isValidValue(info.year) &&
      isValidValue(info.numH) &&
      isValidValue(info.numH)) {
    ret = true;
  }

  return ret;
}

/**
 * @brief convert node_info to LaserDebug
 * @param node  LiDAR node_info information
 * @param info  LiDAR LaserDebug information
 */
inline void parsePackageNode(const node_info &node, LaserDebug &info) 
{
  switch (node.index) {
    case 0:
      break;
    case 1:
      info.cVer = node.debugInfo;
      break;
    case 2:
      info.debug2 = node.debugInfo;
      break;
    case 3:
      //健康信息
      info.health = node.debugInfo;
      break;
    case 4:
      info.hfVer = node.debugInfo;
      break;
    case 5:
      info.fVer = node.debugInfo;
      break;
    case 6:
      break;
    case 7:
      info.model = node.debugInfo;
      break;
    case 8:
      break;
    case 9:
      info.year = node.debugInfo;
      break;
    case 10:
      info.month = node.debugInfo;
      break;
    case 11:
      info.day = node.debugInfo;
      break;
    case 12:
      info.numH = node.debugInfo;
      break;
    case 13:
      info.numL = node.debugInfo;
      break;
    default:
      break;
  }

  // if (info.MaxDebugIndex > node.index) {
  //   info.W3F4CusMajor_W4F0CusMinor = 0xff;
  // }

  if (node.index > info.maxIndex) {
    info.maxIndex = node.index;
  }
}

// #include <iostream>
// #include <iomanip>
/**
 * @brief convert LaserDebug information to device_info
 * @param info      LiDAR LaserDebug information
 * @param value     LiDAR Device information
 * @return true if converted successfully, otherwise false.
 */
inline bool parseLaserDebugInfo(const LaserDebug &debug, device_info &di)
{
  bool ret = false;

  uint8_t model = uint8_t(debug.model & 0x0F);
  uint8_t CustomVerMajor = uint8_t(debug.hfVer & 0x0F);
  uint8_t CustomVerMinor = debug.fVer;
  // uint8_t lidarmodel = uint8_t(debug.debug2) >> 3;
  uint8_t hardwareVer = uint8_t(debug.hfVer) >> 4;

  uint8_t Year = uint8_t(debug.year >> 2);
  uint8_t Moth = uint8_t(debug.month >> 3);
  uint8_t Date = uint8_t(debug.day >> 2);
  uint32_t Number = 
    uint32_t(debug.year & 0x03) << 19 |
    uint32_t(debug.month & 0x07) << 16 |
    uint32_t(debug.day & 0x03) << 14 |
    uint32_t(debug.numH & 0x7F) << 7 |
    uint32_t(debug.numL & 0x7F);

  if (Moth && Date && Number)
  {
    di.firmware_version = uint16_t(CustomVerMajor << 8) |
                          uint16_t(CustomVerMinor);
    di.hardware_version = hardwareVer;
    di.model = model;
    std::stringstream ss;
    ss << std::setw(4) << std::setfill('0') << int(Year + 2020);
    ss << std::setw(2) << std::setfill('0') << int(Moth);
    ss << std::setw(2) << std::setfill('0') << int(Date);
    ss << std::setw(8) << std::setfill('0') << Number;
    std::string sn(ss.str());
    // 此处sprintf函数在Python调用中会导致缓存溢出
    //  sprintf(reinterpret_cast<char*>(di.serialnum),
    //    "%04u%02u%02u%08u", Year + 2020, Moth, Date, Number);
    for (int i = 0; i < SDK_SNLEN && i < sn.size(); i++)
    {
      di.serialnum[i] = std::stoi(std::string(1, sn.at(i)));
    }

    ret = true;
  }

  return ret;
}

YDLIDAR_API inline bool printfDeviceInfo(const device_info &di,
                              int platformType=EPT_Module)
{
  if (di.firmware_version == 0 &&
      di.hardware_version == 0) {
    return false;
  }

  uint8_t Major = (uint8_t)(di.firmware_version >> 8);
  uint8_t Minjor = (uint8_t)(di.firmware_version & 0xff);
  std::string sn;
    for (int i = 0; i < SDK_SNLEN; i++)
      sn += char(di.serialnum[i] + 48); //整型值转字符值
    // printf("%01X", di.serialnum[i] & 0xff);
  
  info("%s device info\n"
         "Firmware version: %u.%u\n"
         "Hardware version: %u\n"
         "Model: %s\n"
         "Serial: %s",
         EPT_Module == platformType ? "Module" : "Baseplate",
         Major,
         Minjor,
         di.hardware_version,
         lidarModelToString(di.model).c_str(),
         sn.c_str());

  return true;
}

/**
 * @brief split string to vector by delim format
 * @param s       string
 * @param delim   split format
 * @return split vector
 */
inline std::vector<float> split(const std::string &s, char delim) {
  std::vector<float> elems;
  std::stringstream ss(s);
  std::string number;

  while (std::getline(ss, number, delim)) {
    elems.push_back(atof(number.c_str()));
  }

  return elems;
}

/**
 * @brief Whether the ET LiDAR Protocol type is V1.
 * @param protocol LiDAR Protocol Byte information
 * @return true if it is V1, otherwise false
 */
inline bool isV1Protocol(uint8_t protocol)
{
  if (protocol == Protocol_V1) {
    return true;
  }
  return false;
}

//获取数据值（小端序）
inline uint32_t getLittleValue(const uint8_t *data, int size)
{
  uint32_t v = 0;
  if (!data || !size)
    return v;
  for (int i=0; i<size; ++i)
    v += uint32_t(data[i] << (i * 8));
  return v;
}

//获取数据值（大端序）
inline uint32_t getBigValue(const uint8_t *data, int size)
{
  uint32_t v = 0;
  if (!data || !size)
    return v;
  for (int i=0; i<size; ++i)
    v += uint32_t(data[i] << ((size - 1 - i) * 8));
  return v;
}

}//common
}//core
}//ydlidar
