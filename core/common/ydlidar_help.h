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
#include "DriverInterface.h"
#include <sstream>

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


/*!
 * @brief convert lidar model to string
 * @param model lidar model
 * @return lidar model name
 */
inline std::string lidarModelToString(int model) {
  std::string name = "unkown";

  switch (model) {
    case DriverInterface::YDLIDAR_F4:
      name = "F4";
      break;

    case DriverInterface::YDLIDAR_T1:
      name = "T1";

      break;

    case DriverInterface::YDLIDAR_F2:
      name = "F2";

      break;

    case DriverInterface::YDLIDAR_S4:
      name = "S4";

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

    case DriverInterface::YDLIDAR_TG15:
      name = "TG15";

      break;

    case DriverInterface::YDLIDAR_TG30:
      name = "TG30";

      break;

    case DriverInterface::YDLIDAR_TG50:
      name = "TG50";
      break;

    case DriverInterface::YDLIDAR_T15:
      name = "T15";
      break;

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
inline int lidarModelDefaultSampleRate(int model) {
  int sample_rate = 4;

  switch (model) {
    case DriverInterface::YDLIDAR_F4:
      break;

    case DriverInterface::YDLIDAR_T1:
      break;

    case DriverInterface::YDLIDAR_F2:
      break;

    case DriverInterface::YDLIDAR_S4:
      break;

    case DriverInterface::YDLIDAR_G4:
      sample_rate = 9;
      break;

    case DriverInterface::YDLIDAR_X4:
      sample_rate = 5;
      break;

    case DriverInterface::YDLIDAR_G4PRO:
      sample_rate = 9;
      break;

    case DriverInterface::YDLIDAR_F4PRO:
      sample_rate = 4;
      break;

    case DriverInterface::YDLIDAR_R2:
      sample_rate = 5;
      break;

    case DriverInterface::YDLIDAR_G10:
      sample_rate = 10;
      break;

    case DriverInterface::YDLIDAR_S4B:
      sample_rate = 4;
      break;

    case DriverInterface::YDLIDAR_S2:
      sample_rate = 3;
      break;

    case DriverInterface::YDLIDAR_G6:
      sample_rate = 18;
      break;

    case DriverInterface::YDLIDAR_G2A:
      sample_rate = 5;
      break;

    case DriverInterface::YDLIDAR_G2B:
      sample_rate = 5;
      break;

    case DriverInterface::YDLIDAR_G2C:
      sample_rate = 4;
      break;

    case DriverInterface::YDLIDAR_G4B:
      break;

    case DriverInterface::YDLIDAR_G4C:
      break;

    case DriverInterface::YDLIDAR_G1:
      sample_rate = 9;
      break;

    case DriverInterface::YDLIDAR_G5:
      sample_rate = 9;
      break;

    case DriverInterface::YDLIDAR_G7:
      sample_rate = 18;
      break;

    case DriverInterface::YDLIDAR_TG15:
      sample_rate = 20;
      break;

    case DriverInterface::YDLIDAR_TG30:
      sample_rate = 20;
      break;

    case DriverInterface::YDLIDAR_TG50:
      sample_rate = 20;
      break;

    case DriverInterface::YDLIDAR_T15:
      sample_rate = 20;
      break;

    default:
      break;
  }

  return sample_rate ;
}

/*!
 * @brief Query whether the LiDAR is Octave LiDAR.
 * @param model lidar model
 * @return true if the current lidar sampling rate is octave, otherwise false
 */
inline bool isOctaveLidar(int model) {
  bool ret = false;

  if (model == DriverInterface::YDLIDAR_G6 ||
      model == DriverInterface::YDLIDAR_G7) {
    ret = true;
  }

  return ret;
}

/*!
 * @brief Supports multiple sampling rate
 * @param model   lidar model
 * @return true if THere are multiple sampling rate, otherwise false.
 */
inline bool hasSampleRate(int model) {
  bool ret = false;

  if (model == DriverInterface::YDLIDAR_G4 ||
      model == DriverInterface::YDLIDAR_G5 ||
      model == DriverInterface::YDLIDAR_G4PRO ||
      model == DriverInterface::YDLIDAR_F4PRO ||
      model == DriverInterface::YDLIDAR_G6 ||
      model == DriverInterface::YDLIDAR_G7 ||
      model == DriverInterface::YDLIDAR_TG15 ||
      model == DriverInterface::YDLIDAR_TG50 ||
      model == DriverInterface::YDLIDAR_TG30) {
    ret = true;
  }

  return ret;
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
      model == DriverInterface::YDLIDAR_TG50) {
    ret = true;
  }

  return ret;
}

/*!
 * @brief Whether to support adjusting the scanning frequency .
 * @param model   lidar model
 * @return true if supported, otherwise false.
 */
inline bool hasScanFrequencyCtrl(int model) {
  bool ret = true;

  if (model == DriverInterface::YDLIDAR_S4 ||
      model == DriverInterface::YDLIDAR_S4B ||
      model == DriverInterface::YDLIDAR_S2 ||
      model == DriverInterface::YDLIDAR_X4) {
    ret = false;
  }

  return ret;
}

/*!
 * @brief Does SDK support the LiDAR model.
 * @param model   lidar model
 * @return true if supported, otherwise false.
 */
inline bool isSupportLidar(int model) {
  bool ret = true;

  if (model < DriverInterface::YDLIDAR_F4 ||
      (model > DriverInterface::YDLIDAR_G7 &&
       model < DriverInterface::YDLIDAR_TG15) ||
      (model > DriverInterface::YDLIDAR_TG50 &&
       model < DriverInterface::YDLIDAR_T15)) {
    ret = false;

  }

  return ret;
}

/*!
 * @brief Whether to support intensity.
 * @param model   lidar model
 * @return true if supported, otherwise false.
 */
inline bool hasIntensity(int model) {
  bool ret = false;

  if (model == DriverInterface::YDLIDAR_G2B ||
      model == DriverInterface::YDLIDAR_G4B ||
      model == DriverInterface::YDLIDAR_S4B) {
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
inline bool isSupportScanFrequency(int model, double frequency) {
  bool ret = false;

  if (model >= DriverInterface::YDLIDAR_TG15) {
    if (1 <= frequency && frequency <= 18) {
      ret = true;
    }

    if (model >= DriverInterface::YDLIDAR_T15) {
      if (1 <= frequency && frequency <= 50) {
        ret = true;
      }
    }
  } else {
    if (5 <= frequency && frequency <= 16) {
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

  if (type == TYPE_TRIANGLE) {
    ret = true;
  }

  return ret;
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
inline bool isValidSampleRate(std::map<int, int>  smap) {
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
inline int ConvertUserToLidarSmaple(int model, int m_SampleRate,
                                    int defaultRate) {
  int _samp_rate = 9;

  switch (m_SampleRate) {
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

  if (!isOctaveLidar(model)) {
    _samp_rate = 2;

    switch (m_SampleRate) {
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

    if (model == DriverInterface::YDLIDAR_F4PRO) {
      _samp_rate = 0;

      switch (m_SampleRate) {
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
inline int ConvertLidarToUserSmaple(int model, int rate) {
  int _samp_rate = 9;

  switch (rate) {
    case DriverInterface::YDLIDAR_RATE_4K:
      _samp_rate = 10;

      if (!isOctaveLidar(model)) {
        _samp_rate = 4;
      }

      break;

    case DriverInterface::YDLIDAR_RATE_8K:
      _samp_rate = 16;

      if (!isOctaveLidar(model)) {
        _samp_rate = 8;

        if (model == DriverInterface::YDLIDAR_F4PRO) {
          _samp_rate = 6;
        }
      }

      break;

    case DriverInterface::YDLIDAR_RATE_9K:
      _samp_rate = 18;

      if (!isOctaveLidar(model)) {
        _samp_rate = 9;
      }

      break;

    case DriverInterface::YDLIDAR_RATE_10K:
      _samp_rate = 20;

      if (!isOctaveLidar(model)) {
        _samp_rate = 10;
      }

      break;

    default:
      _samp_rate = 9;

      if (isOctaveLidar(model)) {
        _samp_rate = 18;
      }

      break;
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

  if (isValidValue(info.W3F4CusMajor_W4F0CusMinor) &&
      isValidValue(info.W4F3Model_W3F0DebugInfTranVer) &&
      isValidValue(info.W3F4HardwareVer_W4F0FirewareMajor) &&
      isValidValue(info.W3F4BoradHardVer_W4F0Moth)) {
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

  if (isValidValue(info.W2F5Output2K4K5K_W5F0Date) &&
      isValidValue(info.W1F6GNoise_W1F5SNoise_W1F4MotorCtl_W4F0SnYear) &&
      isValidValue(info.W7F0SnNumH) &&
      isValidValue(info.W7F0SnNumH)) {
    ret = true;
  }

  return ret;
}

/**
 * @brief convert node_info to LaserDebug
 * @param node  LiDAR node_info information
 * @param info  LiDAR LaserDebug information
 */
inline void parsePackageNode(const node_info &node, LaserDebug &info) {
  switch (node.index) {
    case 0://scan frequency

      break;

    case 1://W3F3CusHardVer_W4F0CusSoftVer;
      info.W3F4CusMajor_W4F0CusMinor = node.debugInfo;
      break;

    case 2://W4F3Model_W3F0DebugInfTranVer
      info.W4F3Model_W3F0DebugInfTranVer = node.debugInfo;
      break;

    case 3://W3F4HardwareVer_W4F0FirewareMajor
      info.W3F4HardwareVer_W4F0FirewareMajor = node.debugInfo;

      break;

    case 4://W7F0FirewareMinor
      info.W7F0FirewareMinor = node.debugInfo;

      break;

    case 5://W3F4BoradHardVer_W4F0Moth
      info.W3F4BoradHardVer_W4F0Moth = node.debugInfo;

      break;

    case 6://W2F5Output2K4K5K_W5F0Date
      info.W2F5Output2K4K5K_W5F0Date = node.debugInfo;
      break;

    case 7://W1F6GNoise_W1F5SNoise_W1F4MotorCtl_W4F0SnYear
      info.W1F6GNoise_W1F5SNoise_W1F4MotorCtl_W4F0SnYear =
        node.debugInfo;
      break;

    case 8://W7F0SnNumH
      info.W7F0SnNumH = node.debugInfo;
      break;

    case 9://W7F0SnNumL
      info.W7F0SnNumL = node.debugInfo;

      break;

    case 10://W7F0Health
      info.W7F0Health = node.debugInfo;

      break;

    case 11://W3F4CusHardVer_W4F0CusSoftVer
      info.W3F4CusHardVer_W4F0CusSoftVer = node.debugInfo;

    case 12://W7F0LaserCurrent
      info.W7F0LaserCurrent = node.debugInfo;
      break;

    default:
      break;
  }

  if (info.MaxDebugIndex > node.index) {
    info.W3F4CusMajor_W4F0CusMinor = 0xff;
  }

  if (static_cast<int>(node.index) > info.MaxDebugIndex && node.index < 100) {
    info.MaxDebugIndex = static_cast<int>(node.index);
  }
}

/**
 * @brief convert LaserDebug information to device_info
 * @param info      LiDAR LaserDebug information
 * @param value     LiDAR Device information
 * @return true if converted successfully, otherwise false.
 */
inline bool ParseLaserDebugInfo(const LaserDebug &info, device_info &value) {
  bool ret = false;
  uint8_t CustomVerMajor = (static_cast<uint8_t>
                            (info.W3F4CusMajor_W4F0CusMinor) >> 4);
  uint8_t CustomVerMinor = static_cast<uint8_t>
                           (info.W3F4CusMajor_W4F0CusMinor) & 0x0F;
  uint8_t lidarmodel = (static_cast<uint8_t>(info.W4F3Model_W3F0DebugInfTranVer)
                        >> 3);
  uint8_t hardwareVer = static_cast<uint8_t>
                        (info.W3F4HardwareVer_W4F0FirewareMajor) >> 4;
  uint8_t Moth = static_cast<uint8_t>(info.W3F4BoradHardVer_W4F0Moth) & 0x0F;

  uint8_t Date = static_cast<uint8_t>(info.W2F5Output2K4K5K_W5F0Date) & 0x1F;
  uint8_t Year = static_cast<uint8_t>
                 (info.W1F6GNoise_W1F5SNoise_W1F4MotorCtl_W4F0SnYear) & 0x0F;
  uint16_t Number = ((static_cast<uint8_t>(info.W7F0SnNumH) << 7) |
                     static_cast<uint8_t>(info.W7F0SnNumL));

  if (isVersionValid(info) && info.MaxDebugIndex > 0 && Year) {

    if (isSerialNumbValid(info) && info.MaxDebugIndex > 8) {
      value.firmware_version = (CustomVerMajor << 8 | CustomVerMinor);
      value.hardware_version = hardwareVer;
      value.model = lidarmodel;
      uint32_t year = Year + 2015;
      sprintf(reinterpret_cast<char *>(value.serialnum), "%04d", year);
      sprintf(reinterpret_cast<char *>(value.serialnum + 4), "%02d", Moth);
      sprintf(reinterpret_cast<char *>(value.serialnum + 6), "%02d", Date);
      sprintf(reinterpret_cast<char *>(value.serialnum + 8), "%08d", Number);

      for (int i = 0; i < 16; i++) {
        value.serialnum[i] -= 48;
      }

      ret = true;
    }
  }

  return ret;
}

/**
 * @brief print LiDAR version information
 * @param info      LiDAR Device information
 * @param port      LiDAR serial port or IP Address
 * @param baudrate  LiDAR serial baudrate or network port
 * @return true if Device information is valid, otherwise false
 */
inline bool printfVersionInfo(const device_info &info, const std::string &port,
                              int baudrate) {
  if (info.firmware_version == 0 &&
      info.hardware_version == 0) {
    return false;
  }

  uint8_t Major = (uint8_t)(info.firmware_version >> 8);
  uint8_t Minjor = (uint8_t)(info.firmware_version & 0xff);
  printf("[YDLIDAR] Connection established in [%s][%d]:\n"
         "Firmware version: %u.%u\n"
         "Hardware version: %u\n"
         "Model: %s\n"
         "Serial: ",
         port.c_str(),
         baudrate,
         Major,
         Minjor,
         (unsigned int)info.hardware_version,
         lidarModelToString(info.model).c_str());

  for (int i = 0; i < 16; i++) {
    printf("%01X", info.serialnum[i] & 0xff);
  }

  printf("\n");
  fflush(stdout);
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
inline bool isV1Protocol(uint8_t protocol) {
  if (protocol == Protocol_V1) {
    return true;
  }

  return false;
}

}//common
}//core
}//ydlidar
