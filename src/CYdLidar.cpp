//
// The MIT License (MIT)
//
// Copyright (c) 2019-2020 EAIBOT. All rights reserved.
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
#include <map>
#include <numeric>
#include <algorithm>
#include <math.h>
#include <functional>
#include "CYdLidar.h"
#include "core/math/angles.h"
#include "core/serial/common.h"
#include "core/common/DriverInterface.h"
#include "core/common/ydlidar_help.h"
#include "YDlidarDriver.h"
#include "ETLidarDriver.h"
#include "GS1LidarDriver.h"
#include "GS2LidarDriver.h"

using namespace std;
using namespace impl;
using namespace ydlidar::core;
using namespace ydlidar::core::math;

/*-------------------------------------------------------------
            Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar() : lidarPtr(nullptr)
{
  m_SerialPort = "/dev/ydlidar";
  m_SerialBaudrate = 230400;
  m_FixedResolution = false;
  m_Reversion = false;
  m_Inverted = false; //
  m_AutoReconnect = true;
  m_SingleChannel = false;
  m_LidarType = TYPE_TRIANGLE;
  m_MaxAngle = 180.f;
  m_MinAngle = -180.f;
  m_MaxRange = 64.0;
  m_MinRange = 0.01f;
  m_SampleRate = 5;
  //   m_SampleRatebyD1      = 5;
  // defalutSampleRate     = 5;
  m_ScanFrequency = 10;
  isScanning = false;
  m_FixedSize = 720;
  frequencyOffset = 0.4f;
  m_AbnormalCheckCount = 2;
  Major = 0;
  Minjor = 0;
  m_IgnoreArray.clear();
  m_IgnoreString = "";
  m_PointTime = static_cast<int>(1e9 / 5000);
  m_AngleOffset = 0.0f;
  lidar_model = DriverInterface::YDLIDAR_G2B;
  m_Intensity = false;
  m_IntensityBit = 10;
  last_node_time = getTime();
  global_nodes = new node_info[DriverInterface::MAX_SCAN_NODES];
  last_frequency = 0;
  m_FristNodeTime = getTime();
  m_AllNode = 0;
  m_DeviceType = YDLIDAR_TYPE_SERIAL;
  m_SupportMotorDtrCtrl = true;
  m_SupportHearBeat = false;
  m_parsingCompleted = false;
  m_isAngleOffsetCorrected = false;
  m_field_of_view = 360.f;
  memset(&m_LidarVersion, 0, sizeof(LidarVersion));
  zero_offset_angle_scale = 4.f;
}

/*-------------------------------------------------------------
                    ~CYdLidar
-------------------------------------------------------------*/
CYdLidar::~CYdLidar()
{
  disconnecting();

  if (global_nodes)
  {
    delete[] global_nodes;
    global_nodes = NULL;
  }
}

bool CYdLidar::setlidaropt(int optname, const void *optval, int optlen)
{
  if (optval == NULL)
  {
#if defined(_WIN32)
    SetLastError(EINVAL);
#else
    errno = EINVAL;
#endif
    return false;
  }

  if (optname >= LidarPropFixedResolution)
  {
    if (optlen != sizeof(bool))
    {
#if defined(_WIN32)
      SetLastError(EINVAL);
#else
      errno = EINVAL;
#endif
      return false;
    }
  }
  else if (optname >= LidarPropMaxRange)
  {
    if (optlen != sizeof(float))
    {
#if defined(_WIN32)
      SetLastError(EINVAL);
#else
      errno = EINVAL;
#endif
      return false;
    }
  }
  else if (optname >= LidarPropSerialBaudrate)
  {
    if (optlen != sizeof(int))
    {
#if defined(_WIN32)
      SetLastError(EINVAL);
#else
      errno = EINVAL;
#endif
      return false;
    }
  }
  else
  {
  }

  bool ret = true;

  switch (optname)
  {
  case LidarPropSerialPort:
    m_SerialPort = (const char *)optval;
    break;

  case LidarPropIgnoreArray:
    m_IgnoreString = (const char *)optval;
    m_IgnoreArray = ydlidar::split(m_IgnoreString, ',');

    if (m_IgnoreArray.size() % 2 != 0)
    {
      m_IgnoreArray.clear();
      ret = false;
    }

    break;

  case LidarPropFixedResolution:
    m_FixedResolution = *(bool *)(optval);
    break;

  case LidarPropReversion:
    m_Reversion = *(bool *)(optval);
    break;

  case LidarPropInverted:
    m_Inverted = *(bool *)(optval);
    break;

  case LidarPropAutoReconnect:
    m_AutoReconnect = *(bool *)(optval);
    break;

  case LidarPropSingleChannel:
    m_SingleChannel = *(bool *)(optval);
    break;

  case LidarPropIntenstiy:
    m_Intensity = *(bool *)(optval);
    break;
  case LidarPropIntenstiyBit:
    m_IntensityBit = *(int *)(optval);
    break;

  case LidarPropSupportMotorDtrCtrl:
    m_SupportMotorDtrCtrl = *(bool *)(optval);
    break;

  case LidarPropSupportHeartBeat:
    m_SupportHearBeat = *(bool *)(optval);
    break;

  case LidarPropMaxRange:
    m_MaxRange = *(float *)(optval);
    break;

  case LidarPropMinRange:
    m_MinRange = *(float *)(optval);
    break;

  case LidarPropMaxAngle:
    m_MaxAngle = *(float *)(optval);
    break;

  case LidarPropMinAngle:
    m_MinAngle = *(float *)(optval);
    break;

  case LidarPropScanFrequency:
    m_ScanFrequency = *(float *)(optval);
    break;

  case LidarPropSerialBaudrate:
    m_SerialBaudrate = *(int *)(optval);
    break;

  case LidarPropLidarType:
    m_LidarType = *(int *)(optval);
    break;

  case LidarPropDeviceType:
    m_DeviceType = *(int *)(optval);
    break;

  case LidarPropSampleRate:
    m_SampleRate = *(int *)(optval);
    break;

  case LidarPropAbnormalCheckCount:
    m_AbnormalCheckCount = *(int *)(optval);
    break;

  default:
    ret = false;
    break;
  }

  return ret;
}

bool CYdLidar::getlidaropt(int optname, void *optval, int optlen)
{
  if (optval == NULL)
  {
#if defined(_WIN32)
    SetLastError(EINVAL);
#else
    errno = EINVAL;
#endif
    return false;
  }

  if (optname >= LidarPropFixedResolution)
  {
    if (optlen != sizeof(bool))
    {
#if defined(_WIN32)
      SetLastError(EINVAL);
#else
      errno = EINVAL;
#endif
      return false;
    }
  }
  else if (optname >= LidarPropMaxRange)
  {
    if (optlen != sizeof(float))
    {
#if defined(_WIN32)
      SetLastError(EINVAL);
#else
      errno = EINVAL;
#endif
      return false;
    }
  }
  else if (optname >= LidarPropSerialBaudrate)
  {
    if (optlen != sizeof(int))
    {
#if defined(_WIN32)
      SetLastError(EINVAL);
#else
      errno = EINVAL;
#endif
      return false;
    }
  }
  else
  {
  }

  bool ret = true;

  switch (optname)
  {
  case LidarPropSerialPort:
    memcpy(optval, m_SerialPort.c_str(), optlen);
    break;

  case LidarPropIgnoreArray:
    memcpy(optval, m_IgnoreString.c_str(), optlen);
    break;

  case LidarPropFixedResolution:
    memcpy(optval, &m_FixedResolution, optlen);
    break;

  case LidarPropReversion:
    memcpy(optval, &m_Reversion, optlen);
    break;

  case LidarPropInverted:
    memcpy(optval, &m_Inverted, optlen);
    break;

  case LidarPropAutoReconnect:
    memcpy(optval, &m_AutoReconnect, optlen);
    break;

  case LidarPropSingleChannel:
    memcpy(optval, &m_SingleChannel, optlen);
    break;

  case LidarPropIntenstiy:
    memcpy(optval, &m_Intensity, optlen);
    break;
  case LidarPropIntenstiyBit:
    memcpy(optval, &m_IntensityBit, optlen);
    break;

  case LidarPropSupportMotorDtrCtrl:
    memcpy(optval, &m_SupportMotorDtrCtrl, optlen);
    break;

  case LidarPropSupportHeartBeat:
    memcpy(optval, &m_SupportHearBeat, optlen);
    break;

  case LidarPropMaxRange:
    memcpy(optval, &m_MaxRange, optlen);
    break;

  case LidarPropMinRange:
    memcpy(optval, &m_MinRange, optlen);
    break;

  case LidarPropMaxAngle:
    memcpy(optval, &m_MaxAngle, optlen);
    break;

  case LidarPropMinAngle:
    memcpy(optval, &m_MinAngle, optlen);
    break;

  case LidarPropScanFrequency:
    memcpy(optval, &m_ScanFrequency, optlen);
    break;

  case LidarPropSerialBaudrate:
    memcpy(optval, &m_SerialBaudrate, optlen);
    break;

  case LidarPropLidarType:
    memcpy(optval, &m_LidarType, optlen);
    break;

  case LidarPropDeviceType:
    memcpy(optval, &m_DeviceType, optlen);
    break;

  case LidarPropSampleRate:
    memcpy(optval, &m_SampleRate, optlen);
    break;

  case LidarPropAbnormalCheckCount:
    memcpy(optval, &m_AbnormalCheckCount, optlen);
    break;

  default:
    ret = false;
    break;
  }

  return ret;
}

/*-------------------------------------------------------------
                        initialize
-------------------------------------------------------------*/
bool CYdLidar::initialize()
{
  uint32_t t = getms();
  if (!checkCOMMs())
  {
    fprintf(stderr,
            "[CYdLidar::initialize] Error initializing YDLIDAR check Comms.\n");
    fflush(stderr);
    return false;
  }

  if (!checkStatus())
  {
    fprintf(stderr,
            "[CYdLidar::initialize] Error initializing YDLIDAR check status under [%s] and [%d].\n",
            m_SerialPort.c_str(), m_SerialBaudrate);
    fflush(stderr);
    return false;
  }

  printf("LiDAR init success, Elapsed time %u ms\n", getms() - t);
  fflush(stdout);

  return true;
}

/*-------------------------------------------------------------
                        initialize
-------------------------------------------------------------*/
void CYdLidar::GetLidarVersion(LidarVersion &version)
{
  memcpy(&version, &m_LidarVersion, sizeof(LidarVersion));
}

/*-------------------------------------------------------------
                        turnOn
-------------------------------------------------------------*/
bool CYdLidar::turnOn()
{
  if (isScanning && lidarPtr->isscanning())
  {
    return true;
  }

  uint32_t t = getms();
  // start scan...
  result_t op_result = lidarPtr->startScan();

  if (!IS_OK(op_result))
  {
    op_result = lidarPtr->startScan();

    if (!IS_OK(op_result))
    {
      lidarPtr->stop();
      fprintf(stderr, "[CYdLidar] Failed to start scan mode: %x\n", op_result);
      isScanning = false;
      return false;
    }
  }

  printf("[CYdLidar] Successed to start scan mode, Elapsed time %u ms\n", getms() - t);
  fflush(stdout);

  m_PointTime = lidarPtr->getPointTime();

  // //获取强度标识
  // lidarPtr->getIntensityFlag();

  //计算采样率
  if (checkLidarAbnormal())
  {
    lidarPtr->stop();
    fprintf(stderr,
            "[CYdLidar] Failed to turn on the Lidar, because the lidar is [%s].\n",
            DriverInterface::DescribeDriverError(lidarPtr->getDriverError()));
    isScanning = false;
    return false;
  }

  if (m_SingleChannel && !isNetTOFLidar(m_LidarType))
  {
    handleSingleChannelDevice();
  }
  else
  {
    printf("[YDLIDAR INFO1] Current Sampling Rate : %dK\n", m_SampleRate);
  }

  m_field_of_view = 360.f;

  if (isNetTOFLidar(m_LidarType))
  {
    lidarConfig cfg = lidarPtr->getFinishedScanCfg();
    m_field_of_view = cfg.fov_end - cfg.fov_start;

    if (cfg.fov_end - 180 < m_MaxAngle)
    {
      m_MaxAngle = cfg.fov_end - 180;
    }

    if (cfg.fov_start - 180 > m_MinAngle)
    {
      m_MinAngle = cfg.fov_start - 180;
    }
  }

  last_frequency = 0;
  m_FristNodeTime = getTime();
  m_AllNode = 0;
  m_PointTime = lidarPtr->getPointTime();
  isScanning = true;
  lidarPtr->setAutoReconnect(m_AutoReconnect);
  printf("[YDLIDAR INFO] Now YDLIDAR is scanning ......\n");
  fflush(stdout);
  return true;
}

/*-------------------------------------------------------------
            doProcessSimple
-------------------------------------------------------------*/
bool CYdLidar::doProcessSimple(LaserScan &outscan)
{
  // Boud?
  if (!checkHardware())
  {
    delay(200 / m_ScanFrequency);
    m_AllNode = 0;
    m_FristNodeTime = getTime();
    return false;
  }

  size_t count = ydlidar::YDlidarDriver::MAX_SCAN_NODES;

  // wait Scan data:
  uint64_t tim_scan_start = getTime();
  uint64_t startTs = tim_scan_start;
  //从缓存中获取已采集的一圈扫描数据
  result_t op_result = lidarPtr->grabScanData(global_nodes, count);
  uint64_t tim_scan_end = getTime();
  uint64_t endTs = tim_scan_end;
  uint64_t sys_scan_time = tim_scan_end - tim_scan_start; //获取一圈数据所花费的时间
  outscan.points.clear();

  // Fill in scan data:
  if (IS_OK(op_result))
  {
    int offsetSize = 0;

    if (isNetTOFLidar(m_LidarType))
    {
      double echo_angle = static_cast<double>(m_field_of_view * 1.0 / count);

      if (echo_angle != 0.0)
      {
        offsetSize = static_cast<int>((360 - m_field_of_view) / echo_angle);
      }
    }

    //根据采样频率计算的采样间隔时间计算出来总的扫描时间
    uint64_t scan_time = m_PointTime * (count - 1 + offsetSize);
    int timeDiff = static_cast<int>(sys_scan_time - scan_time);

    bool HighPayLoad = false;

    if (global_nodes[0].stamp > 0 &&
        global_nodes[0].stamp < tim_scan_start)
    {
      tim_scan_end = global_nodes[0].stamp;
      HighPayLoad = true;
    }

    tim_scan_end -= m_PointTime;
    tim_scan_end -= global_nodes[0].delay_time;
    tim_scan_start = tim_scan_end - scan_time;

    if (!HighPayLoad && tim_scan_start < startTs)
    {
      tim_scan_start = startTs;
      tim_scan_end = tim_scan_start + scan_time;
    }

    if ((last_node_time + m_PointTime) >= tim_scan_start &&
        (last_node_time + m_PointTime) < endTs - scan_time)
    {
      tim_scan_start = last_node_time + m_PointTime;
      tim_scan_end = tim_scan_start + scan_time;
    }

    if (m_AllNode == 0 && abs(timeDiff) < 10 * 1e6)
    {
      m_FristNodeTime = tim_scan_start;
      m_AllNode += (count + offsetSize);
    }
    else if (m_AllNode != 0)
    {
      m_AllNode += (count + offsetSize);
    }

    last_node_time = tim_scan_end;

    if (m_MaxAngle < m_MinAngle)
    {
      float temp = m_MinAngle;
      m_MinAngle = m_MaxAngle;
      m_MaxAngle = temp;
    }

    int all_node_count = count;
    LaserDebug debug;

    memset(&debug, 0, sizeof(debug));
    outscan.config.min_angle = math::from_degrees(m_MinAngle);
    outscan.config.max_angle = math::from_degrees(m_MaxAngle);
    //将首末点采集时间差作为采集时长
    //        printf("stamp [%llu]-[%llu]\n", global_nodes[0].stamp, global_nodes[count - 1].stamp);
    outscan.config.scan_time = static_cast<float>((global_nodes[count - 1].stamp - global_nodes[0].stamp)) / 1e9;
    //        outscan.config.scan_time = static_cast<float>(scan_time * 1.0 / 1e9);
    outscan.config.time_increment = outscan.config.scan_time / (double)(count - 1);
    outscan.config.min_range = m_MinRange;
    outscan.config.max_range = m_MaxRange;
    //模组编号
    outscan.moduleNum = global_nodes[0].index;
    //环境标记
    outscan.envFlag = global_nodes[0].is + (uint16_t(global_nodes[1].is) << 8);
    //将一圈中第一个点采集时间作为该圈数据采集时间
    if (global_nodes[0].stamp > 0)
      outscan.stamp = global_nodes[0].stamp;
    else
      outscan.stamp = 0;

    float scanfrequency = 0.0;

    if (m_FixedResolution)
    {
      all_node_count = m_FixedSize;
    }

    outscan.config.angle_increment = math::from_degrees(m_field_of_view) /
      (all_node_count - 1);

    float range = 0.0;
    float intensity = 0.0;
    float angle = 0.0;
    debug.MaxDebugIndex = 0;

    // printf("AngleOffset %f\n", m_AngleOffset);

    //遍历一圈点
    for (int i = 0; i < count; i++)
    {
      const node_info& node = global_nodes[i];

      // printf("%lu a %.01f r %u\n", 
      //   i, float(node.angle_q6_checkbit) / 64.0f, node.distance_q2);

      if (isNetTOFLidar(m_LidarType))
      {
        angle = static_cast<float>(global_nodes[i].angle_q6_checkbit / 100.0f) +
                m_AngleOffset;
      }
      else
      {
        angle = static_cast<float>((global_nodes[i].angle_q6_checkbit >>
                                    LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) /
                                   64.0f) +
                m_AngleOffset;
      }

      if (isOctaveLidar(lidar_model) ||
          isOldVersionTOFLidar(lidar_model, Major, Minjor))
      {
        range = static_cast<float>(global_nodes[i].distance_q2 / 2000.f);
      }
      else
      {
        if (isTOFLidar(m_LidarType) || 
          isNetTOFLidar(m_LidarType) ||
          isGSLidar(m_LidarType))
        {
          range = static_cast<float>(global_nodes[i].distance_q2 / 1000.f);
        }
        else
        {
          range = static_cast<float>(global_nodes[i].distance_q2 / 4000.f);
        }
      }

      intensity = static_cast<float>(global_nodes[i].sync_quality);

      angle = math::from_degrees(angle);

      if (global_nodes[i].scan_frequence != 0)
      {
        scanfrequency = global_nodes[i].scan_frequence / 10.0;

        if (isTOFLidar(m_LidarType))
        {
          if (!isOldVersionTOFLidar(lidar_model, Major, Minjor))
          {
            scanfrequency = global_nodes[i].scan_frequence / 10.0 + 3.0;
          }
        }
      }

      // Rotate 180 degrees or not
      if (m_Reversion || isNetTOFLidar(m_LidarType))
      {
        angle = angle + M_PI;
      }

      // Is it counter clockwise
      if (m_Inverted)
      {
        angle = 2 * M_PI - angle;
      }

      angle = math::normalize_angle(angle);

      // ignore angle
      if (isRangeIgnore(angle))
      {
        range = 0.0;
      }

      //过滤点
      if (!isRangeValid(range) ||
        (m_SunNoise && node.is == SUNNOISEINTENSITY) ||
        (m_GlassNoise && node.is == GLASSNOISEINTENSITY))
      {
        range = .0;
      }

      // printf("i %d d %.03f a %.02f flag %u\n",
      //   i, range, angle*180.0/M_PI, node.sync_flag);

      if (angle >= outscan.config.min_angle &&
          angle <= outscan.config.max_angle)
      {
        LaserPoint point;
        point.angle = angle;
        point.range = range;
        point.intensity = intensity;

        outscan.points.push_back(point);
      }

      parsePackageNode(global_nodes[i], debug);
      if (global_nodes[i].error_package)
      {
        debug.MaxDebugIndex = 255;
      }
    } //end for (int i = 0; i < count; i++)

    if (m_FixedResolution)
    {
      outscan.points.resize(all_node_count);
    }

    //解析V2协议雷达扫描数据中ct信息中的设备信息
    handleVersionInfoByPackage(debug);
    // resample sample rate
    resample(scanfrequency, count, tim_scan_end, tim_scan_start);
    return true;
  }
  else
  {
    if (IS_FAIL(op_result))
    {
      // Error? Retry connection
    }

    if (lidarPtr->getDriverError() != NoError)
    {
      fprintf(stderr, "[YDLIDAR ERROR]: %s\n",
              DriverInterface::DescribeDriverError(lidarPtr->getDriverError()));
      fflush(stderr);
    }

    m_AllNode = 0;
    m_FristNodeTime = tim_scan_start;
  }

  return false;
}

/*-------------------------------------------------------------
            turnOff
-------------------------------------------------------------*/
bool CYdLidar::turnOff()
{
  if (lidarPtr)
  {
    lidarPtr->stop();
  }

  if (isScanning)
  {
    printf("[YDLIDAR INFO] Now YDLIDAR Scanning has stopped ......\n");
    fflush(stdout);
  }

  isScanning = false;
  return true;
}

/*-------------------------------------------------------------
                    disconnecting
-------------------------------------------------------------*/
void CYdLidar::disconnecting()
{
  if (lidarPtr)
  {
    lidarPtr->disconnect();
    delete lidarPtr;
    lidarPtr = nullptr;
  }

  isScanning = false;
}

/*-------------------------------------------------------------
                    getAngleOffset
-------------------------------------------------------------*/
float CYdLidar::getAngleOffset() const
{
  return m_AngleOffset;
}

/*-------------------------------------------------------------
                    isAngleOffsetCorrected
-------------------------------------------------------------*/
bool CYdLidar::isAngleOffsetCorrected() const
{
  return m_isAngleOffsetCorrected;
}

/*-------------------------------------------------------------
                    DescribeError
-------------------------------------------------------------*/
const char *CYdLidar::DescribeError() const
{
  char const *value = "";

  if (lidarPtr)
  {
    return lidarPtr->DescribeError();
  }

  return value;
}

/*-------------------------------------------------------------
                    getDriverError
-------------------------------------------------------------*/
DriverError CYdLidar::getDriverError() const
{
  DriverError er = UnknownError;

  if (lidarPtr)
  {
    return lidarPtr->getDriverError();
  }

  return er;
}

bool CYdLidar::setWorkMode(int mode, uint8_t addr)
{
  if (lidarPtr)
    return (lidarPtr->setWorkMode(mode, addr) == RESULT_OK);
  else
    return false;
}

void CYdLidar::enableSunNoise(bool e)
{
  m_SunNoise = e;
}

void CYdLidar::enableGlassNoise(bool e)
{
  m_GlassNoise = e;
}

bool CYdLidar::getUserVersion(std::string &version)
{
    if (!checkHardware())
    {
        printf("[YDLIDAR] Device is not open!\n");
        return false;
    }

    size_t count = ydlidar::YDlidarDriver::MAX_SCAN_NODES;
    result_t op_result = lidarPtr->grabScanData(global_nodes, count);
    if (IS_OK(op_result) && count > 2)
    {
        uint8_t userVerion = global_nodes[USERVERSIONNDEX].debugInfo;
        version = std::to_string(userVerion & 0xc0) + "." + std::to_string(userVerion & 0x3f);
        return true;
    }

    return false;
}

/*-------------------------------------------------------------
                    isRangeValid
-------------------------------------------------------------*/
bool CYdLidar::isRangeValid(double reading) const
{
  if (reading >= m_MinRange && reading <= m_MaxRange)
  {
    return true;
  }

  return false;
}

/*-------------------------------------------------------------
                    isRangeIgnore
-------------------------------------------------------------*/
bool CYdLidar::isRangeIgnore(double angle) const
{
  bool ret = false;

  for (uint16_t j = 0; j < m_IgnoreArray.size(); j = j + 2)
  {
    if ((math::from_degrees(m_IgnoreArray[j]) <= angle) &&
        (angle <= math::from_degrees(m_IgnoreArray[j + 1])))
    {
      ret = true;
      break;
    }
  }

  return ret;
}

/*-------------------------------------------------------------
                    handleVersionInfoByPackage
-------------------------------------------------------------*/
void CYdLidar::handleVersionInfoByPackage(const LaserDebug &debug)
{
  if (m_parsingCompleted)
  {
    return;
  }

  device_info info;
  memset(&info, 0, sizeof(device_info));

  if (ParseLaserDebugInfo(debug, info))
  {
    if (printfVersionInfo(info, m_SerialPort, m_SerialBaudrate))
    {
      std::string serial_number;
      Major = (uint8_t)(info.firmware_version >> 8);
      Minjor = (uint8_t)(info.firmware_version & 0xff);
      m_LidarVersion.hardware = info.hardware_version;
      m_LidarVersion.soft_major = Major;
      m_LidarVersion.soft_minor = Minjor / 10;
      m_LidarVersion.soft_patch = Minjor % 10;
      memcpy(&m_LidarVersion.sn[0], &info.serialnum[0], 16);

      for (int i = 0; i < 16; i++)
      {
        serial_number += std::to_string(info.serialnum[i] & 0xff);
      }

      m_SerialNumber = serial_number;
      m_parsingCompleted = true;
    }
  }
}

/*-------------------------------------------------------------
                    resample
-------------------------------------------------------------*/
void CYdLidar::resample(int frequency, int count, uint64_t tim_scan_end,
                        uint64_t tim_scan_start)
{
  //重新校准采样率
  // if( (lidar_model  == DriverInterface::YDLIDAR_TG15)
  //     || (lidar_model  == DriverInterface::YDLIDAR_TG30)
  //     || (lidar_model  == DriverInterface::YDLIDAR_TG50) )
  //   {
  //       m_SampleRate = m_SampleRatebyD1;
  //   }

  if (frequency > 3 && frequency <= 15.7 &&
      (frequency - last_frequency) < 0.05)
  {
    int sample = static_cast<int>((frequency * count + 500) / 1000);

    if (sample != m_SampleRate)
    {
    }
  }

  last_frequency = frequency;
  int realSampleRate = 0;

  if (m_AllNode != 0)
  {
    realSampleRate = 1e9 * m_AllNode / (tim_scan_end - m_FristNodeTime);
    int RateDiff = std::abs(static_cast<int>(realSampleRate - m_SampleRate * 1000));

    if (RateDiff > 1000 ||
        (static_cast<int64_t>(tim_scan_end - m_FristNodeTime) > 10 * 1e9 &&
         RateDiff > 30))
    {
      m_AllNode = 0;
      m_FristNodeTime = tim_scan_start;
    }
  }
}

/*-------------------------------------------------------------
            checkLidarAbnormal
-------------------------------------------------------------*/
bool CYdLidar::checkLidarAbnormal()
{
  size_t count = ydlidar::YDlidarDriver::MAX_SCAN_NODES;
  int check_abnormal_count = 0;

  if (m_AbnormalCheckCount < 2)
  {
    m_AbnormalCheckCount = 2;
  }

  result_t op_result = RESULT_FAIL;
  std::vector<int> data;
  int buffer_count = 0;

  while (check_abnormal_count < m_AbnormalCheckCount)
  {
    // Ensure that the voltage is insufficient or the motor resistance is high, causing an abnormality.
    if (check_abnormal_count > 0)
    {
      delay(check_abnormal_count * 1000);
    }

    float scan_time = 0.0;
    uint64_t start_time = 0;
    uint64_t end_time = 0;
    op_result = RESULT_OK;

    while (buffer_count < 10 &&
           (scan_time < 0.05 || !lidarPtr->getSingleChannel()) &&
           IS_OK(op_result))
    {
      start_time = getTime();
      count = ydlidar::YDlidarDriver::MAX_SCAN_NODES;
      op_result = lidarPtr->grabScanData(global_nodes, count);
      end_time = getTime();
      scan_time = 1.0 * static_cast<int64_t>(end_time - start_time) / 1e9;
      buffer_count++;

      if (IS_OK(op_result))
      {
        if (isNetTOFLidar(m_LidarType))
        {
          return !IS_OK(op_result);
        }

        if (CalculateSampleRate(count, scan_time))
        {
          if (!lidarPtr->getSingleChannel())
          {
            return !IS_OK(op_result);
          }
        }
      }
      else
      {
        check_abnormal_count++;
      }
    }

    if (IS_OK(op_result) && lidarPtr->getSingleChannel())
    {
      data.push_back(count);
      int collection = 0;

      while (collection < 5)
      {
        count = ydlidar::YDlidarDriver::MAX_SCAN_NODES;
        start_time = getTime();
        op_result = lidarPtr->grabScanData(global_nodes, count);
        end_time = getTime();

        if (IS_OK(op_result))
        {
          if (isNetTOFLidar(m_LidarType))
          {
            return !IS_OK(op_result);
          }

          if (std::abs(static_cast<int>(data.front() - count)) > 10)
          {
            data.erase(data.begin());
          }

          scan_time = 1.0 * static_cast<int64_t>(end_time - start_time) / 1e9;
          bool ret = CalculateSampleRate(count, scan_time);

          if (scan_time > 0.05 && scan_time < 0.5 && lidarPtr->getSingleChannel())
          {
            if (!ret)
            {
              m_SampleRate = static_cast<int>((count / scan_time + 500) / 1000);
              m_PointTime = 1e9 / (m_SampleRate * 1000);
              lidarPtr->setPointTime(m_PointTime);
            }
          }

          data.push_back(count);

          if (ret)
          {
            break;
          }
        }

        collection++;
      }

      if (data.size() > 1)
      {
        int total = accumulate(data.begin(), data.end(), 0);
        int mean = total / data.size(); // mean value
        m_FixedSize = (static_cast<int>((mean + 5) / 10)) * 10;
        printf("[YDLIDAR]:Single Fixed Size: %d\n", m_FixedSize);
        printf("[YDLIDAR]:Sample Rate: %dK\n", m_SampleRate);
        return false;
      }
    }

    check_abnormal_count++;
  }

  return !IS_OK(op_result);
}

/*-------------------------------------------------------------
                    removeExceptionSample
-------------------------------------------------------------*/
inline void removeExceptionSample(std::map<int, int> &smap)
{
  if (smap.size() < 2)
  {
    return;
  }

  std::map<int, int>::iterator last = smap.begin();
  std::map<int, int>::iterator its = smap.begin();

  while (its != smap.end())
  {
    if (last->second > its->second)
    {
      smap.erase(its++);
    }
    else if (last->second < its->second)
    {
      its = smap.erase(last);
      last = its;
      its++;
    }
    else
    {
      its++;
    }
  }
}

/*-------------------------------------------------------------
                    CalculateSampleRate
-------------------------------------------------------------*/
bool CYdLidar::CalculateSampleRate(int count, double scan_time)
{
  if (count < 1)
    return false;

  // 1、如果雷达支持直接获取采样率，则使用获取的采样率，此时将默认采样率值置为该值
  // 2、如果有设置默认采样率，判断当前雷达型号对应的默认采样率值的个数，个数为1，
  //直接使用该采样率，个数不为1则根据实时采样率进行匹配
  // 3、如果没有设置默认采样率，则使用实时采样率
  int sr = 0;
  bool ret = false;

  if (global_nodes[0].scan_frequence != 0)
  {
    //如果解析到转速信息，根据转速计算采样率
    double scanfrequency = global_nodes[0].scan_frequence / 10.0;
    if (isTOFLidar(m_LidarType) &&
        !isOldVersionTOFLidar(lidar_model, Major, Minjor))
    {
      scanfrequency = global_nodes[0].scan_frequence / 10.0 + 3.0;
    }
    sr = static_cast<int>((count * scanfrequency + 500) / 1000);
  }
  else
  {
    //如果没有解析到转速信息，根据时间计算采样率
    if (scan_time > 0.04 && scan_time < 0.4)
    {
      sr = static_cast<int>((count / scan_time + 500) / 1000);
    }
  }

  // printf("[YDLIDAR] Calc Sample Rate: %dK\n", sr);

  size_t size = defalutSampleRate.size();
  if (size)
  {
    if (size == 1)
    {
      sr = defalutSampleRate.front();
      ret = true;
      // printf("[YDLIDAR] Calc Sample Rate1: %dK\n", sr);
    }
    else
    {
      float d = .0;
      for (size_t i=0; i<size; ++i)
      {
        if (i + 1 < size) //如果是最后
        {
          //按权值2:8分
          d = defalutSampleRate.at(i + 1) - defalutSampleRate.at(i);
          if (float(sr) <= float(defalutSampleRate.at(i) + 0.2 * d))
          {
            sr = defalutSampleRate.at(i);
            break;
          }
        }
        else
        {
          sr = defalutSampleRate.at(i);
        }
      }
      ret = true;
      // printf("[YDLIDAR] Calc Sample Rate2: %dK\n", sr);
    }
  }
  else
  {
    if (sr)
      SampleRateMap[sr] ++;
    if (isValidSampleRate(SampleRateMap))
      ret = true;
    // printf("[YDLIDAR] Calc Sample Rate3: %dK\n", sr);
  }

  if (ret)
  {
    m_SampleRate = sr;
    m_PointTime = 1e9 / (m_SampleRate * 1000);
    lidarPtr->setPointTime(m_PointTime);
    if (!m_SingleChannel)
      m_FixedSize = m_SampleRate * 1000 / (m_ScanFrequency - 0.1);
    printf("[YDLIDAR] Fixed Size: %d\n", m_FixedSize);
    printf("[YDLIDAR] Sample Rate: %dK\n", m_SampleRate);
  }

  return ret;

  // if (global_nodes[0].scan_frequence != 0) //如果解析到转速信息
  // {
  //   double scanfrequency;
  //   scanfrequency = global_nodes[0].scan_frequence / 10.0;

  //   if (isTOFLidar(m_LidarType))
  //   {
  //     if (!isOldVersionTOFLidar(lidar_model, Major, Minjor))
  //     {
  //       scanfrequency = global_nodes[0].scan_frequence / 10.0 + 3.0;
  //     }
  //   }

  //   int samplerate = static_cast<int>((count * scanfrequency + 500) / 1000);
  //   int cnt = 0;

  //   float sr = float(count * scanfrequency + 500) / 1000;
  //   printf("[CalculateSampleRate] Sample Rate: %.03fK(%dK)\n", sr, samplerate);
  //   // fflush(stdout);

  //   if (SampleRateMap.find(samplerate) != SampleRateMap.end())
  //   {
  //     cnt = SampleRateMap[samplerate];
  //   }

  //   cnt++;
  //   SampleRateMap[samplerate] = cnt;

  //   if (isValidSampleRate(SampleRateMap) ||
  //       defalutSampleRate == samplerate ||
  //       m_SampleRate == samplerate)
  //   {
  //     m_SampleRate = samplerate;
  //     m_PointTime = 1e9 / (m_SampleRate * 1000);
  //     lidarPtr->setPointTime(m_PointTime);

  //     if (!m_SingleChannel)
  //     {
  //       m_FixedSize = m_SampleRate * 1000 / (m_ScanFrequency - 0.1);
  //     }

  //     printf("[YDLIDAR1]:Fixed Size: %d\n", m_FixedSize);
  //     printf("[YDLIDAR1]:Sample Rate: %dK\n", m_SampleRate);
  //     return true;
  //   }
  //   else
  //   {
  //     if (SampleRateMap.size() > 1)
  //     {
  //       SampleRateMap.clear();
  //     }
  //   }
  // }
  // else //如果没有解析到转速信息
  // {
  //   if (scan_time > 0.04 && scan_time < 0.4)
  //   {
  //     int samplerate = static_cast<int>((count / scan_time + 500) / 1000);

  //     if (defalutSampleRate == samplerate ||
  //         m_SampleRate == samplerate)
  //     {
  //       m_SampleRate = samplerate;
  //       m_PointTime = 1e9 / (m_SampleRate * 1000);
  //       lidarPtr->setPointTime(m_PointTime);

  //       if (!m_SingleChannel)
  //       {
  //         m_FixedSize = m_SampleRate * 1000 / (m_ScanFrequency - 0.1);
  //       }

  //       printf("[YDLIDAR3]:Fixed Size: %d\n", m_FixedSize);
  //       printf("[YDLIDAR3]:Sample Rate: %dK\n", m_SampleRate);
  //       return true;
  //     }
  //     else
  //     {
  //       int cnt = 0;

  //       if (SampleRateMap.find(samplerate) != SampleRateMap.end())
  //       {
  //         cnt = SampleRateMap[samplerate];
  //       }

  //       cnt++;
  //       SampleRateMap[samplerate] = cnt;

  //       if (SampleRateMap.size() > 1)
  //       {
  //         SampleRateMap.clear();
  //       }

  //       if (isValidSampleRate(SampleRateMap))
  //       {
  //         m_SampleRate = samplerate;
  //         m_PointTime = 1e9 / (m_SampleRate * 1000);
  //         lidarPtr->setPointTime(m_PointTime);

  //         if (!m_SingleChannel)
  //         {
  //           m_FixedSize = m_SampleRate * 1000 / (m_ScanFrequency - 0.1);
  //         }

  //         printf("[YDLIDAR2]:Fixed Size: %d\n", m_FixedSize);
  //         printf("[YDLIDAR2]:Sample Rate: %dK\n", m_SampleRate);
  //         return true;
  //       }
  //     }
  //   }
  // }

  // return false;
}

/*-------------------------------------------------------------
                    getDeviceHealth
-------------------------------------------------------------*/
bool CYdLidar::getDeviceHealth()
{
  if (!lidarPtr)
  {
    return false;
  }

  lidarPtr->stop();

  result_t op_result;
  device_health healthinfo;
  memset(&healthinfo, 0, sizeof(device_health));
  op_result = lidarPtr->getHealth(healthinfo,
                                  DriverInterface::DEFAULT_TIMEOUT / 2);

  if (IS_OK(op_result))
  {
    printf("[YDLIDAR]:Lidar running correctly ! The health status: %s\n",
           (int)healthinfo.status == 0 ? "good" : "bad");

    if (healthinfo.status == 2)
    {
      fprintf(stderr,
              "Error, YDLidar internal error detected. Please reboot the device to retry.\n");
      return false;
    }
    else
    {
      return true;
    }
  }
  else
  {
    fprintf(stderr, "Error, cannot retrieve YDLidar health code: %x\n", op_result);
    return false;
  }
}

/*-------------------------------------------------------------
                    getDeviceInfo
-------------------------------------------------------------*/
bool CYdLidar::getDeviceInfo()
{
  if (!lidarPtr)
  {
    return false;
  }

  bool ret = false;
  device_info devinfo;
  memset(&devinfo, 0, sizeof(device_info));

  result_t op_result = lidarPtr->getDeviceInfo(devinfo,
    DriverInterface::DEFAULT_TIMEOUT / 2);
  if (!IS_OK(op_result))
  {
    fprintf(stderr, "[YDLIDAR INFO] Fail to get device information\n");
    return false;
  }

  if (!isSupportLidar(devinfo.model))
  {
    printf("[YDLIDAR INFO] Current SDK does not support current lidar models[%s]\n",
      lidarModelToString(devinfo.model).c_str());
    return false;
  }

  // check Lidar Type Config
  if (isTOFLidarByModel(devinfo.model))
  {
    if (!isTOFLidar(m_LidarType))
    {
      fprintf(stderr, "Incorrect Lidar Type setting...\n");
      m_LidarType = TYPE_TOF;
      lidarPtr->setLidarType(m_LidarType);
    }
  }
  else
  {
    // if (!isTriangleLidar(m_LidarType) &&
    //     !isNetTOFLidarByModel(devinfo.model) &&
    //     !m_SingleChannel)
    // {
    //   fprintf(stderr, "Incorrect Lidar Type setting, Reset Type to %d...\n",
    //           TYPE_TRIANGLE);
    //   m_LidarType = TYPE_TRIANGLE;
    //   lidarPtr->setLidarType(m_LidarType);
    // }
  }

  frequencyOffset = 0.4;
  lidar_model = devinfo.model;
  bool intensity = hasIntensity(devinfo.model);
  defalutSampleRate = getDefaultSampleRate(devinfo.model);
  // printf("getDefaultSampleRate %d\n", defalutSampleRate.size());

  intensity = m_Intensity;
  std::string serial_number;
  lidarPtr->setIntensities(intensity);
  //  printf("Set Lidar Intensity Bit count %d\n", m_IntensityBit);
  lidarPtr->setIntensityBit(m_IntensityBit);
  ret = true;

  if (printfVersionInfo(devinfo, m_SerialPort, m_SerialBaudrate))
  {
    Major = (uint8_t)(devinfo.firmware_version >> 8);
    Minjor = (uint8_t)(devinfo.firmware_version & 0xff);
    m_LidarVersion.hardware = devinfo.hardware_version;
    m_LidarVersion.soft_major = Major;
    m_LidarVersion.soft_minor = Minjor / 10;
    m_LidarVersion.soft_patch = Minjor % 10;
    memcpy(&m_LidarVersion.sn[0], &devinfo.serialnum[0], 16);

    for (int i = 0; i < 16; i++)
    {
      serial_number += std::to_string(devinfo.serialnum[i] & 0xff);
    }

    m_SerialNumber = serial_number;
    m_parsingCompleted = true;
    zero_offset_angle_scale = lidarZeroOffsetAngleScale(devinfo.model,
                                                        devinfo.firmware_version >> 8, devinfo.firmware_version & 0x00ff);
  }

  // uint32_t t = getms();

  if (hasSampleRate(devinfo.model))
  {
    checkSampleRate();
  }
  else
  {
    m_PointTime = 1e9 / (defalutSampleRate.front() * 1000);
    lidarPtr->setPointTime(m_PointTime);
  }

  // printf("LiDAR get device info finished, Elapsed time %u ms\n", getms() - t);

  if (hasScanFrequencyCtrl(devinfo.model) || 
    ((isTOFLidar(m_LidarType)) && !m_SingleChannel) || 
      isNetTOFLidar(m_LidarType))
  {
    checkScanFrequency();
  }

  if (isSupportHeartBeat(devinfo.model))
  {
    ret &= checkHeartBeat();

    if (!ret)
    {
      fprintf(stderr, "Failed to Set HeartBeat[%d].\n", m_SupportHearBeat);
    }
  }

  if (hasZeroAngle(devinfo.model))
  {
    ret &= checkCalibrationAngle(serial_number);
  }

  return ret;
}

/*-------------------------------------------------------------
                    handleSingleChannelDevice
-------------------------------------------------------------*/
void CYdLidar::handleSingleChannelDevice()
{
  if (!lidarPtr || !lidarPtr->getSingleChannel())
  {
    return;
  }

  device_info devinfo;
  memset(&devinfo, 0, sizeof(device_info));

  result_t op_result = lidarPtr->getDeviceInfo(devinfo);
  if (!IS_OK(op_result))
  {
    return;
  }

  if (printfVersionInfo(devinfo, m_SerialPort, m_SerialBaudrate))
  {
    m_parsingCompleted = true;
    m_LidarVersion.hardware = devinfo.hardware_version;
    m_LidarVersion.soft_major = Major;
    m_LidarVersion.soft_minor = Minjor / 10;
    m_LidarVersion.soft_patch = Minjor % 10;
    memcpy(&m_LidarVersion.sn[0], &devinfo.serialnum[0], 16);
  }

  lidar_model = devinfo.model;
  // defalutSampleRate = getDefaultSampleRate(devinfo.model);

  printf("[YDLIDAR INFO] Single Channel Current Sampling Rate: %dK\n", m_SampleRate);
  return;
}

/*-------------------------------------------------------------
                    checkSampleRate
-------------------------------------------------------------*/
void CYdLidar::checkSampleRate()
{
  sampling_rate _rate = {0};
  int sr = 0;
  int try_count = 0;
  m_FixedSize = 1440;
  result_t ans = lidarPtr->getSamplingRate(_rate);

  if (IS_OK(ans))
  {
    printf("[YDLIDAR] Get Origin Sample Rate: %uK\n", _rate.rate);
    if (!isTOFLidarByModel(lidar_model))
    {
      //非TG系列雷达获取采样率码转成采样率值
      sr = ConvertUserToLidarSmaple(lidar_model, m_SampleRate, _rate.rate);

      //非TG系列雷达通过设备信息获取
      while (sr != _rate.rate)
      {
        ans = lidarPtr->setSamplingRate(_rate);
        try_count++;

        if (try_count > 3)
        {
          break;
        }
      }

      sr = ConvertLidarToUserSmaple(lidar_model, _rate.rate);
    }
    else
    {
      //TG系列雷达直接获取采样率值
      sr = ConvertLidarToUserSmaple(lidar_model, _rate.rate);
    }

    m_SampleRate = sr;
    defalutSampleRate.clear();
    defalutSampleRate.push_back(m_SampleRate);
    printf("[YDLIDAR] Get Sample Rate: %dK\n", m_SampleRate);
  }
}

/*-------------------------------------------------------------
                        checkScanFrequency
-------------------------------------------------------------*/
bool CYdLidar::checkScanFrequency()
{
  float frequency = 7.4f;
  scan_frequency _scan_frequency;
  float hz = 0.f;
  result_t ans = RESULT_FAIL;

  if (isSupportScanFrequency(lidar_model, m_ScanFrequency))
  {
    m_ScanFrequency += frequencyOffset;
    ans = lidarPtr->getScanFrequency(_scan_frequency);

    if (IS_OK(ans))
    {
      frequency = _scan_frequency.frequency / 100.f;
      hz = m_ScanFrequency - frequency;

      if (hz > 0)
      {
        while (hz > 0.95)
        {
          lidarPtr->setScanFrequencyAdd(_scan_frequency);
          hz = hz - 1.0;
        }

        while (hz > 0.09)
        {
          lidarPtr->setScanFrequencyAddMic(_scan_frequency);
          hz = hz - 0.1;
        }

        frequency = _scan_frequency.frequency / 100.0f;
      }
      else
      {
        while (hz < -0.95)
        {
          lidarPtr->setScanFrequencyDis(_scan_frequency);
          hz = hz + 1.0;
        }

        while (hz < -0.09)
        {
          lidarPtr->setScanFrequencyDisMic(_scan_frequency);
          hz = hz + 0.1;
        }

        frequency = _scan_frequency.frequency / 100.0f;
      }
    }
  }
  else
  {
    m_ScanFrequency += frequencyOffset;
    fprintf(stderr, "current scan frequency[%f] is out of range.",
            m_ScanFrequency - frequencyOffset);
  }

  ans = lidarPtr->getScanFrequency(_scan_frequency);

  if (IS_OK(ans))
  {
    frequency = _scan_frequency.frequency / 100.0f;
    m_ScanFrequency = frequency;
  }

  //   if( (lidar_model  == DriverInterface::YDLIDAR_TG15)
  //       || (lidar_model  == DriverInterface::YDLIDAR_TG30)
  //       || (lidar_model  == DriverInterface::YDLIDAR_TG50) )
  //   {
  //       m_SampleRate = m_SampleRatebyD1;
  //   }

  m_ScanFrequency -= frequencyOffset;
  m_FixedSize = m_SampleRate * 1000 / (m_ScanFrequency - 0.1);
  printf("[YDLIDAR INFO] Current Scan Frequency: %fHz\n", m_ScanFrequency);
  return true;
}

bool CYdLidar::checkHeartBeat()
{
  if (!m_SupportHearBeat)
  {
    lidarPtr->setHeartBeat(false);
    return true;
  }

  bool ret = false;
  scan_heart_beat beat;
  int retry = 0;

  do
  {
    result_t ans = lidarPtr->setScanHeartbeat(beat);

    if (IS_OK(ans) && !beat.enable)
    {
      ret = true;
      break;
    }

    retry++;
  } while (retry < 4);

  lidarPtr->setHeartBeat(m_SupportHearBeat);
  return ret;
}

/*-------------------------------------------------------------
                        checkCalibrationAngle
-------------------------------------------------------------*/
bool CYdLidar::checkCalibrationAngle(const std::string &serialNumber)
{
  bool ret = false;
  m_AngleOffset = 0.0;
  result_t ans = RESULT_FAIL;
  offset_angle angle;
  int retry = 0;
  m_isAngleOffsetCorrected = false;

  while (retry < 2)
  {
    ans = lidarPtr->getZeroOffsetAngle(angle);

    if (IS_OK(ans))
    {
      if (angle.angle > zero_offset_angle_scale * 360 ||
          angle.angle < -zero_offset_angle_scale * 360)
      {
        ans = lidarPtr->getZeroOffsetAngle(angle);

        if (!IS_OK(ans))
        {
          retry++;
          continue;
        }
      }

      m_isAngleOffsetCorrected = (angle.angle != 180 * zero_offset_angle_scale);
      m_AngleOffset = angle.angle / zero_offset_angle_scale;
      ret = true;
      printf("[YDLIDAR INFO] Successfully obtained the %s offset angle[%f] from the lidar[%s]\n", m_isAngleOffsetCorrected ? "corrected" : "uncorrrected", m_AngleOffset,
             serialNumber.c_str());
      return ret;
    }

    retry++;
  }

  printf("[YDLIDAR INFO] Current %s AngleOffset : %f°\n",
         m_isAngleOffsetCorrected ? "corrected" : "uncorrrected", m_AngleOffset);
  return ret;
}

/*-------------------------------------------------------------
            checkCOMMs
-------------------------------------------------------------*/
bool CYdLidar::checkCOMMs()
{
  if (!lidarPtr)
  {
    printf("YDLidar SDK initializing\n");

    // create the driver instance
    if (isNetTOFLidar(m_LidarType))
    {
      lidarPtr = new ydlidar::ETLidarDriver(); // T15
    }
    else if (isGS1Lidar(m_LidarType))
    {
      //GS1
      lidarPtr = new ydlidar::GS1LidarDriver();
    }
    else if (isGS2Lidar(m_LidarType))
    {
      //GS2
      lidarPtr = new ydlidar::GS2LidarDriver();
    }
    else
    { 
      //通用雷达
      lidarPtr = new ydlidar::YDlidarDriver(m_DeviceType);
    }

    if (!lidarPtr)
    {
      fprintf(stderr, "Create Driver fail\n");
      return false;
    }

    printf("YDLidar SDK has been initialized\n");
    printf("[YDLIDAR]:SDK Version: %s\n", lidarPtr->getSDKVersion().c_str());
    fflush(stdout);
    lidarPtr->setSupportMotorDtrCtrl(m_SupportMotorDtrCtrl);
  }

  if (lidarPtr->isconnected())
  {
    return true;
  }

  // Is it COMX, X>4? ->  "\\.\COMX"
  if (m_SerialPort.size() >= 3)
  {
    if (tolower(m_SerialPort[0]) == 'c' && tolower(m_SerialPort[1]) == 'o' &&
        tolower(m_SerialPort[2]) == 'm')
    {
      // Need to add "\\.\"?
      if (m_SerialPort.size() > 4 || m_SerialPort[3] > '4')
      {
        m_SerialPort = std::string("\\\\.\\") + m_SerialPort;
      }
    }
  }

  // make connection...
  result_t op_result = lidarPtr->connect(m_SerialPort.c_str(), m_SerialBaudrate);

  if (!IS_OK(op_result))
  {
    if (isNetTOFLidar(m_LidarType))
    {
      fprintf(stderr,
              "[CYdLidar] Error, cannot bind to the specified IP Address[%s]\n",
              m_SerialPort.c_str());
    }
    else
    {
      fprintf(stderr,
              "[CYdLidar] Error, cannot bind to the specified %s[%s] and %s[%d]\n",
              m_DeviceType != YDLIDAR_TYPE_SERIAL ? "IP Adddress" : "serial port",
              m_SerialPort.c_str(), m_DeviceType != YDLIDAR_TYPE_SERIAL ? "network port" : "baudrate", m_SerialBaudrate);
    }

    return false;
  }

  printf("LiDAR successfully connected\n");
  lidarPtr->setSingleChannel(m_SingleChannel);
  lidarPtr->setLidarType(m_LidarType);

  return true;
}

/*-------------------------------------------------------------
                        checkStatus
-------------------------------------------------------------*/
bool CYdLidar::checkStatus()
{
  getDeviceHealth();

  getDeviceInfo();

  return true;
}

/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
bool CYdLidar::checkHardware()
{
  if (!lidarPtr)
  {
    return false;
  }

  if (isScanning && lidarPtr->isscanning())
  {
    return true;
  }

  return false;
}

namespace ydlidar
{
  void os_init()
  {
    ydlidar::core::base::init();
  }

  bool os_isOk()
  {
    return ydlidar::core::base::ok();
  }

  void os_shutdown()
  {
    ydlidar::core::base::shutdown();
  }

  std::map<std::string, std::string> lidarPortList()
  {
    return ydlidar::YDlidarDriver::lidarPortList();
  }

}
