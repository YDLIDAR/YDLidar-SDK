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
#include "core/common/ydlidar_protocol.h"
#include "YDlidarDriver.h"
#include "ETLidarDriver.h"
#include "GSLidarDriver.h"
#include "SDMLidarDriver.h"
#include "DTSLidarDriver.h"
#include "TiaLidarDriver.h"

using namespace std;
using namespace impl;
using namespace ydlidar::core;
using namespace ydlidar::core::common;
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
  m_ScanFrequency = 10;
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
  {
    int sr = *(int*)(optval);
    m_SampleRate = sr;
    break;
  }

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
  {
    int sr = m_SampleRate;
    memcpy(optval, &sr, optlen);
    break;
  }

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
  if (!checkConnect())
  {
    error("Error initializing YDLIDAR check Comms.");
    return false;
  }

  if (!checkStatus())
  {
    error("Error initializing YDLIDAR check status under [%s] and [%d].",
            m_SerialPort.c_str(), m_SerialBaudrate);
    return false;
  }

  info("Lidar init success, Elapsed time [%u]ms", getms() - t);
  return true;
}

/*-------------------------------------------------------------
                        initialize
-------------------------------------------------------------*/
void CYdLidar::GetLidarVersion(LidarVersion &lv)
{
  memcpy(&lv, &m_LidarVersion, sizeof(LidarVersion));

  std::string sn;
  for (int i = 0; i < SDK_SNLEN; i++)
    sn += char(lv.sn[i] + 48);
  info("Lidar version\n"
        "Firmware version: %u.%u.%u\n"
        "Hardware version: %u\n"
        "Serial: %s",
        lv.soft_major,
        lv.soft_minor,
        lv.soft_patch,
        lv.hardware,
        sn.c_str());
}

/*-------------------------------------------------------------
                        turnOn
-------------------------------------------------------------*/
bool CYdLidar::turnOn()
{
  if (lidarPtr->isscanning())
    return true;

  uint32_t t = getms();
  //启动扫描
  result_t ret = lidarPtr->startScan();
  if (!IS_OK(ret))
  {
    ret = lidarPtr->startScan();
    if (!IS_OK(ret))
    {
      lidarPtr->stop();
      error("Failed to start scan mode %d", ret);
      return false;
    }
  }
  info("Successed to start scan mode, Elapsed time %u ms", getms() - t);

  t = getms();
  //计算采样率
  if (!checkLidarAbnormal())
  {
    lidarPtr->stop();
    error("Failed to turn on the Lidar, because the lidar is [%s].",
            DriverInterface::DescribeDriverError(lidarPtr->getDriverError()));
    return false;
  }
  info("Successed to check the lidar, Elapsed time %u ms", getms() - t);

  m_field_of_view = 360.f;
  //网络TOF雷达需要设置视场角
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
  lidarPtr->setAutoReconnect(m_AutoReconnect);
  info("Now lidar is scanning...");

  lastStamp = 0;
  //重置错误
  lidarPtr->setDriverError(NoError);
  return true;
}

bool CYdLidar::isScanning() const
{
  return lidarPtr && lidarPtr->isscanning();
}

bool CYdLidar::doProcessSimple(LaserScan &outscan)
{
  //判断是否已启动扫描
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
  result_t op_result = lidarPtr->grabScanData(global_nodes, count, 1000);
  uint64_t tim_scan_end = getTime();
  uint64_t endTs = tim_scan_end;
  uint64_t sys_scan_time = tim_scan_end - tim_scan_start; //获取一圈数据所花费的时间
  outscan.points.clear();

  // Fill in scan data:
  if (IS_OK(op_result) && count)
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
    tim_scan_end -= global_nodes[0].delayTime;
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
    LaserDebug debug = {0};

    memset(&debug, 0, sizeof(debug));
    outscan.config.min_angle = math::from_degrees(m_MinAngle);
    outscan.config.max_angle = math::from_degrees(m_MaxAngle);
    //将当前末点和上一圈末点采集时间差作为采集时长
    if (lastStamp > 0 && global_nodes[count - 1].stamp > 0)
      outscan.config.scan_time = double(global_nodes[count - 1].stamp - lastStamp) / 1e9;
    else
      outscan.config.scan_time = 0;
    lastStamp = global_nodes[count - 1].stamp;
    //计算时间增量
    if (!ISZERO(outscan.config.scan_time))
      outscan.config.time_increment = outscan.config.scan_time / count;
    else
      outscan.config.time_increment = .0f;
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

    //如果使用固定分辨率
    if (m_FixedResolution)
    {
      if (!isGSLidar(m_LidarType))
        all_node_count = m_FixedSize;
    }

    if (isGSLidar(m_LidarType))
      outscan.config.angle_increment = math::from_degrees(0.4); //GS雷达暂时固定角分辨率为0.4度
    else
      outscan.config.angle_increment = math::from_degrees(m_field_of_view) /
        (all_node_count - 1);

    float range = 0.0;
    float intensity = 0.0;
    float angle = 0.0;
    debug.maxIndex = 0;

    // printf("AngleOffset %f\n", m_AngleOffset);

    //遍历一圈点
    for (int i = 0; i < count; i++)
    {
      const node_info& node = global_nodes[i];

      // printf("%lu a:%.01f d:%u\n", 
      //   i, float(node.angle) / 128.0f, node.dist);

      if (isNetTOFLidar(m_LidarType))
      {
        angle = static_cast<float>(global_nodes[i].angle / 100.0f) +
                m_AngleOffset;
      }
      else
      {
        angle = static_cast<float>((global_nodes[i].angle >>
                                    LIDAR_RESP_ANGLE_SHIFT) /
                                   64.0f) +
                m_AngleOffset;
      }

      if (isOctaveLidar(lidar_model) ||
          isOldVersionTOFLidar(lidar_model, Major, Minjor))
      {
        range = static_cast<float>(global_nodes[i].dist / 2000.f);
      }
      else if (isR3Lidar(lidar_model))
      {
        range = static_cast<float>(global_nodes[i].dist / 40000.f);
      }
      else
      {
        if (isTOFLidar(m_LidarType) || 
          isNetTOFLidar(m_LidarType) ||
          isGSLidar(m_LidarType) ||
          isSDMLidar(m_LidarType) ||
          isDTSLidar(m_LidarType))
        {
          range = static_cast<float>(global_nodes[i].dist / 1000.f);
        }
        else
        {
          range = static_cast<float>(global_nodes[i].dist / 4000.f);
        }
      }

      intensity = static_cast<float>(global_nodes[i].qual);

      angle = math::from_degrees(angle);

      if (global_nodes[i].scanFreq != 0)
      {
        scanfrequency = global_nodes[i].scanFreq / 10.0;

        if (isTOFLidar(m_LidarType)) //TOF雷达转速偏移3Hz
        {
          if (!isOldVersionTOFLidar(lidar_model, Major, Minjor))
          {
            scanfrequency = global_nodes[i].scanFreq / 10.0 + 3.0;
          }
        }
        else if (isTEALidar(lidar_model) ||
          isGSLidar(m_LidarType) ||
          isTIALidar(m_LidarType)) //TEA雷达转速范围10~30，无缩放
        {
          scanfrequency = global_nodes[i].scanFreq;
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
      //   i, range, angle*180.0/M_PI, node.sync);

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
      if (global_nodes[i].error)
      {
        debug.maxIndex = 255;
      }
    } //end for (int i = 0; i < count; i++)

    outscan.size = outscan.points.size(); //保留原点云数

    if (m_FixedResolution)
    {
      if (count > all_node_count)
      {
        //如果点过多则直接删除多余的点并打印警告
        warn("[YDLIDAR]: Real point count %lu > fixed point count %d", count, all_node_count);
      	outscan.points.resize(all_node_count);
      }
      else
      {
        //如果点过少则添加末点
        LaserPoint p = outscan.points.back();
	while (outscan.points.size() < all_node_count)
          outscan.points.push_back(p);
      }
    }

    //解析V2协议雷达扫描数据中ct信息中的设备信息
    // getDeviceInfoByPackage(debug);
    //重新计算采样率
    resample(scanfrequency, count, tim_scan_end, tim_scan_start);

    outscan.scanFreq = scanfrequency;
    outscan.sampleRate = m_SampleRate;

    return true;
  }
  else
  {
    error("[YDLIDAR]: %d %s\n",
      op_result,
      DriverInterface::DescribeDriverError(lidarPtr->getDriverError()));

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
    if (lidarPtr->isscanning())
      info("Now lidar scanning has stopped!");
    lidarPtr->stop();
  }

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
  }
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
        error("Device is not open!");
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

void CYdLidar::setBottomPriority(bool yes)
{
  m_Bottom = yes;
}

bool CYdLidar::getDeviceInfo(device_info& di, int type)
{
  if (lidarPtr)
    return lidarPtr->getDeviceInfoEx(di, type);

  return false;
}

bool CYdLidar::getDeviceInfo(std::vector<device_info_ex>& dis)
{
  if (lidarPtr)
    return IS_OK(lidarPtr->getDeviceInfo(dis));
  return false;
}

void CYdLidar::setAutoIntensity(bool yes)
{
  m_AutoIntensity = yes;
}

bool CYdLidar::getPitchAngle(float& pitch)
{
    if (lidarPtr)
      return lidarPtr->getPitchAngle(pitch);
    return false;
}

bool CYdLidar::ota()
{
  if (lidarPtr)
    return lidarPtr->ota();
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

bool CYdLidar::getDeviceInfoByPackage(const LaserDebug &debug)
{
  if (!lidarPtr)
    return false;

  device_info di;
  memset(&di, 0, DEVICEINFOSIZE);

  if (parseLaserDebugInfo(debug, di))
  {
    if (printfDeviceInfo(di, EPT_Module))
    {
      std::string serial_number;
      Major = (uint8_t)(di.firmware_version >> 8);
      Minjor = (uint8_t)(di.firmware_version & 0xff);
      m_LidarVersion.hardware = di.hardware_version;
      m_LidarVersion.soft_major = Major;
      m_LidarVersion.soft_minor = Minjor / 10;
      m_LidarVersion.soft_patch = Minjor % 10;
      memcpy(&m_LidarVersion.sn[0], &di.serialnum[0], 16);

      for (int i = 0; i < 16; i++)
      {
        serial_number += std::to_string(di.serialnum[i] & 0xff);
      }

      m_SerialNumber = serial_number;
      //设置模组标记
      lidarPtr->setHasDeviceInfo(lidarPtr->getHasDeviceInfo() | EPT_Module);

      return true;
    }
  }

  return false;
}

/*-------------------------------------------------------------
                    resample
-------------------------------------------------------------*/
void CYdLidar::resample(
  int frequency, int count, 
  uint64_t tim_scan_end,
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

//检查异常
bool CYdLidar::checkLidarAbnormal()
{
  size_t count = ydlidar::YDlidarDriver::MAX_SCAN_NODES;
  int checkCount = 0; //检查次数

  if (m_AbnormalCheckCount < 2)
    m_AbnormalCheckCount = 2;

  result_t ret = RESULT_FAIL;
  std::vector<int> data;
  
  while (checkCount < m_AbnormalCheckCount)
  {
    // printf("checkLidarAbnormal %d\n", checkCount);

    // Ensure that the voltage is insufficient or the motor resistance is high, 
    //causing an abnormality.
    if (checkCount)
      delay(500);

    float scan_time = 0.0;
    uint64_t start_time = 0;
    uint64_t end_time = 0;
    int checkOneCount = 0;
    ret = RESULT_OK;

    //单双通雷达，计算采样率
    while (checkOneCount < 5 &&
          //  (scan_time < 0.05 || !lidarPtr->getSingleChannel()) &&
           IS_OK(ret))
    {
      checkOneCount ++;
      start_time = getTime();
      count = ydlidar::YDlidarDriver::MAX_SCAN_NODES;
      ret = lidarPtr->grabScanData(global_nodes, count);
      end_time = getTime();
      scan_time = 1.0 * static_cast<int64_t>(end_time - start_time) / 1e9;
      
      if (IS_OK(ret))
      {
        // 获取CT信息
        if (!(lidarPtr->getHasDeviceInfo() & EPT_Module))
        {
          // printf("Get module device info\n");
          LaserDebug debug = {0};
          for (int i = 0; i < count; ++i)
          {
            parsePackageNode(global_nodes[i], debug);
            if (global_nodes[i].error)
              debug.maxIndex = 255;
          }
          // 解析V2协议雷达扫描数据中ct信息中的设备信息
          getDeviceInfoByPackage(debug);
        }

        if (isNetTOFLidar(m_LidarType))
        {
          return IS_OK(ret);
        }

        data.push_back(count);
        if (std::abs(static_cast<int>(data.front() - count)) > 10)
          data.erase(data.begin());

        if (calcSampleRate(count, scan_time))
        {
          // 双通雷达计算完采样率即可返回
          if (!lidarPtr->getSingleChannel())
          {
            return IS_OK(ret);
          }
        }
        else
        {
          //计算采样率
          if (scan_time > 0.05 && scan_time < 0.5)
          {
            m_SampleRate = static_cast<int>((count / scan_time + 500) / 1000);
            m_PointTime = 1e9 / (m_SampleRate * 1000);
            lidarPtr->setPointTime(m_PointTime);
          }
        }
      }
    }

    //单通雷达计算固定分辨率时的一圈点数
    if (lidarPtr->getSingleChannel() &&
      data.size() > 1)
    {
      int total = accumulate(data.begin(), data.end(), 0);
      int mean = total / data.size(); // mean value
      m_FixedSize = (static_cast<int>((mean + 5) / 10)) * 10;
      info("Single Fixed Size: %d", m_FixedSize);
      info("Sample Rate: %.02fK", m_SampleRate);
      return true;
    }

    checkCount ++;
  }

  return IS_OK(ret);
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
                    calcSampleRate
-------------------------------------------------------------*/
bool CYdLidar::calcSampleRate(int count, double scan_time)
{
  if (count < 1)
    return false;

  // 1、如果雷达支持直接获取转速，则使用获取的转速计算采样率，此时将默认采样率值置为该值
  // 2、如果有设置默认采样率，判断当前雷达型号对应的默认采样率值的个数，个数为1，
  //直接使用该采样率，个数不为1则根据实时采样率进行匹配
  // 3、如果没有设置默认采样率，则使用实时采样率
  float sr = 0;
  bool ret = false;

  if (global_nodes[0].scanFreq != 0)
  {
    //如果解析到转速信息，根据转速计算采样率
    double scanfrequency = global_nodes[0].scanFreq / 10.0;
    if (isTOFLidar(m_LidarType) &&
        !isOldVersionTOFLidar(lidar_model, Major, Minjor))
    {
      scanfrequency = global_nodes[0].scanFreq / 10.0 + 3.0;
    }
    sr = static_cast<int>((count * scanfrequency + 500) / 1000);

    if (isSDMLidar(m_LidarType))
    {
      defalutSampleRate.clear(); //SDM雷达通过协议获取转速计算采样率
      sr = float(count * global_nodes[0].scanFreq) / 1000;
    }
  }
  else
  {
    //如果没有解析到转速信息，根据时间计算采样率
    if (scan_time > 0.04 && scan_time < 0.4)
    {
      sr = static_cast<int>((count / scan_time + 500) / 1000);
    }
  }

  size_t size = defalutSampleRate.size();
  if (size)
  {
    if (size == 1)
    {
      sr = defalutSampleRate.front();
      ret = true;
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
    }
  }
  else
  {
    if (sr > 0)
      SampleRateMap[sr * 1000] ++; //放大1000倍存入
    if (isValidSampleRate(SampleRateMap))
      ret = true;
  }

  if (ret)
  {
    m_SampleRate = sr;
    m_PointTime = 1e9 / (m_SampleRate * 1000);
    lidarPtr->setPointTime(m_PointTime);
    if (!m_SingleChannel)
      //通过增加采样率值来增加点数，以免计算出的点数不够导致缺点
      m_FixedSize = (m_SampleRate + 0.3) * 1000 / m_ScanFrequency;
    
    info("Scan Frequency: %.02fHz", m_ScanFrequency);
    if (!isSDMLidar(m_LidarType)) //非SDM雷达才打印Fixed Size
      info("Fixed Size: %d", m_FixedSize);
    info("Sample Rate: %.02fK", m_SampleRate);
  }

  return ret;
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

  result_t ret;
  device_health healthinfo;
  memset(&healthinfo, 0, sizeof(device_health));
  ret = lidarPtr->getHealth(healthinfo,
    DriverInterface::DEFAULT_TIMEOUT / 2);

  if (IS_OK(ret))
  {
    info("Lidar running correctly! The health status %s",
      healthinfo.status == 0 ? "good" : "bad");
    if (healthinfo.status == 2)
    {
      error("Error, Lidar internal error[0x%X] detected. "
            "Please reboot the device to retry.", 
        healthinfo.error_code);
      return false;
    }
    else
    {
      return true;
    }
  }
  else
  {
    error("Error, cannot retrieve Lidar health code %d", ret);
    return false;
  }
}

/*-------------------------------------------------------------
                    getDeviceInfo
-------------------------------------------------------------*/
bool CYdLidar::getDeviceInfo()
{
  if (!lidarPtr)
    return false;

  bool ret = false;
  device_info di;
  memset(&di, 0, sizeof(device_info));
  result_t op_result = lidarPtr->getDeviceInfo(di,
    DriverInterface::DEFAULT_TIMEOUT / 2);
  if (!IS_OK(op_result))
  {
    error("Fail to get baseplate device information!");
    return false;
  }

  if (!isSupportLidar(di.model))
  {
    error("Current SDK does not support current lidar model [%s]",
      lidarModelToString(di.model).c_str());
    return false;
  }

  // check Lidar Type Config
  if (isTOFLidarByModel(di.model))
  {
    if (!isTOFLidar(m_LidarType))
    {
      error("Incorrect Lidar Type setting...");
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
    //   error("Incorrect Lidar Type setting, Reset Type to %d...\n",
    //           TYPE_TRIANGLE);
    //   m_LidarType = TYPE_TRIANGLE;
    //   lidarPtr->setLidarType(m_LidarType);
    // }
  }

  frequencyOffset = 0.4;
  lidar_model = di.model;
  info("Current Lidar Model Code %d", lidar_model);
  // bool intensity = hasIntensity(di.model);
    // intensity = m_Intensity;
      // lidarPtr->setIntensities(intensity);
  //  printf("Set Lidar Intensity Bit count %d\n", m_IntensityBit);
  // lidarPtr->setIntensityBit(m_IntensityBit);
  defalutSampleRate = getDefaultSampleRate(di.model);
  // printf("getDefaultSampleRate %d\n", defalutSampleRate.size());

  std::string serial_number;
  ret = true;

  if (printfDeviceInfo(di, EPT_Base))
  {
    Major = (uint8_t)(di.firmware_version >> 8);
    Minjor = (uint8_t)(di.firmware_version & 0xff);
    m_LidarVersion.hardware = di.hardware_version;
    m_LidarVersion.soft_major = Major;
    if (isGSLidar(m_LidarType))
    {
      m_LidarVersion.soft_minor = Minjor;
      m_LidarVersion.soft_patch = 0;
    }
    else
    {
      m_LidarVersion.soft_minor = Minjor / 10;
      m_LidarVersion.soft_patch = Minjor % 10;
    }
    memcpy(&m_LidarVersion.sn[0], &di.serialnum[0], SDK_SNLEN);

    for (int i = 0; i < SDK_SNLEN; i++)
    {
      serial_number += std::to_string(di.serialnum[i] & 0xff);
    }

    m_SerialNumber = serial_number;
    zero_offset_angle_scale = lidarZeroOffsetAngleScale(
      di.model, di.firmware_version >> 8, di.firmware_version & 0x00ff);
  }

  // uint32_t t = getms();
  if (hasSampleRate(di.model))
  {
    checkSampleRate();
  }
  else
  {
    if (defalutSampleRate.size())
    {
      m_PointTime = 1e9 / (defalutSampleRate.front() * 1000);
      lidarPtr->setPointTime(m_PointTime);
    }
  }

  // printf("LIDAR get device info finished, Elapsed time %u ms\n", getms() - t);
  //检查转速
  if (hasScanFrequencyCtrl(di.model) || 
    ((isTOFLidar(m_LidarType)) && !m_SingleChannel) || 
      isNetTOFLidar(m_LidarType))
  {
    checkScanFrequency();
  }

  if (isSupportHeartBeat(di.model))
  {
    ret &= checkHeartBeat();

    if (!ret)
    {
      fprintf(stderr, "Failed to Set HeartBeat[%d].\n", m_SupportHearBeat);
    }
  }

  if (hasZeroAngle(di.model))
  {
    ret &= checkCalibrationAngle(serial_number);
  }

  return ret;
}

void CYdLidar::handleSingleChannelDevice()
{
  if (!lidarPtr ||
      lidarPtr->getBottom() ||
      !lidarPtr->getSingleChannel())
  {
    return;
  }

  //获取模组设备信息
  //1、单通雷达需要从CT信息中获取模组设备信息
  //2、双通雷达需要获取启动时抛出的模组设备信息

  device_info di;
  memset(&di, 0, sizeof(device_info));

  result_t op_result = lidarPtr->getDeviceInfo(di);
  if (!IS_OK(op_result))
  {
    return;
  }

  if (printfDeviceInfo(di, EPT_Module))
  {
    m_LidarVersion.hardware = di.hardware_version;
    m_LidarVersion.soft_major = Major;
    m_LidarVersion.soft_minor = Minjor / 10;
    m_LidarVersion.soft_patch = Minjor % 10;
    memcpy(&m_LidarVersion.sn[0], &di.serialnum[0], SDK_SNLEN);
  }

  lidar_model = di.model;
  // defalutSampleRate = getDefaultSampleRate(devinfo.model);

  info("Single channel current sampling rate: %.02fK", m_SampleRate);
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
  result_t ret = lidarPtr->getSamplingRate(_rate);
  if (IS_OK(ret))
  {
    info("Origin sample rate code: %u", _rate.rate);
    if (!isTOFLidarByModel(lidar_model))
    {
      //非TG系列雷达获取采样率码转成采样率值
      sr = ConvertUserToLidarSmaple(lidar_model, m_SampleRate, _rate.rate);
      //非TG系列雷达通过设备信息获取
      while (sr != _rate.rate)
      {
        ret = lidarPtr->setSamplingRate(_rate);
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
    info("Current sample rate: %.02fK", m_SampleRate);
  }
}

bool CYdLidar::checkScanFrequency()
{
  float frequency = 7.4f;
  scan_frequency _scan_frequency;
  float hz = 0.f;
  result_t ans = RESULT_FAIL;

  if (isSupportScanFrequency(lidar_model, m_ScanFrequency))
  {
    //TODO: 此处为何要加上偏移量，待解释
    // m_ScanFrequency += frequencyOffset;
    ans = lidarPtr->getScanFrequency(_scan_frequency);
    if (IS_OK(ans))
    {
      frequency = _scan_frequency.frequency / 100.f;
      if (isTOFLidar(m_LidarType)) //TG雷达转速虚高0.4需要减去还原真实转速
        frequency -= 0.4;
      hz = m_ScanFrequency - frequency;
      info("Current scan frequency: %.02fHz", frequency);
      if (hz > 0)
      {
        //大调速
        while (hz > 0.95)
        {
          lidarPtr->setScanFrequencyAdd(_scan_frequency);
          hz -= 1.0;
        }
        //小调速
        while (hz > 0.09)
        {
          lidarPtr->setScanFrequencyAddMic(_scan_frequency);
          hz -= 0.1;
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
    // m_ScanFrequency += frequencyOffset;
    error("Current scan frequency[%f] is out of range.",
      m_ScanFrequency);
  }

  ans = lidarPtr->getScanFrequency(_scan_frequency);
  if (IS_OK(ans))
  {
    frequency = _scan_frequency.frequency / 100.0f;
    if (isTOFLidar(m_LidarType)) //TG雷达转速虚高0.4需要减去还原真实转速
        frequency -= 0.4;
    m_ScanFrequency = frequency;
  }

  //   if( (lidar_model  == DriverInterface::YDLIDAR_TG15)
  //       || (lidar_model  == DriverInterface::YDLIDAR_TG30)
  //       || (lidar_model  == DriverInterface::YDLIDAR_TG50) )
  //   {
  //       m_SampleRate = m_SampleRatebyD1;
  //   }

  // m_ScanFrequency -= frequencyOffset;
  m_FixedSize = m_SampleRate * 1000 / (m_ScanFrequency - 0.1);
  info("Current scan frequency: %.02fHz", m_ScanFrequency);
  // info("Fixed size: %d", m_FixedSize);
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
      info("Successfully obtained the %s offset angle[%f] from the lidar[%s]", 
        m_isAngleOffsetCorrected ? "corrected" : "uncorrrected", m_AngleOffset,
        serialNumber.c_str());
      return ret;
    }

    retry++;
  }

  info("Current %s AngleOffset : %f°",
    m_isAngleOffsetCorrected ? "corrected" : "uncorrrected", m_AngleOffset);
  return ret;
}

/*-------------------------------------------------------------
            checkConnect
-------------------------------------------------------------*/
bool CYdLidar::checkConnect()
{
  //如果雷达类型有变化则需要先删除旧对象
  if (lidarPtr && 
    lidarPtr->getLidarType() != m_LidarType)
  {
    delete lidarPtr;
    lidarPtr = nullptr;
  }
  //如果未创建对象
  if (!lidarPtr)
  {
    info("SDK initializing");

    //根据雷达类型创建对应的实例
    if (isNetTOFLidar(m_LidarType))
      lidarPtr = new ydlidar::ETLidarDriver(); //T15
    else if (isGSLidar(m_LidarType)) //GS
      lidarPtr = new ydlidar::GSLidarDriver(m_DeviceType);
    else if (isSDMLidar(m_LidarType)) //SDM
      lidarPtr = new ydlidar::SDMLidarDriver();
    else if (isDTSLidar(m_LidarType)) //SDM
      lidarPtr = new ydlidar::DTSLidarDriver();
    else if (isTIALidar(m_LidarType))
      lidarPtr = new ydlidar::TiaLidarDriver();
    else //通用雷达
      lidarPtr = new ydlidar::YDlidarDriver(m_DeviceType);

    if (!lidarPtr)
    {
      error("Create driver fail!");
      return false;
    }

    info("SDK has been initialized");
    info("SDK Version: %s", lidarPtr->getSDKVersion().c_str());
  }

  if (lidarPtr->isconnected())
  {
    return true;
  }

  //初始化
  lidarPtr->setSingleChannel(m_SingleChannel);
  lidarPtr->setLidarType(m_LidarType);
  lidarPtr->setScanFreq(m_ScanFrequency);
  lidarPtr->setSampleRate(m_SampleRate); //设置采样率
  lidarPtr->setSupportMotorDtrCtrl(m_SupportMotorDtrCtrl);
  lidarPtr->setBottom(m_Bottom);
  lidarPtr->setDebug(m_Debug);
  lidarPtr->setOtaName(otaName);
  lidarPtr->setOtaEncode(otaEncode);
  lidarPtr->setIntensities(m_Intensity);
  lidarPtr->setIntensityBit(m_IntensityBit);
  lidarPtr->setAutoIntensity(m_AutoIntensity);

  uint32_t t = getms();

  // Is it COMX, X>4? ->  "\\.\COMX"
  if (m_SerialPort.size() >= 3)
  {
    if (tolower(m_SerialPort[0]) == 'c' && 
        tolower(m_SerialPort[1]) == 'o' &&
        tolower(m_SerialPort[2]) == 'm')
    {
      // Need to add "\\.\"?
      if (m_SerialPort.size() > 4 || m_SerialPort[3] > '4')
      {
        m_SerialPort = std::string("\\\\.\\") + m_SerialPort;
      }
    }
  }
  //连接
  result_t op_result = lidarPtr->connect(m_SerialPort.c_str(), m_SerialBaudrate);
  if (!IS_OK(op_result))
  {
    if (isNetTOFLidar(m_LidarType))
    {
      error("Error, cannot bind to the specified IP Address[%s]",
        m_SerialPort.c_str());
    }
    else
    {
      error("Error, cannot bind to the specified [%s:%s] and [%s:%d]",
        m_DeviceType != YDLIDAR_TYPE_SERIAL ? "IP Address" : "serial port",
        m_SerialPort.c_str(), 
        m_DeviceType != YDLIDAR_TYPE_SERIAL ? "network port" : "baudrate", 
        m_SerialBaudrate);
    }

    return false;
  }

  info("Connect elapsed time %u ms", getms() - t);
  info("Lidar successfully connected [%s:%d]", 
    m_SerialPort.c_str(), m_SerialBaudrate);
  return true;
}

/*-------------------------------------------------------------
                        checkStatus
-------------------------------------------------------------*/
bool CYdLidar::checkStatus()
{
  uint32_t t = getms();
  getDeviceHealth();
  getDeviceInfo();
  info("Check status, Elapsed time %u ms", getms() - t);

  return true;
}

/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
bool CYdLidar::checkHardware()
{
  if (!lidarPtr)
    return false;

  return lidarPtr->isscanning();
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

//打印logo字符
void printLogo()
{
  info("__   ______  _     ___ ____    _    ____");
  info("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\");
  info(" \\ V /| | | | |    | || | | |/ _ \\ | |_) |");
  info("  | | | |_| | |___ | || |_| / ___ \\|  _ <");
  info("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\");
  info("");
}

}
