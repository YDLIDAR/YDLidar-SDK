/*****************************************************************************
*  EAI TOF LIDAR DRIVER                                                      *
*  Copyright (C) 2018-2020 EAI TEAM  chushuifurong618@eaibot.com.                 *
*                                                                            *
*  This file is part of EAI TOF LIDAR DRIVER.                                *
*                                                                            *
*  @file     ETLidarDriver.cpp                                               *
*  @brief    TOF LIDAR DRIVER                                                *
*  Details.                                                                  *
*                                                                            *
*  @author   Tony.Yang                                                       *
*  @email    chushuifurong618@eaibot.com                                     *
*  @version  1.0.0(版本号)                                                    *
*  @date     chushuifurong618@eaibot.com                                     *
*                                                                            *
*                                                                            *
*----------------------------------------------------------------------------*
*  Remark         : Description                                              *
*----------------------------------------------------------------------------*
*  Change History :                                                          *
*  <Date>     | <Version> | <Author>       | <Description>                   *
*----------------------------------------------------------------------------*
*  2018/08/09 | 1.0.0     | Tony.Yang      | Create file                     *
*----------------------------------------------------------------------------*
*                                                                            *
*****************************************************************************/
#if defined(_MSC_VER)
#define NOMINMAX
#endif

#include <stdio.h>
/*Socket Specific headers */
#include <errno.h>
#include <math.h>
#include <algorithm>
#include "ETLidarDriver.h"
#include "core/network/PassiveSocket.h"
#include "core/serial/common.h"
#include "core/math/angles.h"
#include "ydlidar_config.h"

using namespace impl;
using namespace ydlidar;
using namespace ydlidar::core;
using namespace ydlidar::core::network;
using namespace ydlidar::core::base;


/////////////////////////////////////////////////////////////////////////////////////////
// port defaults to 9000 if not provided.
ETLidarDriver::ETLidarDriver() :
  offset_len(0),
  port(9000),
  m_sampleRate(20000),
  m_force_update(false) {

  socket_cmd = new CActiveSocket(CSimpleSocket::SocketTypeTcp);
  socket_data = new CPassiveSocket(CSimpleSocket::SocketTypeUdp);
  socket_data->SetSocketType(CSimpleSocket::SocketTypeUdp);
  socket_cmd->SetConnectTimeout(DEFAULT_CONNECTION_TIMEOUT_SEC,
                                DEFAULT_CONNECTION_TIMEOUT_USEC);
  scan_node_buf = new node_info[MAX_SCAN_NODES];
  scan_node_count = 0;
  m_lastAngle = 0.f;
  m_currentAngle = 0.f;
  nodeIndex = 0;
  retryCount = 0;
  isAutoReconnect = true;
  isAutoconnting = false;;
  m_SupportMotorDtrCtrl = true;
  m_AbnormalCheckCount = 4;

  m_isScanning = false;
  m_isConnected = false;
  m_port = "192.168.0.11";
  m_baudrate = 8000;
  m_config.motor_rpm = 1200;
  m_config.laserScanFrequency = 50;
  m_config.correction_angle = 20640;
  m_config.correction_distance = 6144;
  m_SingleChannel = false;
  m_LidarType = TYPE_TOF_NET;
  m_PointTime = 1e9 / 20000;
  m_isValidDevice = true;

}

/////////////////////////////////////////////////////////////////////////////////////////
ETLidarDriver::~ETLidarDriver() {
  disconnect();
  isAutoReconnect = false;
  ScopedLocker data_lock(_data_lock);

  if (socket_data) {
    delete socket_data;
    socket_data = NULL;
  }

  ScopedLocker lock(_cmd_lock);

  if (socket_cmd) {
    delete socket_cmd;
    socket_cmd = NULL;
  }

  if (scan_node_buf) {
    delete[]  scan_node_buf;
    scan_node_buf = nullptr;
  }
}

void ETLidarDriver::updateScanCfg(const lidarConfig &config) {
  if (m_isConnected) {
    return;
  }

  m_force_update = true;
  m_user_config = config;

}

result_t ETLidarDriver::connect(const char *port_path, uint32_t baudrate) {
  m_port = port_path;
  m_baudrate = baudrate;
  m_isConnected = false;

  if (!configPortConnect(m_port.c_str(), port)) {
    setDriverError(NotOpenError);
    m_isValidDevice = false;
    return RESULT_FAIL;
  }

  m_isValidDevice = true;
  disConfigConnect();
  lidarConfig config;
  bool ret = getScanCfg(config);

  if (!ret) {
    return RESULT_FAIL;
  }

  if (config.dataRecvPort != baudrate) {
    m_baudrate = config.dataRecvPort;
//    if (!m_force_update) {
//      m_user_config = config;
//      m_user_config.dataRecvPort = baudrate;
//      m_force_update = true;
//    }
  }

  if (m_force_update) {
    setScanCfg(m_user_config);
  } else {
    m_user_config = config;
  }

  if (!dataPortConnect(m_config.deviceIp, m_config.dataRecvPort)) {
    stopMeasure();
    setDriverError(NotOpenError);
    return RESULT_FAIL;
  }

  m_isConnected = true;
  return RESULT_OK;
}

bool ETLidarDriver::isconnected() const {
  return m_isConnected;
}

bool ETLidarDriver::isscanning() const {
  return m_isScanning;
}

void ETLidarDriver::setIntensities(const bool &isintensities) {
  m_intensities = isintensities;
}

void ETLidarDriver::setAutoReconnect(const bool &enable) {
  isAutoReconnect = enable;
}

const char *ETLidarDriver::DescribeError(bool isTCP) {
  if (isTCP) {
    ScopedLocker lock(_cmd_lock);
    return socket_cmd != NULL ? socket_cmd->DescribeError() : "NO Socket";
  } else {
    ScopedLocker lock(_data_lock);
    return socket_data != NULL ? socket_data->DescribeError() : "NO Socket";
  }
}

bool ETLidarDriver::configPortConnect(const char *lidarIP, int tcpPort) {

  ScopedLocker lock(_cmd_lock);

  if (!socket_cmd) {
    return false;
  }

  if (!socket_cmd->IsSocketValid()) {
    if (!socket_cmd->Initialize()) {
      return false;
    }
  } else {
    return socket_cmd->IsSocketValid();
  }

  socket_cmd->SetNonblocking();

  if (!socket_cmd->Open(lidarIP, tcpPort)) {
    socket_cmd->Close();
    return false;
  }

  socket_cmd->SetSendTimeout(DEFAULT_TIMEOUT / 1000,
                             (DEFAULT_TIMEOUT % 1000) * 1000);
  socket_cmd->SetReceiveTimeout(DEFAULT_TIMEOUT / 1000,
                                (DEFAULT_TIMEOUT % 1000) * 1000);
  socket_cmd->SetBlocking();

  return socket_cmd->IsSocketValid();
}

void ETLidarDriver::disConfigConnect() {
  ScopedLocker lock(_cmd_lock);

  if (!socket_cmd) {
    return;
  }

  socket_cmd->Close();
}

void ETLidarDriver::disconnect() {
  isAutoReconnect = false;
  stop();
  ScopedLocker lock(_data_lock);

  if (!socket_data) {
    return;
  }

  socket_data->Close();
}

std::string ETLidarDriver::getSDKVersion() {
  return YDLIDAR_SDK_VERSION_STR;
}

result_t ETLidarDriver::getHealth(device_health &health, uint32_t timeout) {
  result_t ans = RESULT_OK;
  disableDataGrabbing();

  health.error_code = 0;
  health.status = 0;

  return ans;
}

result_t ETLidarDriver::getDeviceInfo(device_info &info, uint32_t timeout) {
  result_t ans = RESULT_OK;

  info.firmware_version = 0;
  info.hardware_version = 0;
  info.model = YDLIDAR_T15;

  return ans;
}

result_t ETLidarDriver::startScan(bool force, uint32_t timeout) {
  result_t ans;

  if (m_isScanning) {
    return RESULT_OK;
  }

  {
    bool ret = startMeasure();

    if (!ret) {
      startMeasure();
    }

    if (!ret) {
      return RESULT_FAIL;
    }

    ans = this->createThread();
    return ans;
  }

  return RESULT_OK;
}


result_t ETLidarDriver::stop() {
  if (isAutoconnting) {
    isAutoReconnect = false;
    m_isScanning = false;
  }

  disableDataGrabbing();

  if (m_isValidDevice) {
    bool ret = stopMeasure();

    if (!ret) {
      stopMeasure();
    }
  }

  return RESULT_OK;
}

result_t ETLidarDriver::createThread() {
  m_isScanning = true;
  _thread = CLASS_THREAD(ETLidarDriver, cacheScanData);

  if (_thread.getHandle() == 0) {
    m_isScanning = false;
    return RESULT_FAIL;
  }

  return RESULT_OK;
}


char *ETLidarDriver::configMessage(const char *descriptor, char *value) {
  char buf[100];
  char transDesc[32];
  char recvDesc[32];
  static char recvValue[32];

  strncpy(transDesc, descriptor, sizeof(transDesc));
  valLastName(transDesc);

  sprintf(buf, "%s=%s\n", transDesc, value);
  ScopedLocker lock(_cmd_lock);

  if (!socket_cmd) {
    return NULL;
  }

  socket_cmd->Send(reinterpret_cast<uint8_t *>(buf), strlen(buf));

  memset(buf, 0, sizeof(buf));

  if (socket_cmd->Select(0, 800000)) {
    socket_cmd->Receive(sizeof(buf), reinterpret_cast<uint8_t *>(buf));

    if (2 == sscanf(buf, "%[^=]=%[^=]", recvDesc, recvValue)) {
      if (!strcmp(transDesc, recvDesc)) {
        return recvValue;
      } else {
        return NULL;
      }
    } else {
      return NULL;
    }

  } else {
    return value;
  }

  return NULL;
}

bool ETLidarDriver::startMeasure() {
  bool ret;

  if (!configPortConnect(m_port.c_str(), port)) {
    return  false;
  }

  lidarConfig cfg;
  ret = (configMessage(valName(cfg.motor_en), (char *)configValue[1]) != NULL);
  ret &= (configMessage(valName(cfg.laser_en), (char *)configValue[1]) != NULL);
  disConfigConnect();
  return ret;
}

bool ETLidarDriver::stopMeasure() {
  if (!configPortConnect(m_port.c_str(), port)) {
    return  false;
  }

  bool ret ;
  lidarConfig cfg;
  ret = (configMessage(valName(cfg.motor_en), (char *)configValue[0]) != NULL);
  ret &= (configMessage(valName(cfg.laser_en), (char *)configValue[0]) != NULL);
  disConfigConnect();
  return ret;
}

lidarConfig ETLidarDriver::getFinishedScanCfg() {
  return m_config;
}

bool ETLidarDriver::getScanCfg(lidarConfig &config,
                               const std::string &ip_address) {
  bool ret = true;

  if (!ip_address.empty()) {
    m_port = ip_address;
  }

  lidarConfig cfg;

  if (!configPortConnect(m_port.c_str(), port)) {
    config = m_config;
    ret = false;
    return  ret;
  }

  char *result = configMessage(valName(cfg.laser_en));

  if (result != NULL) {
    cfg.laser_en = atoi(result);
  } else {
    ret &= false;
  }

  result = configMessage(valName(cfg.motor_en));

  if (result != NULL) {
    cfg.motor_en = atoi(result);
  } else {
    ret &= false;
  }

  result = configMessage(valName(cfg.motor_rpm));

  if (result != NULL) {
    cfg.motor_rpm = atoi(result);
  } else {
    ret &= false;
  }

  result = configMessage(valName(cfg.fov_start));

  if (result != NULL) {
    cfg.fov_start = atoi(result);
  } else {
    ret &= false;
  }

  result = configMessage(valName(cfg.fov_end));

  if (result != NULL) {
    cfg.fov_end = atoi(result);
  } else {
    ret &= false;
  }

  result = configMessage(valName(cfg.trans_sel));

  if (result != NULL) {
    cfg.trans_sel = atoi(result);
  } else {
    ret &= false;
  }

  result = configMessage(valName(cfg.dataRecvPort));

  if (result != NULL) {
    cfg.dataRecvPort = atoi(result);
  } else {
    ret &= false;
  }


  result = configMessage(valName(cfg.dhcp_en));

  if (result != NULL) {
    cfg.dhcp_en = atoi(result);
  } else {
    ret &= false;
  }

  result = configMessage(valName(cfg.dataRecvIp));

  if (result != NULL) {
    strcpy(cfg.dataRecvIp, result);
  } else {
    ret &= false;
  }

  result = configMessage(valName(cfg.deviceIp));

  if (result != NULL) {
    strcpy(cfg.deviceIp, result);
  } else {
    ret &= false;
  }

  result = configMessage(valName(cfg.deviceNetmask));

  if (result != NULL) {
    strcpy(cfg.deviceNetmask, result);
  } else {
    ret &= false;
  }


  result = configMessage(valName(cfg.deviceGatewayIp));

  if (result != NULL) {
    strcpy(cfg.deviceGatewayIp, result);
  } else {
    ret &= false;
  }

  result = configMessage(valName(cfg.laserScanFrequency));

  if (result != NULL) {
    cfg.laserScanFrequency = atoi(result);
    m_sampleRate = 1000 / cfg.laserScanFrequency * 1000;
  } else {
    cfg.laserScanFrequency = 50;
    m_sampleRate = 1000 / cfg.laserScanFrequency * 1000;
  }

  result = configMessage(valName(cfg.correction_angle));

  if (result != NULL) {
    cfg.correction_angle = atoi(result);
  } else {
    cfg.correction_angle = 20640;
  }

  result = configMessage(valName(cfg.correction_distance));

  if (result != NULL) {
    cfg.correction_distance = atoi(result);
  } else {
    cfg.correction_distance = 6144;

  }

  if (ret) {
    m_config = cfg;
    config = cfg;
    m_sampleRate = static_cast<int>(1e9 / m_sampleRate);
  }

  disConfigConnect();
  return ret;
}


void ETLidarDriver::setScanCfg(const lidarConfig &config) {
  char str[32];

  if (!configPortConnect(m_port.c_str(), port)) {
    return ;
  }

  char *result = NULL;

  if (m_config.motor_rpm != config.motor_rpm) {
    _itoa(config.motor_rpm, str, 10);
    result = configMessage(valName(config.motor_rpm), str);

    if (result != NULL) {
      m_config.motor_rpm = atoi(result);
    }
  }


  if (m_config.fov_start != config.fov_start) {
    _itoa(config.fov_start, str, 10);
    result = configMessage(valName(config.fov_start), str);

    if (result != NULL) {
      m_config.fov_start = atoi(result);
    }
  }


  if (m_config.fov_end != config.fov_end) {
    _itoa(config.fov_end, str, 10);
    result = configMessage(valName(config.fov_end), str);

    if (result != NULL) {
      m_config.fov_end = atoi(result);
    }
  }


  if (m_config.dataRecvPort != config.dataRecvPort) {
    _itoa(config.dataRecvPort, str, 10);
    result = configMessage(valName(config.dataRecvPort), str);

    if (result != NULL) {
      m_config.dataRecvPort = atoi(result);
    }
  }


  if (strcmp(m_config.dataRecvIp, config.dataRecvIp)) {
    result = configMessage(valName(config.dataRecvIp), (char *)config.dataRecvIp);

    if (result != NULL) {
      strcpy(m_config.dataRecvIp, result);
    }
  }

  disConfigConnect();
}



bool ETLidarDriver::dataPortConnect(const char *lidarIP, int localPort) {
  ScopedLocker lock(_data_lock);

  if (!socket_data) {
    return false;
  }

  if (!socket_data->IsSocketValid()) {
    if (socket_data->Initialize()) {
      if (!socket_data->Listen(NULL, localPort)) {
        socket_data->Close();
        return false;
      }

      socket_data->SetReceiveTimeout(DEFAULT_TIMEOUT / 1000,
                                     (DEFAULT_TIMEOUT % 1000) * 1000);
    }
  }

  return socket_data->IsSocketValid();
}

void ETLidarDriver::disableDataGrabbing() {
  {
    ScopedLocker l(_lock);

    if (m_isScanning) {
      m_isScanning = false;
      _dataEvent.set();
    }
  }
  _thread.join();
}

result_t ETLidarDriver::grabScanData(node_info *nodebuffer, size_t &count,
                                     uint32_t timeout) {
  switch (_dataEvent.wait(timeout)) {
    case Event::EVENT_TIMEOUT:
      count = 0;
      return RESULT_TIMEOUT;

    case Event::EVENT_OK: {
      if (scan_node_count == 0) {
        return RESULT_FAIL;
      }

      ScopedLocker l(_lock);
      size_t size_to_copy = std::min(count, scan_node_count);
      memcpy(nodebuffer, scan_node_buf, size_to_copy * sizeof(node_info));
      count = size_to_copy;
      scan_node_count = 0;
    }

    return RESULT_OK;

    default:
      count = 0;
      return RESULT_FAIL;
  }

}

result_t ETLidarDriver::getScanFrequency(scan_frequency &frequency,
    uint32_t timeout) {
  lidarConfig cfg;
  result_t  ans = RESULT_FAIL;

  if (!configPortConnect(m_port.c_str(), port)) {
    return  RESULT_FAIL;
  }

  char *result = configMessage(valName(cfg.motor_rpm));

  if (result != NULL) {
    cfg.motor_rpm = atoi(result);
    frequency.frequency = static_cast<uint32_t>(100 * (cfg.motor_rpm / 60.f + 0.4));
    ans = RESULT_OK;
  }

  return ans;

}

result_t ETLidarDriver::setScanFrequencyAdd(scan_frequency &frequency,
    uint32_t timeout) {
  lidarConfig cfg;
  result_t  ans = RESULT_FAIL;
  char str[32];

  if (!configPortConnect(m_port.c_str(), port)) {
    return ans;
  }

  char *result = configMessage(valName(cfg.motor_rpm));

  if (result == NULL) {
    return  ans;
  }

  if (result != NULL) {
    cfg.motor_rpm = atoi(result);

  }

  cfg.motor_rpm += 60;
  _itoa(cfg.motor_rpm, str, 10);
  result = configMessage(valName(cfg.motor_rpm), str);

  if (result != NULL) {
    cfg.motor_rpm = atoi(result);
    m_config.motor_rpm = cfg.motor_rpm;
    frequency.frequency = static_cast<uint32_t>(100 * (cfg.motor_rpm / 60.f + 0.4));
    ans = RESULT_OK;
  }


  return ans;
}

result_t ETLidarDriver::setScanFrequencyDis(scan_frequency &frequency,
    uint32_t timeout) {
  lidarConfig cfg;
  result_t  ans = RESULT_FAIL;
  char str[32];

  if (!configPortConnect(m_port.c_str(), port)) {
    return ans;
  }

  char *result = configMessage(valName(cfg.motor_rpm));

  if (result == NULL) {
    return  ans;
  }

  if (result != NULL) {
    cfg.motor_rpm = atoi(result);
  }

  cfg.motor_rpm -= 60;
  _itoa(cfg.motor_rpm, str, 10);
  result = configMessage(valName(cfg.motor_rpm), str);

  if (result != NULL) {
    cfg.motor_rpm = atoi(result);
    m_config.motor_rpm = cfg.motor_rpm;
    frequency.frequency = static_cast<uint32_t>(100 * (cfg.motor_rpm / 60.f + 0.4));
    ans = RESULT_OK;
  }


  return ans;
}

result_t ETLidarDriver::setScanFrequencyAddMic(scan_frequency &frequency,
    uint32_t timeout) {
  lidarConfig cfg;
  result_t  ans = RESULT_FAIL;
  char str[32];

  if (!configPortConnect(m_port.c_str(), port)) {
    return ans;
  }

  char *result = configMessage(valName(cfg.motor_rpm));

  if (result == NULL) {
    return  ans;
  }

  if (result != NULL) {
    cfg.motor_rpm = atoi(result);

  }

  cfg.motor_rpm += 6;
  _itoa(cfg.motor_rpm, str, 10);
  result = configMessage(valName(cfg.motor_rpm), str);

  if (result != NULL) {
    cfg.motor_rpm = atoi(result);
    m_config.motor_rpm = cfg.motor_rpm;
    frequency.frequency = static_cast<uint32_t>(100 * (cfg.motor_rpm / 60.f + 0.4));
    ans = RESULT_OK;
  }


  return ans;

}

result_t ETLidarDriver::setScanFrequencyDisMic(scan_frequency &frequency,
    uint32_t timeout) {
  lidarConfig cfg;
  result_t  ans = RESULT_FAIL;
  char str[32];

  if (!configPortConnect(m_port.c_str(), port)) {
    return ans;
  }

  char *result = configMessage(valName(cfg.motor_rpm));

  if (result == NULL) {
    return  ans;
  }

  if (result != NULL) {
    cfg.motor_rpm = atoi(result);

  }

  cfg.motor_rpm -= 6;
  _itoa(cfg.motor_rpm, str, 10);
  result = configMessage(valName(cfg.motor_rpm), str);

  if (result != NULL) {
    cfg.motor_rpm = atoi(result);
    m_config.motor_rpm = cfg.motor_rpm;
    frequency.frequency = static_cast<uint32_t>(100 * (cfg.motor_rpm / 60.f + 0.4));
    ans = RESULT_OK;
  }

  return ans;
}

result_t ETLidarDriver::getSamplingRate(sampling_rate &rate, uint32_t timeout) {

  lidarConfig cfg;
  result_t  ans = RESULT_FAIL;

  if (!configPortConnect(m_port.c_str(), port)) {
    return ans;
  }

  char *result = configMessage(valName(cfg.laserScanFrequency));
  ans = RESULT_OK;

  if (result != NULL) {
    cfg.laserScanFrequency = atoi(result);
    m_sampleRate = 1000 / cfg.laserScanFrequency * 1000;
  } else {
    cfg.laserScanFrequency = 50;
    m_sampleRate = 1000 / cfg.laserScanFrequency * 1000;
  }

  rate.rate = YDLIDAR_RATE_10K;

  return ans;

}

result_t ETLidarDriver::setSamplingRate(sampling_rate &rate, uint32_t timeout) {
  return RESULT_FAIL;
}

result_t ETLidarDriver::getZeroOffsetAngle(offset_angle &angle,
    uint32_t timeout) {
  return RESULT_FAIL;
}

result_t ETLidarDriver::setScanHeartbeat(scan_heart_beat &beat,
    uint32_t timeout) {
  return RESULT_FAIL;
}

result_t ETLidarDriver::startAutoScan(bool force, uint32_t timeout) {
  result_t ans;
  {
    bool ret = startMeasure();

    if (!ret) {
      startMeasure();
    }

    if (!ret) {
      return RESULT_FAIL;
    }

  }

  return RESULT_OK;
}

result_t ETLidarDriver::checkAutoConnecting() {
  result_t ans = RESULT_FAIL;
  isAutoconnting = true;
  m_InvalidNodeCount = 0;
  setDriverError(TimeoutError);

  while (isAutoReconnect && isAutoconnting) {
    {
      disConfigConnect();
      ScopedLocker lock(_data_lock);

      if (!socket_data) {
        return RESULT_FAIL;
      }

      if (socket_data->isOpen()) {
        size_t buffer_size = socket_data->available();

        if (m_BufferSize && m_BufferSize % 90 == 0) {
          setDriverError(BlockError);
        } else {
          if (buffer_size > 0 || m_BufferSize > 0) {
            setDriverError(TrembleError);
            m_BufferSize += buffer_size;
          } else {
            setDriverError(NotBufferError);
          }
        }
      }

      socket_data->Close();
    }
    retryCount++;

    if (retryCount > 100) {
      retryCount = 100;
    }

    int tempCount = 0;

    while (isAutoReconnect && isscanning() && tempCount < retryCount) {
      delay(100);
      tempCount++;
    }

    tempCount = 0;
    int retryConnect = 0;

    while (isAutoReconnect &&
           connect(m_port.c_str(), m_baudrate) != RESULT_OK) {
      retryConnect++;

      if (retryConnect > 25) {
        retryConnect = 25;
      }

      setDriverError(NotOpenError);

      while (isAutoReconnect && isscanning() && tempCount < retryConnect) {
        delay(200);
        tempCount++;
      }
    }

    if (!isAutoReconnect) {
      m_isScanning = false;
      return RESULT_FAIL;
    }

    if (isconnected()) {
      delay(100);
      {
        ans = startAutoScan();

        if (!IS_OK(ans)) {
          ans = startAutoScan();
        }
      }

      if (IS_OK(ans)) {
        if (getDriverError() == DeviceNotFoundError) {
          setDriverError(NoError);
        }

        isAutoconnting = false;
        return ans;
      } else {
        setDriverError(DeviceNotFoundError);
      }
    }
  }

  return RESULT_FAIL;

}

void ETLidarDriver::CheckLaserStatus() {
  if (m_InvalidNodeCount < 2) {
    if (m_driverErrno == NoError) {
      setDriverError(LaserFailureError);
    }
  } else {
    if (m_driverErrno == LaserFailureError) {
      setDriverError(NoError);
    }
  }

  m_InvalidNodeCount = 0;
}

int ETLidarDriver::cacheScanData() {
  node_info      local_buf[100];
  size_t         count = 100;
  node_info      local_scan[MAX_SCAN_NODES];
  size_t         scan_count = 0;
  result_t       ans = RESULT_FAIL;
  memset(local_scan, 0, sizeof(local_scan));
  waitScanData(local_buf, count);

  int timeout_count   = 0;
  retryCount = 0;
  m_BufferSize = 0;
  m_InvalidNodeCount = 0;
  bool m_last_frame_valid = false;

  while (m_isScanning) {
    count = 100;
    ans = waitScanData(local_buf, count);

    if (!IS_OK(ans)) {
      if (IS_FAIL(ans) || timeout_count > DEFAULT_TIMEOUT_COUNT) {
        if (!isAutoReconnect) {
          fprintf(stderr, "exit scanning thread!!\n");
          fflush(stderr);
          {
            m_isScanning = false;
          }
          return RESULT_FAIL;
        } else {
          if (m_last_frame_valid) {
            m_BufferSize = 0;
            m_last_frame_valid = false;
          }

          ans = checkAutoConnecting();

          if (IS_OK(ans)) {
            timeout_count = 0;
            local_scan[0].sync = NODE_UNSYNC;
          } else {
            m_isScanning = false;
            return RESULT_FAIL;
          }
        }
      } else {
        timeout_count++;
        local_scan[0].sync = NODE_UNSYNC;

        if (m_driverErrno == NoError) {
          setDriverError(TimeoutError);
        }

        fprintf(stderr, "timeout count: %d\n", timeout_count);
        fflush(stderr);
      }
    } else {
      timeout_count = 0;
      retryCount = 0;
      m_BufferSize = 0;
      m_last_frame_valid = true;

      if (retryCount != 0) {
        setDriverError(NoError);
      }
    }


    for (size_t pos = 0; pos < count; ++pos) {
      if (local_buf[pos].sync & LIDAR_RESP_SYNCBIT) {
        if ((local_scan[0].sync & LIDAR_RESP_SYNCBIT)) {
          _lock.lock();//timeout lock, wait resource copy
          local_scan[0].stamp = local_buf[pos].stamp;
          local_scan[0].delayTime = local_buf[pos].delayTime;
          local_scan[0].scanFreq = local_buf[pos].scanFreq;
          memcpy(scan_node_buf, local_scan, scan_count * sizeof(node_info));
          scan_node_count = scan_count;
          _dataEvent.set();
          _lock.unlock();
        }

        scan_count = 0;
      }

      local_scan[scan_count++] = local_buf[pos];

      if (scan_count == _countof(local_scan)) {
        scan_count -= 1;
      }
    }
  }

  m_isScanning = false;

  return RESULT_OK;
}

result_t ETLidarDriver::waitScanData(node_info *nodebuffer, size_t &count,
                                     uint32_t timeout) {
  if (!m_isConnected) {
    count = 0;
    return RESULT_FAIL;
  }

  size_t     recvNodeCount    =  0;
  uint32_t   startTs          = getms();
  uint32_t   waitTime         = 0;
  result_t   ans              = RESULT_FAIL;

  while ((waitTime = getms() - startTs) <= timeout && recvNodeCount < count) {
    node_info node;
    ans = waitPackage(&node, timeout - waitTime);

    if (!IS_OK(ans)) {
      count = recvNodeCount;
      return ans;
    }

    nodebuffer[recvNodeCount++] = node;

    if (node.sync & LIDAR_RESP_SYNCBIT) {
      count = recvNodeCount;
      CheckLaserStatus();
      return RESULT_OK;
    }

    if (recvNodeCount == count) {
      return RESULT_OK;
    }
  }

  count = recvNodeCount;
  return RESULT_FAIL;
}

result_t ETLidarDriver::waitPackage(node_info *node, uint32_t timeout) {

  int offset;
  result_t ans;

  if (nodeIndex == 0) {
    ans = getScanData();

    if (!IS_OK((ans))) {
      return ans;
    }
  }

  (*node).sync =  NODE_UNSYNC;
  (*node).scanFreq = 0;
  (*node).debugInfo = 0xff;
  (*node).index = 0xff;

  offset = frame.dataIndex + 4 * nodeIndex;
  (*node).dist = static_cast<uint16_t>(DSL(frame.frameBuf[offset + 2],
                        8) | DSL(frame.frameBuf[offset + 3], 0));

  if ((*node).dist > 0) {
    m_InvalidNodeCount++;
  }

  if (isV1Protocol(frame.dataFormat)) {
    (*node).qual = (uint16_t)(DSL(frame.frameBuf[offset],
                                          8) | DSL(frame.frameBuf[offset + 1], 0));
  } else {
    (*node).qual = (uint16_t)frame.frameBuf[offset];
  }

  if (nodeIndex > 0) {
    if (isV1Protocol(frame.dataFormat)) {
      m_currentAngle = (frame.frameCrc - frame.startAngle) / (frame.dataNum - 1) /
                       100.f;
    } else {
      m_currentAngle =  frame.frameBuf[offset + 1] / 100.f;
    }

    m_currentAngle += m_lastAngle;
  } else {
    m_currentAngle = frame.startAngle / 100.f;
  }

  m_currentAngle = ydlidar::core::math::normalize_angle_positive_from_degree(
                     m_currentAngle);
  (*node).angle = static_cast<uint16_t>(m_currentAngle * 100);
  m_lastAngle = m_currentAngle;
  nodeIndex++;

  if (nodeIndex >= frame.dataNum) {
    (*node).sync = frame.headFrameFlag ? NODE_SYNC : NODE_UNSYNC;
    (*node).stamp = getTime();//(uint64_t)(frame.timestamp * 100);
    (*node).delayTime = 0;
    nodeIndex = 0;
    m_lastAngle = 0.f;
    m_currentAngle = 0.f;
  }

  return RESULT_OK;
}

result_t ETLidarDriver::getScanData() {
  /* wait data from socket. */
  {
    ScopedLocker lock(_data_lock);

    if (!socket_data) {
      return RESULT_FAIL;
    }

    if (socket_data->Receive(sizeof(frame.frameBuf),
                             reinterpret_cast<uint8_t *>(frame.frameBuf)) < 0) {
      return RESULT_TIMEOUT;
    }
  }

  /* check frame head */
  frame.frameHead = DSL(frame.frameBuf[0], 8) | DSL(frame.frameBuf[1], 0);

  if (FRAME_PREAMBLE != frame.frameHead) {
    return RESULT_TIMEOUT;
  }

  /* check device type */
  frame.deviceType = (frame.frameBuf[2] >> 4) & 0xf;

  if (LIDAR_2D != frame.deviceType) {
    return RESULT_TIMEOUT;
  }

  /* check frame type */
  frame.frameType = frame.frameBuf[2] & 0xf;

  if (DATA_FRAME != frame.frameType) {
    return RESULT_TIMEOUT;
  }

  /* parser head length */
  frame.dataIndex = (frame.frameBuf[3] >> 4) & 0xf;
  frame.dataIndex = (frame.dataIndex + 1) * 4;

  /* parser frame index */
  frame.frameIndex = frame.frameBuf[3] & 0xf;

  /* parser timestamp */
  frame.timestamp = DSL(frame.frameBuf[4], 24) | DSL(frame.frameBuf[5], 16)
                    | DSL(frame.frameBuf[6], 8)  | DSL(frame.frameBuf[7], 0);

  /* parser head frame flag */
  frame.headFrameFlag = (frame.frameBuf[8] >> 4) & 0xf;

  /* parser data format */
  frame.dataFormat = frame.frameBuf[8] & 0xf;

  /* parser distance scale */
  frame.disScale = frame.frameBuf[9];

  /* parser start angle */
  frame.startAngle = DSL(frame.frameBuf[10], 8) | DSL(frame.frameBuf[11], 0);

  /* parser valid data number */
  frame.dataNum = DSL(frame.frameBuf[12], 8) | DSL(frame.frameBuf[13], 0);

  /* parser frame crc */
  frame.frameCrc = DSL(frame.frameBuf[14], 8) | DSL(frame.frameBuf[15], 0);

  if (frame.dataNum < 1) {
    return RESULT_TIMEOUT;
  }

  return RESULT_OK;

}

