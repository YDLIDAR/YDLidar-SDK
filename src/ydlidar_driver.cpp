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
#include <core/serial/serial.h>
#include <core/network/ActiveSocket.h>
#include "ydlidar_driver.h"
#include <core/serial/common.h>
#include <math.h>
#include <ydlidar_config.h>
using namespace impl;

namespace ydlidar {
using namespace core::serial;
using namespace core::common;
using namespace core::network;

YDlidarDriver::YDlidarDriver(uint8_t type):
  _serial(NULL),
  m_TranformerType(type) {
  m_isConnected         = false;
  m_isScanning          = false;
  //串口配置参数
  m_intensities         = false;
  isAutoReconnect       = true;
  isAutoconnting        = false;
  m_baudrate            = 230400;
  m_SupportMotorDtrCtrl = true;
  m_HeartBeat           = false;
  scan_node_count       = 0;
  sample_rate           = 5000;
  m_PointTime           = 1e9 / 5000;
  trans_delay           = 0;
  scan_frequence        = 0;
  m_sampling_rate       = -1;
  model                 = -1;
  retryCount            = 0;
  has_device_header     = false;
  m_SingleChannel       = false;
  m_LidarType           = TYPE_TRIANGLE;

  //解析参数
  PackageSampleBytes    = 2;
  IntervalSampleAngle   = 0.0;
  FirstSampleAngle      = 0;
  LastSampleAngle       = 0;
  CheckSum              = 0;
  CheckSumCal           = 0;
  SampleNumlAndCTCal    = 0;
  LastSampleAngleCal    = 0;
  CheckSumResult        = true;
  Valu8Tou16            = 0;
  package_CT            = CT_Normal;
  nowPackageNum         = 0;
  package_Sample_Num    = 0;

  last_device_byte      = 0x00;
  asyncRecvPos          = 0;
  async_size            = 0;
  headerBuffer          = reinterpret_cast<uint8_t *>(&header_);
  infoBuffer            = reinterpret_cast<uint8_t *>(&info_);
  healthBuffer          = reinterpret_cast<uint8_t *>(&health_);
  package_Sample_Index  = 0;
  globalRecvBuffer      = new uint8_t[sizeof(node_packages)];
  scan_node_buf         = new node_info[MAX_SCAN_NODES];
  package_index         = 0;
  has_package_error     = false;

  get_device_health_success         = false;
  get_device_info_success           = false;
  IntervalSampleAngle_LastPackage   = 0.0;
  m_heartbeat_ts = getms();
  m_BlockRevSize = 0;
}

YDlidarDriver::~YDlidarDriver() {
  {
    m_isScanning = false;
  }

  isAutoReconnect = false;
  _thread.join();
  delay(200);

  ScopedLocker lck(_cmd_lock);

  if (_serial) {
    if (_serial->isOpen()) {
      _serial->flush();
      _serial->closePort();
    }
  }

  if (_serial) {
    delete _serial;
    _serial = NULL;
  }

  if (globalRecvBuffer) {
    delete[] globalRecvBuffer;
    globalRecvBuffer = NULL;
  }

  if (scan_node_buf) {
    delete[] scan_node_buf;
    scan_node_buf = NULL;
  }
}


result_t YDlidarDriver::connect(const char *port_path, uint32_t baudrate) {
  ScopedLocker lck(_cmd_lock);
  m_baudrate = baudrate;
  serial_port = string(port_path);

  if (!_serial) {
    if (m_TranformerType == YDLIDAR_TYPE_TCP) {
      _serial = new CActiveSocket();
    } else {
      _serial = new serial::Serial(port_path, m_baudrate,
                                   serial::Timeout::simpleTimeout(DEFAULT_TIMEOUT));
    }

    _serial->bindport(port_path, baudrate);
  }

  {
    ScopedLocker l(_lock);

    if (!_serial->open()) {
      setDriverError(NotOpenError);
      return RESULT_FAIL;
    }

    m_isConnected = true;

  }

  stopScan();
  delay(100);
  clearDTR();

  return RESULT_OK;
}

const char *YDlidarDriver::DescribeError(bool isTCP) {
  char const *value = "";

  if (_serial) {
    return _serial->DescribeError();
  }

  return value;
}


void YDlidarDriver::setDTR() {
  if (!m_isConnected) {
    return ;
  }

  if (_serial) {
    _serial->setDTR(1);
  }

}

void YDlidarDriver::clearDTR() {
  if (!m_isConnected) {
    return ;
  }

  if (_serial) {
    _serial->setDTR(0);
  }
}
void YDlidarDriver::flushSerial() {
  if (!m_isConnected) {
    return;
  }

  size_t len = _serial->available();

  if (len) {
    _serial->readSize(len);
  }

  delay(20);
}


void YDlidarDriver::disconnect() {
  isAutoReconnect = false;

  if (!m_isConnected) {
    return ;
  }

  stop();
  delay(10);
  ScopedLocker l(_cmd_lock);

  if (_serial) {
    if (_serial->isOpen()) {
      _serial->closePort();
    }
  }

  m_isConnected = false;

}


void YDlidarDriver::disableDataGrabbing() {
  {
    if (m_isScanning) {
      m_isScanning = false;
      _dataEvent.set();
    }
  }
  _thread.join();
}

bool YDlidarDriver::isscanning() const {
  return m_isScanning;
}

bool YDlidarDriver::isconnected() const {
  return m_isConnected;
}

result_t YDlidarDriver::sendCommand(uint8_t cmd, const void *payload,
                                    size_t payloadsize) {
  uint8_t pkt_header[10];
  cmd_packet *header = reinterpret_cast<cmd_packet * >(pkt_header);
  uint8_t checksum = 0;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  if (payloadsize && payload) {
    cmd |= LIDAR_CMDFLAG_HAS_PAYLOAD;
  }

  header->syncByte = LIDAR_CMD_SYNC_BYTE;
  header->cmd_flag = cmd;
  sendData(pkt_header, 2) ;

  if ((cmd & LIDAR_CMDFLAG_HAS_PAYLOAD) && payloadsize && payload) {
    checksum ^= LIDAR_CMD_SYNC_BYTE;
    checksum ^= cmd;
    checksum ^= (payloadsize & 0xFF);

    for (size_t pos = 0; pos < payloadsize; ++pos) {
      checksum ^= ((uint8_t *)payload)[pos];
    }

    uint8_t sizebyte = (uint8_t)(payloadsize);
    sendData(&sizebyte, 1);

    sendData((const uint8_t *)payload, sizebyte);

    sendData(&checksum, 1);
  }

  return RESULT_OK;
}

result_t YDlidarDriver::sendData(const uint8_t *data, size_t size) {
  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  if (data == NULL || size == 0) {
    return RESULT_FAIL;
  }

  size_t r = 0;

  while (size) {
    r = _serial->writeData(data, size);

    if (r < 1) {
      return RESULT_FAIL;
    }

    size -= r;
    data += r;
  }

  return RESULT_OK;
}

result_t YDlidarDriver::getData(uint8_t *data, size_t size) {
  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  size_t r = 0;

  while (size) {
    r = _serial->readData(data, size);

    if (r < 1) {
      return RESULT_FAIL;
    }

    size -= r;
    data += r;
  }

  return RESULT_OK;
}

result_t YDlidarDriver::waitResponseHeader(lidar_ans_header *header,
    uint32_t timeout) {
  int  recvPos     = 0;
  uint32_t startTs = getms();
  uint8_t  recvBuffer[sizeof(lidar_ans_header)];
  uint8_t  *headerBuffer = reinterpret_cast<uint8_t *>(header);
  uint32_t waitTime = 0;
  has_device_header = false;
  last_device_byte = 0x00;

  while ((waitTime = getms() - startTs) <= timeout) {
    size_t remainSize = sizeof(lidar_ans_header) - recvPos;
    size_t recvSize = 0;
    result_t ans = waitForData(remainSize, timeout - waitTime, &recvSize);

    if (!IS_OK(ans)) {
      return ans;
    }

    if (recvSize > remainSize) {
      recvSize = remainSize;
    }

    ans = getData(recvBuffer, recvSize);

    if (IS_FAIL(ans)) {
      return RESULT_FAIL;
    }

    for (size_t pos = 0; pos < recvSize; ++pos) {
      uint8_t currentByte = recvBuffer[pos];

      switch (recvPos) {
        case 0:
          if (currentByte != LIDAR_ANS_SYNC_BYTE1) {
            if (last_device_byte == (PH & 0xFF) && currentByte == (PH >> 8)) {
              has_device_header = true;
            }

            last_device_byte = currentByte;
            continue;
          }

          break;

        case 1:
          if (currentByte != LIDAR_ANS_SYNC_BYTE2) {
            last_device_byte = currentByte;
            recvPos = 0;
            continue;
          }

          break;
      }

      headerBuffer[recvPos++] = currentByte;
      last_device_byte = currentByte;

      if (recvPos == sizeof(lidar_ans_header)) {
        return RESULT_OK;
      }
    }
  }

  return RESULT_FAIL;
}

result_t YDlidarDriver::waitForData(size_t data_count, uint32_t timeout,
                                    size_t *returned_size) {
  size_t length = 0;

  if (returned_size == NULL) {
    returned_size = (size_t *)&length;
  }

  return (result_t)_serial->waitfordata(data_count, timeout, returned_size);
}

result_t YDlidarDriver::checkAutoConnecting(bool serialError) {
  result_t ans = RESULT_FAIL;
  isAutoconnting = true;
  m_InvalidNodeCount = 0;

  if (m_driverErrno != BlockError) {
    setDriverError(TimeoutError);
  }


  while (isAutoReconnect && isAutoconnting) {
    {
      ScopedLocker l(_cmd_lock);

      if (_serial) {
        if (_serial->isOpen() || m_isConnected) {
          size_t buffer_size = _serial->available();
          m_BufferSize += buffer_size;

          if (m_BufferSize && m_BufferSize % 7 == 0) {
            setDriverError(BlockError);
          } else {
            if (buffer_size > 0 || m_BufferSize > 0) {
              if (m_driverErrno != BlockError) {
                setDriverError(TrembleError);
              }
            } else {
              setDriverError(NotBufferError);
            }
          }

          if ((retryCount % 2 == 1) || serialError) {
            m_isConnected = false;
            _serial->closePort();
            delete _serial;
            _serial = NULL;
          } else {
            m_BufferSize -= buffer_size;
          }
        }
      }
    }

    if (!m_isConnected && ((retryCount % 2 == 1) || serialError)) {
      if (!IS_OK(connect(serial_port.c_str(), m_baudrate))) {
        setDriverError(NotOpenError);
      }
    }

    retryCount++;

    if (retryCount > 50) {
      retryCount = 50;
    }

    int tempCount = 0;

    while (isAutoReconnect && isscanning() && tempCount < retryCount) {
      delay(200);
      tempCount++;
    }

    tempCount = 0;
    int retryConnect = 0;

    while (isAutoReconnect &&
           connect(serial_port.c_str(), m_baudrate) != RESULT_OK) {
      retryConnect++;

      if (retryConnect > 25) {
        retryConnect = 25;
      }

      tempCount = 0;
      setDriverError(NotOpenError);

      while (isAutoReconnect && isscanning() && tempCount < retryConnect) {
        delay(200);
        tempCount++;
      }
    }

    if (!isAutoReconnect) {
      m_isScanning = false;
      isAutoconnting = false;
      return RESULT_FAIL;
    }

    if (isconnected()) {
      delay(50);

      if (!m_SingleChannel && m_driverErrno != BlockError) {
        device_info devinfo;
        ans = getDeviceInfo(devinfo);

        if (!IS_OK(ans)) {
          stopScan();
          ans = getDeviceInfo(devinfo);
        }

        if (!IS_OK(ans)) {
          setDriverError(DeviceNotFoundError);
          continue;
        }
      }

      {
        ScopedLocker l(_cmd_lock);
        ans = startAutoScan();

        if (!IS_OK(ans)) {
          ans = startAutoScan();
        }
      }

      if (IS_OK(ans)) {
        isAutoconnting = false;

        if (getDriverError() == DeviceNotFoundError) {
          setDriverError(NoError);
        }

        return ans;
      } else {
        setDriverError(DeviceNotFoundError);
      }
    }
  }

  isAutoconnting = false;
  return RESULT_FAIL;

}

result_t YDlidarDriver::autoHeartBeat() {
  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  ScopedLocker lock(_lock);
  result_t ans = sendCommand(LIDAR_CMD_SCAN);
  return ans;
}

void YDlidarDriver::KeepLiveHeartBeat() {
  if (m_HeartBeat) {
    uint32_t end_ts = getms();

    if (end_ts - m_heartbeat_ts > DEFAULT_HEART_BEAT) {
      autoHeartBeat();
      m_heartbeat_ts = end_ts;
    }
  }
}

void YDlidarDriver::CheckLaserStatus() {
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

int YDlidarDriver::cacheScanData() {
  node_info      local_buf[128];
  size_t         count = 128;
  node_info      local_scan[MAX_SCAN_NODES];
  size_t         scan_count = 0;
  result_t       ans = RESULT_FAIL;
  memset(local_scan, 0, sizeof(local_scan));

  if (m_SingleChannel) {
    waitDevicePackage();
  }

  flushSerial();
  waitScanData(local_buf, count);

  int timeout_count   = 0;
  retryCount = 0;
  m_BufferSize = 0;
  m_heartbeat_ts = getms();
  bool m_last_frame_valid = false;

  while (m_isScanning) {
    count = 128;
    ans = waitScanData(local_buf, count, DEFAULT_TIMEOUT / 2);

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

          ans = checkAutoConnecting(IS_FAIL(ans));

          if (IS_OK(ans)) {
            timeout_count = 0;
            local_scan[0].sync_flag = Node_NotSync;
          } else {
            m_isScanning = false;
            return RESULT_FAIL;
          }
        }
      } else {
        timeout_count++;
        local_scan[0].sync_flag = Node_NotSync;

        if (m_driverErrno == NoError) {
          setDriverError(TimeoutError);
        }

        fprintf(stderr, "timout count: %d\n", timeout_count);
        fflush(stderr);
      }
    } else {
      timeout_count = 0;
      m_last_frame_valid = true;

      if (retryCount != 0) {
        setDriverError(NoError);
      }

      retryCount = 0;
      m_BufferSize = 0;
    }


    for (size_t pos = 0; pos < count; ++pos) {
      if (local_buf[pos].sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT) {
        if ((local_scan[0].sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT)) {
          _lock.lock();//timeout lock, wait resource copy
          local_scan[0].stamp = local_buf[pos].stamp;

          if (local_scan[0].stamp == 0) {
            local_scan[0].stamp = getTime();
          }

          local_scan[0].scan_frequence = local_buf[pos].scan_frequence;
          local_scan[0].delay_time = local_buf[pos].delay_time;
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

    KeepLiveHeartBeat();
  }

  m_isScanning = false;

  return RESULT_OK;
}

result_t YDlidarDriver::checkDeviceInfo(uint8_t *recvBuffer, uint8_t byte,
                                        int recvPos, int recvSize, int pos) {
  if (asyncRecvPos == sizeof(lidar_ans_header)) {
    if ((((pos < recvSize - 1) && byte == LIDAR_ANS_SYNC_BYTE1) ||
         (last_device_byte == LIDAR_ANS_SYNC_BYTE1 && byte == LIDAR_ANS_SYNC_BYTE2)) &&
        recvPos == 0) {
      if ((last_device_byte == LIDAR_ANS_SYNC_BYTE1 &&
           byte == LIDAR_ANS_SYNC_BYTE2)) {
        asyncRecvPos = 0;
        async_size = 0;
        headerBuffer[asyncRecvPos] = last_device_byte;
        asyncRecvPos++;
        headerBuffer[asyncRecvPos] = byte;
        asyncRecvPos++;
        last_device_byte = byte;
        return RESULT_OK;
      } else {
        if (pos < recvSize - 1) {
          if (recvBuffer[pos + 1] == LIDAR_ANS_SYNC_BYTE2) {
            asyncRecvPos = 0;
            async_size = 0;
            headerBuffer[asyncRecvPos] = byte;
            asyncRecvPos++;
            last_device_byte = byte;
            return RESULT_OK;
          }
        }

      }

    }

    last_device_byte = byte;

    if (header_.type == LIDAR_ANS_TYPE_DEVINFO ||
        header_.type == LIDAR_ANS_TYPE_DEVHEALTH) {
      if (header_.size < 1) {
        asyncRecvPos = 0;
        async_size = 0;
      } else {

        if (header_.type == LIDAR_ANS_TYPE_DEVHEALTH) {
          if (async_size < sizeof(health_)) {
            healthBuffer[async_size] = byte;
            async_size++;

            if (async_size == sizeof(health_)) {
              asyncRecvPos = 0;
              async_size = 0;
              get_device_health_success = true;
              last_device_byte = byte;
              return RESULT_OK;
            }

          } else {
            asyncRecvPos = 0;
            async_size = 0;
          }

        } else {

          if (async_size < sizeof(info_)) {
            infoBuffer[async_size] = byte;
            async_size++;

            if (async_size == sizeof(info_)) {
              asyncRecvPos = 0;
              async_size = 0;
              get_device_info_success = true;

              last_device_byte = byte;
              return RESULT_OK;
            }

          } else {
            asyncRecvPos = 0;
            async_size = 0;
          }
        }
      }
    } else if (header_.type == LIDAR_ANS_TYPE_MEASUREMENT) {
      asyncRecvPos = 0;
      async_size = 0;

    }

  } else {

    switch (asyncRecvPos) {
      case 0:
        if (byte == LIDAR_ANS_SYNC_BYTE1 && recvPos == 0) {
          headerBuffer[asyncRecvPos] = byte;
          last_device_byte = byte;
          asyncRecvPos++;
        }

        break;

      case 1:
        if (byte == LIDAR_ANS_SYNC_BYTE2 && recvPos == 0) {
          headerBuffer[asyncRecvPos] = byte;
          asyncRecvPos++;
          last_device_byte = byte;
          return RESULT_OK;
        } else {
          asyncRecvPos = 0;
        }

        break;

      default:
        break;
    }

    if (asyncRecvPos >= 2) {
      if (((pos < recvSize - 1 && byte == LIDAR_ANS_SYNC_BYTE1) ||
           (last_device_byte == LIDAR_ANS_SYNC_BYTE1 && byte == LIDAR_ANS_SYNC_BYTE2)) &&
          recvPos == 0) {
        if ((last_device_byte == LIDAR_ANS_SYNC_BYTE1 &&
             byte == LIDAR_ANS_SYNC_BYTE2)) {
          asyncRecvPos = 0;
          async_size = 0;
          headerBuffer[asyncRecvPos] = last_device_byte;
          asyncRecvPos++;
        } else {
          if (pos < recvSize - 2) {
            if (recvBuffer[pos + 1] == LIDAR_ANS_SYNC_BYTE2) {
              asyncRecvPos = 0;
            }
          }
        }
      }

      headerBuffer[asyncRecvPos] = byte;
      asyncRecvPos++;
      last_device_byte = byte;
      return RESULT_OK;
    }
  }

  return RESULT_FAIL;

}

result_t YDlidarDriver::waitDevicePackage(uint32_t timeout) {
  int recvPos = 0;
  asyncRecvPos = 0;
  uint32_t startTs = getms();
  uint32_t waitTime = 0;
  result_t ans = RESULT_FAIL;

  while ((waitTime = getms() - startTs) <= timeout) {
    size_t remainSize = PackagePaidBytes - recvPos;
    size_t recvSize = 0;
    result_t ans = waitForData(remainSize, timeout - waitTime, &recvSize);

    if (!IS_OK(ans)) {
      return ans;
    }

    ans = RESULT_FAIL;

    if (recvSize > remainSize) {
      recvSize = remainSize;
    }

    getData(globalRecvBuffer, recvSize);

    for (size_t pos = 0; pos < recvSize; ++pos) {
      uint8_t currentByte = globalRecvBuffer[pos];

      if (checkDeviceInfo(globalRecvBuffer, currentByte, recvPos, recvSize,
                          pos) == RESULT_OK) {
        continue;
      }
    }

    if (get_device_info_success) {
      ans = RESULT_OK;
      break;
    }
  }

  flushSerial();
  return ans;

}

void YDlidarDriver::checkBlockStatus(uint8_t currentByte) {
  switch (m_BlockRevSize) {
    case 0:
      if (currentByte == LIDAR_ANS_SYNC_BYTE1) {
        m_BlockRevSize++;
      }

      break;

    case 1:
      if (currentByte == LIDAR_ANS_SYNC_BYTE2) {
        setDriverError(BlockError);
        m_BlockRevSize = 0;
      }

      break;

    default:
      break;
  }
}

result_t YDlidarDriver::parseResponseHeader(uint8_t  *packageBuffer,
    uint32_t timeout) {
  int recvPos         = 0;
  uint32_t startTs    = getms();
  uint32_t waitTime   = 0;
  m_BlockRevSize = 0;
  package_Sample_Num         = 0;
  uint8_t package_type    = 0;
  result_t ans = RESULT_TIMEOUT;

  while ((waitTime = getms() - startTs) <= timeout) {
    size_t remainSize   = PackagePaidBytes - recvPos;
    size_t recvSize     = 0;
    ans = waitForData(remainSize, timeout - waitTime, &recvSize);

    if (!IS_OK(ans)) {
      return ans;
    }

    if (recvSize > remainSize) {
      recvSize = remainSize;
    }

    getData(globalRecvBuffer, recvSize);

    for (size_t pos = 0; pos < recvSize; ++pos) {
      uint8_t currentByte = globalRecvBuffer[pos];

      switch (recvPos) {
        case 0:
          if (currentByte == (PH & 0xFF)) {
          } else {
            checkBlockStatus(currentByte);
            continue;
          }

          break;

        case 1:
          CheckSumCal = PH;

          if (currentByte == (PH >> 8)) {
            if (m_driverErrno == BlockError) {
              setDriverError(NoError);
            }
          } else {
            has_package_error = true;
            recvPos = 0;
            continue;
          }

          break;

        case 2:
          SampleNumlAndCTCal = currentByte;
          package_type = currentByte & 0x01;

          if ((package_type == CT_Normal) || (package_type == CT_RingStart)) {
            if (package_type == CT_RingStart) {
              scan_frequence = (currentByte & 0xFE) >> 1;
            }
          } else {
            has_package_error = true;
            recvPos = 0;
            continue;
          }

          break;

        case 3:
          SampleNumlAndCTCal += (currentByte * 0x100);
          package_Sample_Num = currentByte;
          break;

        case 4:
          if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
            FirstSampleAngle = currentByte;
          } else {
            has_package_error = true;
            recvPos = 0;
            continue;
          }

          break;

        case 5:
          FirstSampleAngle += currentByte * 0x100;
          CheckSumCal ^= FirstSampleAngle;
          FirstSampleAngle = FirstSampleAngle >> 1;
          break;

        case 6:
          if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
            LastSampleAngle = currentByte;
          } else {
            has_package_error = true;
            recvPos = 0;
            continue;
          }

          break;

        case 7:
          LastSampleAngle = currentByte * 0x100 + LastSampleAngle;
          LastSampleAngleCal = LastSampleAngle;
          LastSampleAngle = LastSampleAngle >> 1;

          if (package_Sample_Num == 1) {
            IntervalSampleAngle = 0;
          } else {
            if (LastSampleAngle < FirstSampleAngle) {
              if ((FirstSampleAngle > 270 * 64) && (LastSampleAngle < 90 * 64)) {
                IntervalSampleAngle = (float)((360 * 64 + LastSampleAngle -
                                               FirstSampleAngle) / ((
                                                     package_Sample_Num - 1) * 1.0));
                IntervalSampleAngle_LastPackage = IntervalSampleAngle;
              } else {
                IntervalSampleAngle = IntervalSampleAngle_LastPackage;
              }
            } else {
              IntervalSampleAngle = (float)((LastSampleAngle - FirstSampleAngle) / ((
                                              package_Sample_Num - 1) * 1.0));
              IntervalSampleAngle_LastPackage = IntervalSampleAngle;
            }
          }

          break;

        case 8:
          CheckSum = currentByte;
          break;

        case 9:
          CheckSum += (currentByte * 0x100);
          break;
      }

      packageBuffer[recvPos++] = currentByte;
    }

    if (recvPos == PackagePaidBytes) {
      ans = RESULT_OK;
      break;
    }

    ans = RESULT_TIMEOUT;
  }

  return ans;
}

result_t YDlidarDriver::parseResponseScanData(uint8_t *packageBuffer,
    uint32_t timeout) {
  int recvPos         = 0;
  uint32_t startTs    = getms();
  uint32_t waitTime   = 0;
  result_t ans = RESULT_TIMEOUT;

  while ((waitTime = getms() - startTs) <= timeout) {
    size_t remainSize = package_Sample_Num * PackageSampleBytes - recvPos;
    size_t recvSize = 0;
    ans = waitForData(remainSize, timeout - waitTime, &recvSize);

    if (!IS_OK(ans)) {
      return ans;
    }

    if (recvSize > remainSize) {
      recvSize = remainSize;
    }

    getData(globalRecvBuffer, recvSize);

    for (size_t pos = 0; pos < recvSize; ++pos) {
      if (m_intensities && !isTOFLidar(m_LidarType)) {
        if (recvPos % 3 == 2) {
          Valu8Tou16 += globalRecvBuffer[pos] * 0x100;
          CheckSumCal ^= Valu8Tou16;
        } else if (recvPos % 3 == 1) {
          Valu8Tou16 = globalRecvBuffer[pos];
        } else {
          CheckSumCal ^= globalRecvBuffer[pos];
        }
      } else {
        if (recvPos % 2 == 1) {
          Valu8Tou16 += globalRecvBuffer[pos] * 0x100;
          CheckSumCal ^= Valu8Tou16;
        } else {
          Valu8Tou16 = globalRecvBuffer[pos];
        }
      }

      packageBuffer[PackagePaidBytes + recvPos] = globalRecvBuffer[pos];
      recvPos++;
    }

    if (package_Sample_Num * PackageSampleBytes == recvPos) {
      ans = RESULT_OK;
      break;
    }
  }

  if (package_Sample_Num * PackageSampleBytes != recvPos) {
    return RESULT_FAIL;
  }

  return ans;
}

result_t YDlidarDriver::waitPackage(node_info *node, uint32_t timeout) {
  (*node).index = 255;
  (*node).scan_frequence  = 0;
  (*node).error_package = 0;
  (*node).debugInfo = 0xff;

  if (package_Sample_Index == 0) {
    uint8_t  *packageBuffer = (m_intensities) ? (isTOFLidar(m_LidarType) ?
                              (uint8_t *)&tof_package.package_Head : (uint8_t *)&package.package_Head) :
                              (uint8_t *)&packages.package_Head;
    result_t ans = parseResponseHeader(packageBuffer, timeout);

    if (!IS_OK(ans)) {
      return ans;
    }

    ans = parseResponseScanData(packageBuffer, timeout);

    if (!IS_OK(ans)) {
      return ans;
    }

    calcuteCheckSum(node);
    calcutePackageCT();
  }

  parseNodeDebugFromBuffer(node);
  parseNodeFromeBuffer(node);
  return RESULT_OK;
}

void YDlidarDriver::calcuteCheckSum(node_info *node) {
  CheckSumCal ^= SampleNumlAndCTCal;
  CheckSumCal ^= LastSampleAngleCal;

  if (CheckSumCal != CheckSum) {
    CheckSumResult = false;
    has_package_error = true;
    (*node).error_package = 1;
  } else {
    CheckSumResult = true;
  }
}

void YDlidarDriver::calcutePackageCT() {
  if (m_intensities) {
    if (isTOFLidar(m_LidarType)) {
      package_CT = tof_package.package_CT;
      nowPackageNum = tof_package.nowPackageNum;
    } else {
      package_CT = package.package_CT;
      nowPackageNum = package.nowPackageNum;
    }
  } else {
    package_CT = packages.package_CT;
    nowPackageNum = packages.nowPackageNum;
  }
}

void YDlidarDriver::parseNodeDebugFromBuffer(node_info *node) {
  if ((package_CT & 0x01) == CT_Normal) {
    (*node).sync_flag = Node_NotSync;
    (*node).debugInfo = 0xff;

    if (!has_package_error) {
      if (package_Sample_Index == 0) {
        package_index++;
        (*node).debugInfo = (package_CT >> 1);
        (*node).index = package_index;
      }
    } else {
      (*node).error_package = 1;
      (*node).index = 255;
      package_index = 0xff;
    }
  } else {
    (*node).sync_flag = Node_Sync;
    package_index = 0;

    if (CheckSumResult) {
      has_package_error = false;
      (*node).index = package_index;
      (*node).debugInfo = (package_CT >> 1);
      (*node).scan_frequence  = scan_frequence;
    }
  }
}

void YDlidarDriver::parseNodeFromeBuffer(node_info *node) {
  int32_t  AngleCorrectForDistance    = 0;
  (*node).sync_quality = Node_Default_Quality;
  (*node).delay_time = 0;
  (*node).stamp = 0;

  if (CheckSumResult) {
    if (m_intensities) {
      if (isTriangleLidar(m_LidarType)) {
        (*node).sync_quality = ((uint16_t)((
                                             package.packageSample[package_Sample_Index].PakageSampleDistance & 0x03) <<
                                           LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT) |
                                (package.packageSample[package_Sample_Index].PakageSampleQuality));
        (*node).distance_q2 =
          package.packageSample[package_Sample_Index].PakageSampleDistance & 0xfffc;
      } else {
        (*node).sync_quality =
          tof_package.packageSample[package_Sample_Index].PakageSampleQuality;
        (*node).distance_q2 =
          tof_package.packageSample[package_Sample_Index].PakageSampleDistance;
      }
    } else {
      (*node).distance_q2 = packages.packageSampleDistance[package_Sample_Index];

      if (isTriangleLidar(m_LidarType)) {
        (*node).sync_quality = ((uint16_t)(0xfc |
                                           packages.packageSampleDistance[package_Sample_Index] &
                                           0x0003)) << LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
      }

    }

    if ((*node).distance_q2 != 0) {
      if (isOctaveLidar(model)) {
        AngleCorrectForDistance = (int32_t)(((atan(((21.8 * (155.3 - ((
                                                *node).distance_q2 / 2.0))) / 155.3) / ((
                                                    *node).distance_q2 / 2.0))) * 180.0 / 3.1415) * 64.0);
      } else  {
        if (isTriangleLidar(m_LidarType)) {
          AngleCorrectForDistance = (int32_t)(((atan(((21.8 * (155.3 - ((
                                                  *node).distance_q2 / 4.0))) / 155.3) / ((
                                                      *node).distance_q2 / 4.0))) * 180.0 / 3.1415) * 64.0);
        }
      }

      m_InvalidNodeCount++;
    } else {
      AngleCorrectForDistance = 0;
    }

    float sampleAngle = IntervalSampleAngle * package_Sample_Index;

    if ((FirstSampleAngle + sampleAngle +
         AngleCorrectForDistance) < 0) {
      (*node).angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + sampleAngle +
                                    AngleCorrectForDistance + 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
                                  LIDAR_RESP_MEASUREMENT_CHECKBIT;
    } else {
      if ((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) > 23040) {
        (*node).angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + sampleAngle +
                                      AngleCorrectForDistance - 23040)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
                                    LIDAR_RESP_MEASUREMENT_CHECKBIT;
      } else {
        (*node).angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + sampleAngle +
                                      AngleCorrectForDistance)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) +
                                    LIDAR_RESP_MEASUREMENT_CHECKBIT;
      }
    }
  } else {
    (*node).sync_flag       = Node_NotSync;
    (*node).sync_quality    = Node_Default_Quality;
    (*node).angle_q6_checkbit = LIDAR_RESP_MEASUREMENT_CHECKBIT;
    (*node).distance_q2      = 0;
    (*node).scan_frequence  = 0;
  }

  package_Sample_Index++;

  if (package_Sample_Index >= nowPackageNum) {
    package_Sample_Index = 0;
    CheckSumResult = false;
  }
}

result_t YDlidarDriver::waitScanData(node_info *nodebuffer, size_t &count,
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

    if (node.sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT) {
      size_t size = _serial->available();
      uint64_t delayTime = 0;

      if (size > PackagePaidBytes) {
        size_t packageNum = 0;
        size_t Number = 0;
        size_t PackageSize = TrianglePackageDataSize;
        packageNum = size / PackageSize;
        Number = size % PackageSize;
        delayTime = packageNum * (PackageSize - PackagePaidBytes) * m_PointTime / 2;

        if (Number > PackagePaidBytes) {
          delayTime += m_PointTime * ((Number - PackagePaidBytes) / 2);
        }
      }

      nodebuffer[recvNodeCount - 1].delay_time = size * trans_delay + delayTime;
      nodebuffer[recvNodeCount - 1].scan_frequence = node.scan_frequence;
      nodebuffer[recvNodeCount - 1].stamp = getTime();
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


result_t YDlidarDriver::grabScanData(node_info *nodebuffer, size_t &count,
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
      size_t size_to_copy = min(count, scan_node_count);
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


result_t YDlidarDriver::ascendScanData(node_info *nodebuffer, size_t count) {
  float inc_origin_angle = (float)360.0 / count;
  int i = 0;

  for (i = 0; i < (int)count; i++) {
    if (nodebuffer[i].distance_q2 == 0) {
      continue;
    } else {
      while (i != 0) {
        i--;
        float expect_angle = (nodebuffer[i + 1].angle_q6_checkbit >>
                              LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) /
                             64.0f - inc_origin_angle;

        if (expect_angle < 0.0f) {
          expect_angle = 0.0f;
        }

        uint16_t checkbit = nodebuffer[i].angle_q6_checkbit &
                            LIDAR_RESP_MEASUREMENT_CHECKBIT;
        nodebuffer[i].angle_q6_checkbit = (((uint16_t)(expect_angle * 64.0f)) <<
                                           LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
      }

      break;
    }
  }

  if (i == (int)count) {
    return RESULT_FAIL;
  }

  for (i = (int)count - 1; i >= 0; i--) {
    if (nodebuffer[i].distance_q2 == 0) {
      continue;
    } else {
      while (i != ((int)count - 1)) {
        i++;
        float expect_angle = (nodebuffer[i - 1].angle_q6_checkbit >>
                              LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) /
                             64.0f + inc_origin_angle;

        if (expect_angle > 360.0f) {
          expect_angle -= 360.0f;
        }

        uint16_t checkbit = nodebuffer[i].angle_q6_checkbit &
                            LIDAR_RESP_MEASUREMENT_CHECKBIT;
        nodebuffer[i].angle_q6_checkbit = (((uint16_t)(expect_angle * 64.0f)) <<
                                           LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
      }

      break;
    }
  }

  float frontAngle = (nodebuffer[0].angle_q6_checkbit >>
                      LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;

  for (i = 1; i < (int)count; i++) {
    if (nodebuffer[i].distance_q2 == 0) {
      float expect_angle =  frontAngle + i * inc_origin_angle;

      if (expect_angle > 360.0f) {
        expect_angle -= 360.0f;
      }

      uint16_t checkbit = nodebuffer[i].angle_q6_checkbit &
                          LIDAR_RESP_MEASUREMENT_CHECKBIT;
      nodebuffer[i].angle_q6_checkbit = (((uint16_t)(expect_angle * 64.0f)) <<
                                         LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
    }
  }

  size_t zero_pos = 0;
  float pre_degree = (nodebuffer[0].angle_q6_checkbit >>
                      LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;

  for (i = 1; i < (int)count ; ++i) {
    float degree = (nodebuffer[i].angle_q6_checkbit >>
                    LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;

    if (zero_pos == 0 && (pre_degree - degree > 180)) {
      zero_pos = i;
      break;
    }

    pre_degree = degree;
  }

  node_info *tmpbuffer = new node_info[count];

  for (i = (int)zero_pos; i < (int)count; i++) {
    tmpbuffer[i - zero_pos] = nodebuffer[i];
  }

  for (i = 0; i < (int)zero_pos; i++) {
    tmpbuffer[i + (int)count - zero_pos] = nodebuffer[i];
  }

  memcpy(nodebuffer, tmpbuffer, count * sizeof(node_info));
  delete[] tmpbuffer;

  return RESULT_OK;
}

/************************************************************************/
/* get health state of lidar                                            */
/************************************************************************/
result_t YDlidarDriver::getHealth(device_health &health, uint32_t timeout) {
  result_t ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  if (m_SingleChannel) {
    if (get_device_health_success) {
      health = this->health_;
      return RESULT_OK;
    }

    health.error_code = 0;
    health.status = 0;
    return RESULT_OK;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_DEVICE_HEALTH)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVHEALTH) {
      return RESULT_FAIL;
    }

    if (response_header.size < sizeof(device_health)) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&health), sizeof(health));
  }
  return RESULT_OK;
}

/************************************************************************/
/* get device info of lidar                                             */
/************************************************************************/
result_t YDlidarDriver::getDeviceInfo(device_info &info, uint32_t timeout) {
  result_t  ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  if (m_SingleChannel) {
    if (get_device_info_success) {
      info = this->info_;
      return RESULT_OK;
    }

    info.model = YDLIDAR_S2;
    info.firmware_version = 0;
    info.hardware_version = 0;
    return RESULT_OK;
  }

//  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_DEVICE_INFO)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size < sizeof(device_info)) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&info), sizeof(info));
    model = info.model;
  }

  return RESULT_OK;
}

/************************************************************************/
/* the set to signal quality                                            */
/************************************************************************/
void YDlidarDriver::setIntensities(const bool &isintensities) {
  if (m_intensities != isintensities) {
    if (globalRecvBuffer) {
      delete[] globalRecvBuffer;
      globalRecvBuffer = NULL;
    }

    if (isintensities && isTOFLidar(m_LidarType)) {
      globalRecvBuffer = new uint8_t[sizeof(tof_node_package)];
    } else {
      globalRecvBuffer = new uint8_t[isintensities ? sizeof(node_package) : sizeof(
                                       node_packages)];
    }
  }

  m_intensities = isintensities;

  if (m_intensities) {
    if (isTOFLidar(m_LidarType)) {
      PackageSampleBytes = 4;
    } else {
      PackageSampleBytes = 3;
    }
  } else {
    PackageSampleBytes = 2;
  }
}
/**
* @brief 设置雷达异常自动重新连接 \n
* @param[in] enable    是否开启自动重连:
*     true	开启
*	  false 关闭
*/
void YDlidarDriver::setAutoReconnect(const bool &enable) {
  isAutoReconnect = enable;
}


void YDlidarDriver::checkTransDelay() {
  //calc stamp
  trans_delay = _serial->getByteTime();
  sample_rate = lidarModelDefaultSampleRate(model) * 1000;

  switch (model) {
    case YDLIDAR_G4://g4
    case YDLIDAR_G5:
    case YDLIDAR_G4PRO:
    case YDLIDAR_F4PRO:
    case YDLIDAR_G6://g6
    case YDLIDAR_G7:
    case YDLIDAR_TG15:
    case YDLIDAR_TG30:
    case YDLIDAR_TG50:
      if (m_sampling_rate == -1) {
        sampling_rate _rate;
        _rate.rate = 0;
        getSamplingRate(_rate);
        m_sampling_rate = _rate.rate;
      }

      sample_rate =  ConvertLidarToUserSmaple(model, m_sampling_rate);
      sample_rate *= 1000;
      break;

    case YDLIDAR_G2C:
      sample_rate = 4000;
      break;

    case YDLIDAR_G1:
      sample_rate = 9000;
      break;

    case YDLIDAR_G4C:
      sample_rate = 4000;
      break;

    default:
      break;
  }

  m_PointTime = 1e9 / sample_rate;
}

/************************************************************************/
/*  start to scan                                                       */
/************************************************************************/
result_t YDlidarDriver::startScan(bool force, uint32_t timeout) {
  result_t ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  if (m_isScanning) {
    return RESULT_OK;
  }

  stop();
  checkTransDelay();
  flushSerial();
  delay(30);
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(force ? LIDAR_CMD_FORCE_SCAN : LIDAR_CMD_SCAN)) !=
        RESULT_OK) {
      return ans;
    }

    if (!m_SingleChannel) {

      lidar_ans_header response_header;

      if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
        return ans;
      }

      if (response_header.type != LIDAR_ANS_TYPE_MEASUREMENT) {
        return RESULT_FAIL;
      }

      if (response_header.size < 5) {
        return RESULT_FAIL;
      }
    }

    ans = this->createThread();
  }

  if (isSupportMotorCtrl(model)) {
    startMotor();
  }

  return ans;
}


result_t YDlidarDriver::stopScan(uint32_t timeout) {
  UNUSED(timeout);

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  ScopedLocker l(_lock);
  sendCommand(LIDAR_CMD_FORCE_STOP);
  delay(5);
  sendCommand(LIDAR_CMD_STOP);
  delay(5);
  return RESULT_OK;
}

result_t YDlidarDriver::createThread() {
  _thread = CLASS_THREAD(YDlidarDriver, cacheScanData);

  if (_thread.getHandle() == 0) {
    m_isScanning = false;
    return RESULT_FAIL;
  }

  m_isScanning = true;
  return RESULT_OK;
}


result_t YDlidarDriver::startAutoScan(bool force, uint32_t timeout) {
  result_t ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  flushSerial();
  delay(10);
  {

    ScopedLocker l(_lock);

    if ((ans = sendCommand(force ? LIDAR_CMD_FORCE_SCAN : LIDAR_CMD_SCAN)) !=
        RESULT_OK) {
      return ans;
    }

    if (!m_SingleChannel) {
      lidar_ans_header response_header;

      if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
        return ans;
      }

      if (response_header.type != LIDAR_ANS_TYPE_MEASUREMENT) {
        return RESULT_FAIL;
      }

      if (response_header.size < 5) {
        return RESULT_FAIL;
      }
    }

  }

  if (isSupportMotorCtrl(model)) {
    startMotor();
  }

  return RESULT_OK;
}

/************************************************************************/
/*   stop scan                                                   */
/************************************************************************/
result_t YDlidarDriver::stop() {
  if (isAutoconnting) {
    isAutoReconnect = false;
    m_isScanning = false;
  }

  disableDataGrabbing();
  stopScan();

  if (isSupportMotorCtrl(model)) {
    stopMotor();
  }

  return RESULT_OK;
}

/************************************************************************/
/*  reset device                                                        */
/************************************************************************/
result_t YDlidarDriver::reset(uint32_t timeout) {
  UNUSED(timeout);
  result_t ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  ScopedLocker l(_lock);

  if ((ans = sendCommand(LIDAR_CMD_RESET)) != RESULT_OK) {
    return ans;
  }

  return RESULT_OK;
}

/************************************************************************/
/*  startMotor                                                          */
/************************************************************************/
result_t YDlidarDriver::startMotor() {
  ScopedLocker l(_lock);

  if (m_SupportMotorDtrCtrl) {
    setDTR();
    delay(500);
  } else {
    clearDTR();
    delay(500);
  }

  return RESULT_OK;
}

/************************************************************************/
/*  stopMotor                                                           */
/************************************************************************/
result_t YDlidarDriver::stopMotor() {
  ScopedLocker l(_lock);

  if (m_SupportMotorDtrCtrl) {
    clearDTR();
    delay(500);
  } else {
    setDTR();
    delay(500);
  }

  return RESULT_OK;
}

/************************************************************************/
/* get the current scan frequency of lidar                              */
/************************************************************************/
result_t YDlidarDriver::getScanFrequency(scan_frequency &frequency,
    uint32_t timeout) {
  result_t  ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_AIMSPEED)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != sizeof(frequency)) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
  }
  return RESULT_OK;
}

/************************************************************************/
/* add the scan frequency by 1Hz each time                              */
/************************************************************************/
result_t YDlidarDriver::setScanFrequencyAdd(scan_frequency &frequency,
    uint32_t timeout) {
  result_t  ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_ADD)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != sizeof(frequency)) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
  }
  return RESULT_OK;
}

/************************************************************************/
/* decrease the scan frequency by 1Hz each time                         */
/************************************************************************/
result_t YDlidarDriver::setScanFrequencyDis(scan_frequency &frequency,
    uint32_t timeout) {
  result_t  ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_DIS)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != sizeof(frequency)) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
  }
  return RESULT_OK;
}

/************************************************************************/
/* add the scan frequency by 0.1Hz each time                            */
/************************************************************************/
result_t YDlidarDriver::setScanFrequencyAddMic(scan_frequency &frequency,
    uint32_t timeout) {
  result_t  ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_ADDMIC)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != sizeof(frequency)) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
  }
  return RESULT_OK;
}

/************************************************************************/
/* decrease the scan frequency by 0.1Hz each time                       */
/************************************************************************/
result_t YDlidarDriver::setScanFrequencyDisMic(scan_frequency &frequency,
    uint32_t timeout) {
  result_t  ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_DISMIC)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != sizeof(frequency)) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
  }
  return RESULT_OK;
}

/************************************************************************/
/*  get the sampling rate of lidar                                      */
/************************************************************************/
result_t YDlidarDriver::getSamplingRate(sampling_rate &rate, uint32_t timeout) {
  result_t  ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_SAMPLING_RATE)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != sizeof(rate)) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&rate), sizeof(rate));
    m_sampling_rate = rate.rate;
  }
  return RESULT_OK;
}

/************************************************************************/
/*  the set to sampling rate                                            */
/************************************************************************/
result_t YDlidarDriver::setSamplingRate(sampling_rate &rate, uint32_t timeout) {
  result_t  ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_SAMPLING_RATE)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != sizeof(rate)) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&rate), sizeof(rate));
  }
  return RESULT_OK;
}

/************************************************************************/
/*  the get to zero offset angle                                        */
/************************************************************************/
result_t YDlidarDriver::getZeroOffsetAngle(offset_angle &angle,
    uint32_t timeout) {
  result_t  ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_OFFSET_ANGLE)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size < sizeof(offset_angle)) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&angle), sizeof(angle));
  }
  return RESULT_OK;
}

/************************************************************************/
/*  the set to heart beat                                               */
/************************************************************************/
result_t YDlidarDriver::setScanHeartbeat(scan_heart_beat &beat,
    uint32_t timeout) {
  result_t  ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_SET_HEART_BEAT)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size != 1) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&beat), sizeof(beat));
  }
  return RESULT_OK;
}

/************************************************************************/
/*  the get to zero offset angle                                        */
/************************************************************************/
result_t YDlidarDriver::getAutoZeroOffsetAngle(offset_angle &angle,
    uint32_t timeout) {
  result_t  ans;

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  flushSerial();
  {
    ScopedLocker l(_lock);

    if ((ans = sendCommand(LIDAR_CMD_GET_OFFSET_ANGLE)) != RESULT_OK) {
      return ans;
    }

    lidar_ans_header response_header;

    if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
      return ans;
    }

    if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
      return RESULT_FAIL;
    }

    if (response_header.size < sizeof(offset_angle)) {
      return RESULT_FAIL;
    }

    if (waitForData(response_header.size, timeout) != RESULT_OK) {
      return RESULT_FAIL;
    }

    getData(reinterpret_cast<uint8_t *>(&angle), sizeof(angle));
  }
  return RESULT_OK;
}



std::string YDlidarDriver::getSDKVersion() {
  return YDLIDAR_SDK_VERSION_STR;
}
std::map<std::string, std::string>  YDlidarDriver::lidarPortList() {
  std::vector<PortInfo> lst = list_ports();
  std::map<std::string, std::string> ports;

  for (std::vector<PortInfo>::iterator it = lst.begin(); it != lst.end(); it++) {
    std::string port = "ydlidar" + (*it).device_id;
    ports[port] = (*it).port;
  }

  return ports;
}

}
