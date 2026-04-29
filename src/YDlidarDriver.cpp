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
#include <math.h>
#include <algorithm>
#include "core/common/ydlidar_help.h"
#include "core/common/YdDataStream.h"
#include "core/serial/common.h"
#include "core/serial/serial.h"
#include "core/network/ActiveSocket.h"
#include "YDlidarDriver.h"
#include "ydlidar_config.h"

using namespace impl;

namespace ydlidar
{
using namespace core::serial;
using namespace core::common;
using namespace core::network;

YDlidarDriver::YDlidarDriver(uint8_t type) 
 : _serial(NULL)
{
  m_isConnected = false;
  m_isScanning = false;
  // 串口配置参数
  m_intensities = false;
  m_intensityBit = NODE_QUAL0;
  isAutoReconnect = true;
  isAutoconnting = false;
  m_baudrate = 230400;
  m_SupportMotorDtrCtrl = true;
  m_HeartBeat = false;
  sample_rate = 5000;
  m_PointTime = 1e9 / 5000;
  trans_delay = 0;

  m_sampling_rate = -1;
  model = YDLIDAR_S2PRO;
  retryCount = 0;
  m_SingleChannel = false;
  m_LidarType = TYPE_TRIANGLE;
  m_DeviceType = type;

  scan_node_buf = new node_info[MAX_SCAN_NODES];
  scan_node_count = 0;

  m_HasDeviceInfo = EPT_None;
  m_heartbeat_ts = getms();
  m_BlockRevSize = 0;
}

YDlidarDriver::~YDlidarDriver()
{
  stopThread();
  {
    ScopedLocker l(_cmd_lock);
    if (_serial)
    {
      if (_serial->isOpen())
      {
        _serial->flush();
        _serial->closePort();
      }
    }
    if (_serial)
    {
      delete _serial;
      _serial = NULL;
    }
  }

  {
    ScopedLocker l(_lock);
    if (scan_node_buf)
    {
      delete[] scan_node_buf;
      scan_node_buf = NULL;
    }
  }
}

result_t YDlidarDriver::connect(const char *port_path, uint32_t baudrate)
{
  m_baudrate = baudrate;
  m_port = string(port_path);

  {
    ScopedLocker l(_cmd_lock);
    if (!_serial)
    {
      if (YDLIDAR_TYPE_TCP == m_DeviceType)
      {
        _serial = new CActiveSocket();
      }
      else if (YDLIDAR_TYPE_UDP == m_DeviceType)
      {
        _serial = new CActiveSocket(CSimpleSocket::SocketTypeUdp);
      }
      else
      {
        _serial = new serial::Serial(
          m_port, m_baudrate,
          serial::Timeout::simpleTimeout(DEFAULT_TIMEOUT/2));
      }
      _serial->bindport(port_path, baudrate);
    }
    if (!_serial->open())
    {
      setDriverError(NotOpenError);
      return RESULT_FAIL;
    }
    if (YDLIDAR_TYPE_SERIAL != m_DeviceType)
    {
      CActiveSocket* s = dynamic_cast<CActiveSocket*>(_serial);
      if (s)
        s->SetReceiveTimeout(SDK_TIMEOUT / SDK_UNIT1000, 0);
    }
      
    m_isConnected = true;
  }

  return RESULT_OK;
}

const char *YDlidarDriver::DescribeError(bool isTCP)
{
  if (_serial)
    return _serial->DescribeError();
  return nullptr;
}

void YDlidarDriver::setDTR()
{
  if (!m_isConnected)
  {
    return;
  }

  if (_serial)
  {
    _serial->setDTR(1);
  }
}

void YDlidarDriver::clearDTR()
{
  if (!m_isConnected)
  {
    return;
  }

  if (_serial)
  {
    _serial->setDTR(0);
  }
}

void YDlidarDriver::clearRead()
{
  bool need = false;
  {
    ScopedLocker l(_cmd_lock);
    if (!isOpen())
      return;
    if (_serial->available())
      need = true;
  }
  if (need)
    readAll();
}

void YDlidarDriver::disconnect()
{
  isAutoReconnect = false;

  if (!m_isConnected)
    return;

  stop();
  delay(10);

  ScopedLocker l(_cmd_lock);
  if (_serial)
  {
    if (_serial->isOpen())
    {
      _serial->closePort();
    }
  }

  m_isConnected = false;
}

void YDlidarDriver::stopThread()
{
  m_isScanning = false;
  _dataEvent.set();
  if (m_thread)
  {
    if (m_thread->joinable())
      m_thread->join();
    delete m_thread;
    m_thread = nullptr;
  }
}

bool YDlidarDriver::isscanning() const
{
  return m_isScanning;
}

bool YDlidarDriver::isconnected() const
{
  return m_isConnected;
}

bool YDlidarDriver::sendCmd(uint8_t cmd)
{
  clearRead();

  std::vector<uint8_t> d;
  d.push_back(PHA5);
  d.push_back(cmd);
  return IS_OK(sendData((const uint8_t*)d.data(), d.size()));
}

result_t YDlidarDriver::sendData(const uint8_t *data, size_t size)
{
  if (data == NULL || size == 0)
    return RESULT_FAIL;
  ScopedLocker l(_cmd_lock);
  if (!isOpen())
    return RESULT_FAIL;

  size_t r = 0;
  while (size)
  {
    r = _serial->writeData(data, size);
    if (r < 1)
    {
      return RESULT_FAIL;
    }
    if (m_Debug)
      debugh(data, r, "[send] ");

    size -= r;
    data += r;
  }

  return RESULT_OK;
}

result_t YDlidarDriver::getData(uint8_t *data, size_t size)
{
  if (!m_isConnected)
    return RESULT_FAIL;

  size_t r = 0;
  while (size)
  {
    r = _serial->readData(data, size);
    if (r < 1)
    {
      return RESULT_FAIL;
    }

    if (m_Debug)
    {
      debugh(data, r, "[recv] ");
    }

    size -= r;
    data += r;
  }

  return RESULT_OK;
}

result_t YDlidarDriver::waitForData(size_t data_count, uint32_t timeout,
                                    size_t *returned_size)
{
  size_t length = 0;

  if (returned_size == NULL)
  {
    returned_size = (size_t *)&length;
  }

  return (result_t)_serial->waitfordata(data_count, timeout, returned_size);
}

result_t YDlidarDriver::checkAutoConnecting(bool serialError)
{
  result_t ans = RESULT_FAIL;
  m_InvalidNodeCount = 0;
  if (m_driverErrno != BlockError)
    setDriverError(TimeoutError);

  while (isAutoReconnect && isscanning())
  {
    //先断开串口连接
    {
      ScopedLocker l(_cmd_lock);
      if (_serial)
      {
        if (_serial->isOpen())
        {
          //根据串口缓存数据大小判断错误类型（有点离谱）
          size_t buffer_size = _serial->available();
          m_BufferSize += buffer_size;
          if (m_BufferSize && m_BufferSize % 7 == 0)
          {
            setDriverError(BlockError);
          }
          else
          {
            if (buffer_size > 0 || m_BufferSize > 0)
            {
              if (m_driverErrno != BlockError)
              {
                setDriverError(TrembleError);
              }
            }
            else
            {
              setDriverError(NotBufferError);
            }
          }

          m_isConnected = false;
          _serial->closePort();
        }
      }
    }

    //重连串口
    while (isscanning() &&
            connect(m_port.c_str(), m_baudrate) != RESULT_OK)
    {
      setDriverError(NotOpenError);
      delay(300);
    }
    //如果未连接串口或者已停止扫描，则返回
    if (!isconnected() || !isscanning())
    {
      return RESULT_FAIL;
    }

    //尝试启动雷达，如果成功则返回成功，否则进入下一次循环
    {
      delay(50);

      if (!m_SingleChannel)
      {
        device_info di;
        ans = getDeviceInfo(di, 1000);
        if (!IS_OK(ans))
        {
          stopScan();
          ans = getDeviceInfo(di);
        }
        if (!IS_OK(ans))
        {
          setDriverError(DeviceNotFoundError);
          continue;
        }
      }

      ans = startAutoScan();
      if (IS_OK(ans))
      {
        if (getDriverError() == DeviceNotFoundError)
        {
          setDriverError(NoError);
        }
        return ans;
      }
      else
      {
        setDriverError(DeviceNotFoundError);
      }
    }
  }

  return RESULT_FAIL;
}

result_t YDlidarDriver::autoHeartBeat()
{
  if (!m_isConnected)
    return RESULT_FAIL;

  return sendCmd(LIDAR_CMD_SCAN) ? RESULT_OK : RESULT_FAIL;
}

void YDlidarDriver::keepHeartBeat()
{
  if (m_HeartBeat)
  {
    uint32_t et = getms();
    if (et - m_heartbeat_ts > DEFAULT_HEART_BEAT)
    {
      autoHeartBeat();
      m_heartbeat_ts = et;
    }
  }
}

// 解析扫描数据线程主函数
int YDlidarDriver::cacheScanData()
{
  result_t ans = RESULT_FAIL;
  int timeoutCount = 0;
  retryCount = 0;
  m_BufferSize = 0;
  m_heartbeat_ts = getms();
  m_zeroTime = getms(); //零位包时间
  m_data.clear();
  m_datas.clear();

  m_isScanning = true; //标记扫描线程开始
  while (m_isScanning)
  {
    ans = parseData(TIMEOUT_500);
    if (!IS_OK(ans))
    {
      if (timeoutCount > DEFAULT_TIMEOUT_COUNT)
      {
        if (!isAutoReconnect)
        {
          error("Exit scanning thread");
          m_isScanning = false;
          return RESULT_FAIL;
        }
        else
        {
          ans = checkAutoConnecting(IS_FAIL(ans));
          if (IS_OK(ans))
          {
            timeoutCount = 0;
          }
          else
          {
            m_isScanning = false;
            return RESULT_FAIL;
          }
        }
      }
      else
      {
        timeoutCount++;
        if (m_driverErrno == NoError)
          setDriverError(TimeoutError);
        error("Timeout count: %d", timeoutCount);
      }
    }
    else
    {
      timeoutCount = 0;
      retryCount = 0;
      m_BufferSize = 0;
      setDriverError(NoError);
    }
    //判断是否需要发心跳
    keepHeartBeat();
  }
  m_isScanning = false;

  return RESULT_OK;
}

// 获取雷达模组抛出的设备信息、健康信息
result_t YDlidarDriver::waitDevicePackage(uint32_t timeout)
{
  result_t ret = RESULT_FAIL;

  std::vector<uint8_t> d;
  if (!waitResp(LIDAR_ANS_TYPE_DEVINFO, d, timeout))
    return ret;
  if (d.size() < DEVICEINFOSIZE)
    return ret;
  device_info di;
  memcpy(reinterpret_cast<uint8_t*>(&di), d.data(), DEVICEINFOSIZE);
  // 没有启用底板时才使用模组的雷达型号码
  if (!m_Bottom)
      model = di.model;
  m_HasDeviceInfo |= EPT_Module;
  m_ModuleDevInfo = di;
  printfDeviceInfo(di, EPT_Module);

  clearRead();
  return RESULT_OK;
}

void YDlidarDriver::checkBlockStatus(uint8_t c)
{
  switch (m_BlockRevSize)
  {
  case 0:
    if (c == PHA5)
    {
      m_BlockRevSize++;
    }
    break;
  case 1:
    if (c == PH5A)
    {
      setDriverError(BlockError);
      m_BlockRevSize = 0;
    }
    break;
  default:
    break;
  }
}

result_t YDlidarDriver::parseData(uint32_t timeout)
{
  uint32_t st = getms();
  uint32_t dt = 0;
  int recvPos = 0;
  int trunPos = 0; //截断位置
  int count = 0; //点数
  uint8_t zero = 0; //零位包标记
  uint16_t sa = 0; //起始角
  uint16_t ea = 0; //结束角
  result_t ret = RESULT_TIMEOUT;

  m_BlockRevSize = 0;

  while ((dt = getms() - st) <= timeout)
  {
    std::vector<uint8_t> d = readAll(timeout - dt);
    if (d.empty())
      return ret;
    m_data.insert(m_data.end(), d.begin(), d.end());
    size_t size = m_data.size();
    for (size_t i = 0; i < size; ++i)
    {
      uint8_t c = m_data.at(i);
      switch (recvPos)
      {
      case 0:
        if (c != PH1)
        {
          checkBlockStatus(c);
          continue;
        }
        break;
      case 1:
      {
        if (c == PH2)
        {
          if (m_driverErrno == BlockError)
            setDriverError(NoError);
        }
        else if (c == PH1) // 防止出现连续0xAA
        {
          continue;
        }
        else if (c == PH3) // 时间戳标识
        {
          recvPos = 0;
          size_t lastPos = i - 1;
          // 解析时间戳（共8个字节）
          int remainSize = STAMPPACKSIZE - (size - i + 1); // 计算剩余应读字节数
          if (remainSize > 0)
          {
            //如果剩余数据量不足，则跳出循环等待数据
            i = size;
            continue;
          }
          else
          {
            i += 6;
          }
          // 时间戳校验和检测
          uint8_t csc = 0; // 计算校验和
          uint8_t csr = 0; // 实际校验和
          for (int j = 0; j < STAMPPACKSIZE; ++j)
          {
            if (j == 2)
              csr = m_data[lastPos + j];
            else
              csc ^= m_data[lastPos + j];
          }
          if (csc != csr)
          {
            error("Checksum error c[0x%02X] != r[0x%02X]", csc, csr);
          }
          else
          {
            stamp_package sp;
            memcpy(&sp, &m_data[lastPos], STAMPPACKSIZE);
            stamp = uint64_t(sp.stamp) * 1000000; // 毫秒转纳秒需要×1000000
            // debug("stamp: 0x%"PRIx64" -> 0x%"PRIx64"", sp.stamp, stamp);
            // 测试扫描时长
            // static uint32_t s_scanTime = 0;
            // if (s_scanTime > 0)
            // {
            //     uint32_t dt = sp.stamp - s_scanTime;
            //     if (dt < 44 || dt > 57)
            //     {
                    // error("单帧时长[%u]ms超出标准[%u~%u]",
                    //     dt, 44, 57);
            //     }
            // }
            // s_scanTime = sp.stamp;
          }
          trunPos = i + 1;
          continue;
        }
        else
        {
          recvPos = 0;
          continue;
        }
        break;
      }
      case 2:
        zero = c & LIDAR_RESP_SYNCBIT; // 是否是零位包标识
        if (zero)
        {
          if (m_Debug)
            debug("Zero package");
        }
        break;
      case 3:
        if (c > TRI_PACKMAXNODES)
        {
            warn("Current pack point count %d too big", c);
            recvPos = 0;
            continue;
        }
        count = c * getPointBytes();
        break;
      case 4:
        if (c & LIDAR_RESP_CHECKBIT)
        {
          sa = c;
        }
        else
        {
          recvPos = 0;
          continue;
        }
        break;
      case 5:
        sa += uint16_t(c << 8);
        sa = sa >> 1;
        break;
      case 6:
        if (c & LIDAR_RESP_CHECKBIT)
        {
          ea = c;
        }
        else
        {
          recvPos = 0;
          continue;
        }
        break;
      case 7:
        ea += uint16_t(c << 8);
        ea = ea >> 1;
        break;
      case 8:
        break;
      case 9:
        break;
      }
      recvPos++;

      //如果解析到点云包头
      if (recvPos == TRI_PACKHEADSIZE)
      {
        recvPos = 0;
        int s = i + 1 + count;
        if (s <= size)
        {
          if (m_Debug)
            debug("Current angle range[%.02f°~%.02f°]",
              sa / SDK_UNIT64, ea / SDK_UNIT64);
          std::vector<uint8_t> data(m_data.begin() + i + 1 - TRI_PACKHEADSIZE, 
            m_data.begin() + s);
          //计算校验和
          if (calcCheckSum(data))
          {
            ret = RESULT_OK;
            if (zero)
            {
              zero = false;
              parsePoints();
              m_datas.clear();
              m_zeroTime = getms();
            }
            else
            {
              //如果长时间未获取到零位包则需要清空数据
              if (getms() - m_zeroTime > TIMEOUT_2S)
                m_datas.clear();
            }
            m_datas.push_back(Pack(data, stamp));
          }
          trunPos = s;
          i += count;
        }
        else
        {
          break; //跳出循环等待足够数据
        }
      }
    }
    if (IS_OK(ret))
      break;
  }

  if (trunPos) // 截断数据
  {
    if (trunPos < m_data.size())
      m_data = std::vector<uint8_t>(m_data.begin() + trunPos, m_data.end());
    else
      m_data.clear();
  }
  else if (m_data.size() > SDK_SIZE1K) //数据量过大时清空
  {
    warn("Current invalid data %u too big, has cleared", m_data.size());
    m_data.clear();
  }

  return ret;
}

result_t YDlidarDriver::grabScanData(
    node_info *nodes,
    size_t &count,
    uint32_t timeout)
{
  switch (_dataEvent.wait(timeout))
  {
  case Event::EVENT_TIMEOUT:
    count = 0;
    return RESULT_TIMEOUT;
  case Event::EVENT_OK:
  {
    ScopedLocker l(_lock);
    size_t size_to_copy = min(count, scan_node_count);
    memcpy(nodes, scan_node_buf, size_to_copy * sizeof(node_info));
    count = size_to_copy;
    scan_node_count = 0;
    return RESULT_OK;
  }
  default:
    count = 0;
    return RESULT_FAIL;
  }
}

result_t YDlidarDriver::getHealth(device_health &health, uint32_t timeout)
{
  result_t ans = RESULT_FAIL;
  if (!m_isConnected)
    return ans;

  if (m_SingleChannel)
  {
    health.error_code = 0;
    health.status = 0;
    return RESULT_OK;
  }

  //如果是双通雷达，需要先停止
  stop();
  {
    if (!sendCmd(LIDAR_CMD_GET_DEVICE_HEALTH))
      return ans;
    std::vector<uint8_t> d;
    if (!waitResp(LIDAR_ANS_TYPE_DEVHEALTH, d, timeout))
      return ans;
    if (d.size() < HEALTHINFOSIZE)
      return ans;
    memcpy(reinterpret_cast<uint8_t*>(&health), d.data(), HEALTHINFOSIZE);
  }
  return RESULT_OK;
}

result_t YDlidarDriver::getDeviceInfo(device_info &di, uint32_t timeout)
{
  result_t ans = RESULT_FAIL;
  if (!m_isConnected)
    return RESULT_FAIL;

  // 单通，仅能获取到模组设备信息
  if (m_SingleChannel)
  {
    // 获取启动时抛出的设备信息或每帧数据中的设备信息
    if (m_HasDeviceInfo & EPT_Module)
    {
      di = m_ModuleDevInfo;
      return RESULT_OK;
    }
    else
    {
      // 未获取到设备信息时，返回一个无效的设备信息
      di.model = YDLIDAR_S2;
      return RESULT_FAIL;
    }
  }
  // 双通，此处仅能获取到底板设备信息
  else
  {
    if (m_Bottom)
    {
      if (!sendCmd(LIDAR_CMD_GET_DEVICE_INFO))
        return ans;
      std::vector<uint8_t> d;
      if (!waitResp(LIDAR_ANS_TYPE_DEVINFO, d, timeout))
        return ans;
      if (d.size() < DEVICEINFOSIZE)
        return ans;
      memcpy(reinterpret_cast<uint8_t*>(&di), d.data(), DEVICEINFOSIZE);
      model = di.model;
      m_HasDeviceInfo |= EPT_Base;
      m_BaseDevInfo = di;
      ans = RESULT_OK;
    }
    return ans;
  }
}

bool YDlidarDriver::getDeviceInfoEx(device_info &info, int type)
{
  if ((type & EPT_Module) &&
      (m_HasDeviceInfo & EPT_Module))
  {
    info = m_ModuleDevInfo;
    return true;
  }
  else if ((type & EPT_Base) &&
            (m_HasDeviceInfo & EPT_Base))
  {
    info = m_BaseDevInfo;
    return true;
  }

  return false;
}

void YDlidarDriver::setIntensities(const bool &isintensities)
{
  m_intensities = isintensities;
}

void YDlidarDriver::setAutoReconnect(const bool &enable)
{
  isAutoReconnect = enable;
}

void YDlidarDriver::checkTransDelay()
{
  // calc stamp
  trans_delay = _serial->getByteTime();
  sample_rate = getDefaultSampleRate(model).front() * 1000;

  switch (model)
  {
  case YDLIDAR_G4: // g4
  case YDLIDAR_G5:
  case YDLIDAR_G4PRO:
  case YDLIDAR_F4PRO:
  case YDLIDAR_G6: // g6
  case YDLIDAR_G7:
  case YDLIDAR_TG15:
  case YDLIDAR_TG30:
  case YDLIDAR_TG50:
    if (m_sampling_rate == -1)
    {
      sampling_rate _rate;
      _rate.rate = 0;
      getSamplingRate(_rate);
      m_sampling_rate = _rate.rate;
    }

    sample_rate = ConvertLidarToUserSmaple(model, m_sampling_rate);
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

result_t YDlidarDriver::startScan(bool force, uint32_t timeout)
{
  result_t ret = RESULT_FAIL;
  if (!m_isConnected)
    return RESULT_FAIL;
  if (m_isScanning)
    return RESULT_OK;

  hasStamp = true;
  stop();
  checkTransDelay();
  delay(30);
  {
    // 不管单双通雷达都发送启动命令
    if (!sendCmd(force ? LIDAR_CMD_FORCE_SCAN : LIDAR_CMD_SCAN))
      return ret;
    if (!m_SingleChannel) //双通雷达
    {
      //双通雷达需要等待响应
      std::vector<uint8_t> d;
      if (!waitResp(LIDAR_ANS_TYPE_MEASUREMENT, d, timeout))
        return ret;
      if (d.size() < 5)
        return ret;
      //此处仅获取模组设备信息
      {
        waitDevicePackage(TIMEOUT_1S);
      }
    }

    // 创建数据解析线程
    ret = startThread();
  }

  if (isSupportMotorCtrl(model))
    startMotor();

  m_isScanning = true;

  return ret;
}

result_t YDlidarDriver::stopScan(uint32_t timeout)
{
  UNUSED(timeout);
  if (!m_isConnected)
    return RESULT_FAIL;

  sendCmd(LIDAR_CMD_FORCE_STOP);
  delay(5);
  sendCmd(LIDAR_CMD_STOP);
  delay(5);
  return RESULT_OK;
}

result_t YDlidarDriver::startThread()
{
  m_thread = new std::thread(&YDlidarDriver::cacheScanData, this);
  if (!m_thread)
  {
    error("[YDLIDAR] Fail to create thread");
    return RESULT_FAIL;
  }

  info("[YDLIDAR] Create thread 0x%X", m_thread->get_id());
  return RESULT_OK;
}

result_t YDlidarDriver::startAutoScan(bool force, uint32_t timeout)
{
  result_t ans = RESULT_FAIL;
  if (!m_isConnected)
    return RESULT_FAIL;
  {
    if (!sendCmd(force ? LIDAR_CMD_FORCE_SCAN : LIDAR_CMD_SCAN))
      return ans;

    if (!m_SingleChannel)
    {
      std::vector<uint8_t> d;
      if (!waitResp(LIDAR_ANS_TYPE_MEASUREMENT, d, timeout))
        return ans;
      if (d.size() < 5)
        return ans;
    }
  }

  if (isSupportMotorCtrl(model))
  {
    startMotor();
  }

  return RESULT_OK;
}

result_t YDlidarDriver::stop()
{
  stopThread();
  stopScan();
  if (isSupportMotorCtrl(model))
    stopMotor();

  return RESULT_OK;
}

result_t YDlidarDriver::reset(uint32_t timeout)
{
  UNUSED(timeout);
  result_t ans = RESULT_FAIL;
  if (!m_isConnected)
    return RESULT_FAIL;

  if (!sendCmd(LIDAR_CMD_RESET))
    return ans;

  return RESULT_OK;
}

result_t YDlidarDriver::startMotor()
{
  ScopedLocker l(_cmd_lock);
  if (m_SupportMotorDtrCtrl)
  {
    setDTR();
    delay(500);
  }
  else
  {
    clearDTR();
    delay(500);
  }

  return RESULT_OK;
}

result_t YDlidarDriver::stopMotor()
{
  ScopedLocker l(_cmd_lock);

  if (m_SupportMotorDtrCtrl)
  {
    clearDTR();
    delay(500);
  }
  else
  {
    setDTR();
    delay(500);
  }

  return RESULT_OK;
}

result_t YDlidarDriver::getScanFrequency(
    scan_frequency &frequency,
    uint32_t timeout)
{
  result_t ans = RESULT_FAIL;
  if (!m_isConnected)
    return RESULT_FAIL;

  stopThread();
  {
    if (!sendCmd(LIDAR_CMD_GET_AIMSPEED))
      return ans;
    std::vector<uint8_t> d;
    if (!waitResp(LIDAR_ANS_TYPE_DEVINFO, d, timeout))
      return ans;
    if (d.size() < SCANFREQSIZE)
      return ans;
    memcpy(reinterpret_cast<uint8_t*>(&frequency), d.data(), SCANFREQSIZE);
  }
  return RESULT_OK;
}

result_t YDlidarDriver::setScanFrequencyAdd(
  scan_frequency &frequency,
  uint32_t timeout)
{
  result_t ans = RESULT_FAIL;
  if (!m_isConnected)
    return RESULT_FAIL;

  stopThread();
  {
    if (!sendCmd(LIDAR_CMD_SET_AIMSPEED_ADD))
      return ans;
    std::vector<uint8_t> d;
    if (!waitResp(LIDAR_ANS_TYPE_DEVINFO, d, timeout))
      return ans;
    if (d.size() < SCANFREQSIZE)
      return ans;
    memcpy(reinterpret_cast<uint8_t*>(&frequency), d.data(), SCANFREQSIZE);
  }
  return RESULT_OK;
}

result_t YDlidarDriver::setScanFrequencyDis(
  scan_frequency &frequency,
  uint32_t timeout)
{
  result_t ans = RESULT_FAIL;
  if (!m_isConnected)
    return RESULT_FAIL;

  stopThread();
  {
    if (!sendCmd(LIDAR_CMD_SET_AIMSPEED_DIS))
      return ans;
    std::vector<uint8_t> d;
    if (!waitResp(LIDAR_ANS_TYPE_DEVINFO, d, timeout))
      return ans;
    if (d.size() < SCANFREQSIZE)
      return ans;
    memcpy(reinterpret_cast<uint8_t*>(&frequency), d.data(), SCANFREQSIZE);
  }
  return RESULT_OK;
}

result_t YDlidarDriver::setScanFrequencyAddMic(
  scan_frequency &frequency,
  uint32_t timeout)
{
  result_t ans = RESULT_FAIL;
  if (!m_isConnected)
    return ans;

  stopThread();
  {
    if (!sendCmd(LIDAR_CMD_SET_AIMSPEED_ADDMIC))
      return ans;
    std::vector<uint8_t> d;
    if (!waitResp(LIDAR_ANS_TYPE_DEVINFO, d, timeout))
      return ans;
    if (d.size() < SCANFREQSIZE)
      return ans;
    memcpy(reinterpret_cast<uint8_t*>(&frequency), d.data(), SCANFREQSIZE);
  }
  return RESULT_OK;
}

result_t YDlidarDriver::setScanFrequencyDisMic(scan_frequency &frequency,
                                                uint32_t timeout)
{
  result_t ans = RESULT_FAIL;
  if (!m_isConnected)
    return ans;

  stopThread();
  {
    if (!sendCmd(LIDAR_CMD_SET_AIMSPEED_DISMIC))
      return ans;
    std::vector<uint8_t> d;
    if (!waitResp(LIDAR_ANS_TYPE_DEVINFO, d, timeout))
      return ans;
    if (d.size() < SCANFREQSIZE)
      return ans;
    memcpy(reinterpret_cast<uint8_t*>(&frequency), d.data(), SCANFREQSIZE);
  }
  return RESULT_OK;
}

result_t YDlidarDriver::getSamplingRate(
  sampling_rate &rate, 
  uint32_t timeout)
{
  result_t ans = RESULT_FAIL;
  if (!m_isConnected)
    return ans;

  stopThread();
  {
    if (!sendCmd(LIDAR_CMD_GET_SAMPLING_RATE))
      return ans;
    std::vector<uint8_t> d;
    if (!waitResp(LIDAR_ANS_TYPE_DEVINFO, d, timeout))
      return ans;
    if (d.size() < SAMPLERATESIZE)
      return ans;
    memcpy(reinterpret_cast<uint8_t*>(&rate), d.data(), SAMPLERATESIZE);
    m_sampling_rate = rate.rate;
  }
  return RESULT_OK;
}

result_t YDlidarDriver::setSamplingRate(
  sampling_rate &rate, 
  uint32_t timeout)
{
  result_t ans = RESULT_FAIL;
  if (!m_isConnected)
    return ans;

  stopThread();
  {
    if (!sendCmd(LIDAR_CMD_SET_SAMPLING_RATE))
      return ans;
    std::vector<uint8_t> d;
    if (!waitResp(LIDAR_ANS_TYPE_DEVINFO, d, timeout))
      return ans;
    if (d.size() < SAMPLERATESIZE)
      return ans;
    memcpy(reinterpret_cast<uint8_t*>(&rate), d.data(), SAMPLERATESIZE);
  }
  return RESULT_OK;
}

result_t YDlidarDriver::getZeroOffsetAngle(
  offset_angle &angle,
  uint32_t timeout)
{
  result_t ans = RESULT_FAIL;
  if (!m_isConnected)
    return ans;

  stopThread();
  {
    if (!sendCmd(LIDAR_CMD_GET_OFFSET_ANGLE))
      return ans;
    std::vector<uint8_t> d;
    if (!waitResp(LIDAR_ANS_TYPE_DEVINFO, d, timeout))
      return ans;
    if (d.size() < OFFSETANGLESIZE)
      return ans;
    memcpy(reinterpret_cast<uint8_t*>(&angle), d.data(), OFFSETANGLESIZE);
  }
  return RESULT_OK;
}

result_t YDlidarDriver::setScanHeartbeat(
  scan_heart_beat &beat,
  uint32_t timeout)
{
  result_t ans = RESULT_FAIL;
  if (!m_isConnected)
    return ans;

  stopThread();
  {
    if (!sendCmd(LIDAR_CMD_SET_HEART_BEAT))
      return ans;
    std::vector<uint8_t> d;
    if (!waitResp(LIDAR_ANS_TYPE_DEVINFO, d, timeout))
      return ans;
    if (d.size() < HEARTBEATSIZE)
      return ans;
    memcpy(reinterpret_cast<uint8_t*>(&beat), d.data(), HEARTBEATSIZE);
  }
  return RESULT_OK;
}

result_t YDlidarDriver::getAutoZeroOffsetAngle(
  offset_angle &angle,
  uint32_t timeout)
{
  result_t ans = RESULT_FAIL;
  if (!m_isConnected)
    return ans;

  stopThread();
  {
    if (!sendCmd(LIDAR_CMD_GET_OFFSET_ANGLE))
      return ans;
    std::vector<uint8_t> d;
    if (!waitResp(LIDAR_ANS_TYPE_DEVINFO, d, timeout))
      return ans;
    if (d.size() < OFFSETANGLESIZE)
      return ans;
    memcpy(reinterpret_cast<uint8_t*>(&angle), d.data(), OFFSETANGLESIZE);
  }
  return RESULT_OK;
}

bool YDlidarDriver::getPitchAngle(float& pitch)
{
  if (!m_isConnected || m_SingleChannel)
      return false;

  result_t ret = RESULT_OK;
  if (!sendCmd(LIDAR_CMD_GETPITCH))
    return false;
  std::vector<uint8_t> d;
  if (!waitResp(LIDAR_ANS_TYPE_PITCH, d, TIMEOUT_300))
    return false;
  if (d.size() < sizeof(int32_t))
    return false;
  int32_t p = 0;
  memcpy(reinterpret_cast<uint8_t*>(&p), d.data(), sizeof(int32_t));
  pitch = p / SDK_UNIT100; //缩小100倍
  return true;
}

bool YDlidarDriver::isOpen() const
{
  return _serial && _serial->isOpen();
}

std::vector<uint8_t> YDlidarDriver::readAll(uint32_t timeout)
{
  std::vector<uint8_t> data;
  ScopedLocker l(_cmd_lock);
  if (!isOpen())
    return data;
  if (YDLIDAR_TYPE_UDP != m_DeviceType)
  {
    //等待数据
    size_t len = 0;
    result_t ret = _serial->waitfordata(1, timeout, &len);
    if (!IS_OK(ret))
      return data;
    //读取数据
    data.resize(len);
  }
  else
  {
    data.resize(TOFPACKSIZE);
  }
  size_t rs = _serial->readData(data.data(), data.size());
  if (!rs)
  {
    // warn("Failed to recv data");
    data.clear(); //重置
  }
  else
  {
    data.resize(rs);
    if (m_Debug)
      debugh(data.data(), data.size(), "[recv] ");
  }
  return data;
}

bool YDlidarDriver::waitResp(
  uint8_t type,
  std::vector<uint8_t>& data,
  uint32_t timeout)
{
  uint32_t st = getms();
  uint32_t dt = 0;
  int pos = 0;
  uint32_t size = 0;
  std::vector<uint8_t> d;

  while ((dt = getms() - st) < timeout)
  {
      //读取协议头部分
      std::vector<uint8_t> dh = readAll(timeout);
      if (dh.empty())
          return false;
      for (int i=0; i<dh.size(); ++i)
      {
          uint8_t c = dh.at(i);
          switch (pos)
          {
          case 0:
              if (c != PHA5)
              {
                  continue;
              }
              d.clear();
              break;
          case 1:
              if (c != PH5A)
              {
                  if (c != PHA5)
                      pos = 0;
                  continue;
              }
              break;
          case 2:
              size = c;
              break;
          case 3:
              size += uint32_t(c << 8);
              break;
          case 4:
              size += uint32_t(c << 16);
              break;
          case 5:
              size += uint32_t(c << 24);
              size = size << 2 >> 2;
              break;
          case 6:
              if (c != type)
              {
                  pos = 0;
                  continue;
              }
              break;
          }
          pos ++;
          if (pos > TRIRESPHEADSIZE)
              d.push_back(c);
          //如果解析到协议头
          if (pos >= TRIRESPHEADSIZE + size)
          {
              data = d;
//            debugh(data.toHex().toUpper());
              return true;
          }
      }
  }

  return false;
}

int YDlidarDriver::getPointBytes()
{
  if (NODE_QUAL0 == m_intensityBit)
    return 2;
  else if (NODE_QUAL8 == m_intensityBit || 
           NODE_QUAL10 == m_intensityBit)
    return 3;
  else
    return 4;
}

uint64_t YDlidarDriver::calcDelay()
{
  size_t size = 0;
  {
    ScopedLocker l(_cmd_lock);
    if (isOpen())
      size = _serial->available();
  }
  uint64_t delayTime = 0;
  if (size > TRI_PACKHEADSIZE)
  {
    size_t packageNum = 0;
    size_t Number = 0;
    size_t PackageSize = TRI_PACKMAXNODES;
    packageNum = size / PackageSize;
    Number = size % PackageSize;
    delayTime = packageNum * (PackageSize - TRI_PACKHEADSIZE) * m_PointTime / 2;
    if (Number > TRI_PACKHEADSIZE)
    {
      delayTime += m_PointTime * ((Number - TRI_PACKHEADSIZE) / 2);
    }
  }
  return size * trans_delay + delayTime;
}

bool YDlidarDriver::calcCheckSum(const std::vector<uint8_t>& data)
{
  //判断校验和是否有误
  const uint16_t* p = reinterpret_cast<const uint16_t*>(data.data());
  uint16_t ccs = 0; //计算的校验和
  uint16_t ocs = 0; //原来的校验和
  //计算头部信息（头部标识也参与计算）
  for (int i=0; i<(TRI_PACKHEADSIZE-2)/2; ++i)
  {
      ccs ^= *p ++;
  }
  ocs = *p ++; //原有校验和

  uint8_t count = uint8_t(data.at(3));
  for (int i = 0; i < count; ++i)
  {
      if (NODE_QUAL0 == m_intensityBit ||
          NODE_QUAL16 == m_intensityBit) //如果强度信息2字节
      {
          ccs ^= *p ++;
      }
      else //如果强度信息1字节，则先计算第一个字节，再算后边两个字节
      {
          const uint8_t *p2 = reinterpret_cast<const uint8_t*>(p);
          ccs ^= *p2 ++;
          p = reinterpret_cast<const uint16_t*>(p2);
      }

      ccs ^= *p ++;
  }

  if (ccs != ocs)
  {
    error("Checksum error 0x%04X != 0x%04X", ccs, ocs);
    //如果启用自动判断信号强度
    if (m_AutoIntensity)
    {
      //如果连续2次校验和错误，则自动改变信号强度位数
      if (++m_csCount > 1)
      {
          if (NODE_QUAL16 == m_intensityBit)
            m_intensityBit = NODE_QUAL8;
          else if (NODE_QUAL8 == m_intensityBit)
            m_intensityBit = NODE_QUAL0;
          else
            m_intensityBit = NODE_QUAL16;
          m_csCount = 0;
          info("The intensity has been automatically adjusted to[%d]bit", 
            m_intensityBit);
      }
    }
    return false;
  }
  m_csCount = 0;
  return true;
}

bool YDlidarDriver::parsePoints()
{
    std::vector<node_info> nodes;
    int validCount = 0; //有效点数
    uint32_t delay = calcDelay();
    for (const auto& data : m_datas)
    {
        YdDataStream ds(data.data);
        uint16_t h = 0; //头
        uint8_t freq = 0; //转速
        uint8_t count = 0; //转速
        uint16_t a = 0; //角度
        float sa = .0;
        float ea = .0;
        float step = 0.0; //角度增量

        ds >> h;
        ds >> freq;
        freq = freq >> 1;
        ds >> count;
        ds >> a;
        sa = (a >> 1) / SDK_UNIT64;
        ds >> a;
        ea = (a >> 1) / SDK_UNIT64;
        step = fabs(ea - sa) / (count - 1);
        ds >> h; //跳过校验和
        std::vector<node_info> ns(count);
        for (uint8_t i=0; i<count; ++i)
        {
          float a = (sa + step * i); //角度值
          uint16_t p = 0; //强度
          uint16_t d = 0; //距离
          if (NODE_QUAL16 == m_intensityBit)
          {
              ds >> p;
              ds >> d;
          }
          else if (NODE_QUAL8 == m_intensityBit ||
                   NODE_QUAL10 == m_intensityBit)
          {
            uint8_t p2;
            ds >> p2;
            ds >> d;
            //距离值低2位为强度值的高2位
            p = uint16_t(p2) | uint16_t((d & 0x03) << 8);
          }
          else /*if (NODE_QUAL0 == m_intensityBit)*/
          {
            ds >> d;
          }
          float ca = 0;
          if (d) //如果距离值有效
          {
            if (isOctaveLidar(model))
            {
              ca = ((atan(((21.8 * (155.3 - (d / 2.0))) / 155.3) / (d / 2.0))) * SDK_ANGLE180 / M_PI);
            }
            else if (isSCLLidar(m_LidarType) ||
                    isSCLLidar2(model))
            {
              // SCL雷达角度二级解析公式
              ca = atan(17.8 / (d / 4.0)) * SDK_ANGLE180 / M_PI;
            }
            else if (isTriangleLidar(m_LidarType) &&
                    !isTminiLidar(model) && // 去掉Tmini雷达的角度二级解析
                    !isR3Lidar(model)) // 去掉R3雷达的角度二级解析
            {
              ca = ((atan(((21.8 * (155.3 - (d / 4.0))) / 155.3) / (d / 4.0))) * SDK_ANGLE180 / M_PI);
            }
            validCount ++;
          }
          auto& node = ns[i];
          node.scanFreq = freq;
          node.angle = (a + ca) * SDK_UNIT128;
          node.dist = d;
          node.qual = p;
          node.stamp = data.stamp;
          node.delayTime = delay;
//            SeLog::debug("i:%d,a:%f,d:%u,p:%u",
//                i, sa + step * i, d, p);

        }
        nodes.insert(nodes.end(), ns.begin(), ns.end());
    }
    //判断有效点数
    if (validCount < 2)
    {
      if (m_driverErrno == NoError)
        setDriverError(LaserFailureError);
    }
    else
    {
      if (m_driverErrno == LaserFailureError)
        setDriverError(NoError);
    }
    //将点云存入缓存
    {
        ScopedLocker l(_lock);
        memcpy(scan_node_buf, nodes.data(), nodes.size() * SDKNODESIZE);
        scan_node_count = nodes.size();
        _dataEvent.set();
    }

    return true;
}

}
