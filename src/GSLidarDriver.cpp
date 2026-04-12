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
#include <math.h>
#include <fstream>
#include <algorithm>
#include "GSLidarDriver.h"
#include "core/serial/common.h"
#include "core/serial/serial.h"
#include "core/network/ActiveSocket.h"
#include "core/common/ydlidar_help.h"
#include "ydlidar_config.h"

#define GS_CMD_STARTIAP 0x0A //启动IAP
#define GS_CMD_EXECIAP 0x0B //运行IAP，传输数据包
#define GS_CMD_STOPIAP 0x0C //停止IAP
#define GS_CMD_ACKIAP 0x20 //IAP应答
#define GS_CMD_RESET 0x67 //复位
#define GS_CMD_ACKOK 0x01 //正常
#define GS_CMD_ZERO 0x00 //0

#define DATA_LEN_PER_FRAME (81 - 18 + 1) //每帧数据长度

using namespace impl;

namespace ydlidar {
using namespace core::common;
using namespace core::serial;
using namespace core::network;

GSLidarDriver::GSLidarDriver(uint8_t type)
{
    //串口配置参数
    m_intensities       = false;
    isAutoReconnect     = true;
    m_baudrate          = 230400;
    scan_node_count     = 0;
    sample_rate         = 5000;
    m_PointTime         = 1e9 / 5000;
    trans_delay         = 0;
    retryCount          = 0;
    m_SingleChannel     = false;
    m_LidarType         = TYPE_GS;
    m_DeviceType = type;
    //解析参数
    PackageSampleBytes  = 2;
    CheckSum            = 0;
    CheckSumCal         = 0;
    CheckSumResult      = false;
    moduleNum           = 0;

    nodeIndex = 0;
    globalRecvBuffer = new uint8_t[GSPACKSIZE];
    scan_node_buf = new node_info[MAX_SCAN_NODES];
    for (int i=0; i<LIDAR_MAXCOUNT; ++i)
    {
        k0[i] = 0;
        k1[i] = 0;
        b0[i] = 0;
        b1[i] = 0;
        bias[i] = 0;
    }
}

GSLidarDriver::~GSLidarDriver() 
{
    disableDataGrabbing();

    ScopedLocker l(_cmd_lock);
    if (_comm) {
        if (_comm->isOpen()) {
            _comm->flush();
            _comm->closePort();
        }
        delete _comm;
        _comm = NULL;
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

result_t GSLidarDriver::connect(const char *port_path, uint32_t baudrate) 
{
    m_baudrate = baudrate;
    m_port = string(port_path);
    {
        ScopedLocker l(_cmd_lock);
        if (!_comm)
        {
            if (m_DeviceType == YDLIDAR_TYPE_TCP)
            {
                _comm = new CActiveSocket();
            }
            else
            {
                _comm = new serial::Serial(m_port, m_baudrate,
                    serial::Timeout::simpleTimeout(DEFAULT_TIMEOUT));
            }
            _comm->bindport(port_path, baudrate);
        }
        if (!_comm->open())
        {
            setDriverError(NotOpenError);
            return RESULT_FAIL;
        }
        m_isConnected = true;
    }

    //DTR
    if (m_SupportMotorDtrCtrl)
    	setDTR();
    else
        clearDTR();

    stopScan();
    //配置GS2模组地址（三个模组）
    setDeviceAddress(300);

    return RESULT_OK;
}

void GSLidarDriver::setDTR() {
    if (!m_isConnected) {
        return ;
    }

    if (_comm) {
        _comm->setDTR(1);
    }

}

void GSLidarDriver::clearDTR() {
    if (!m_isConnected) {
        return ;
    }

    if (_comm) {
        _comm->setDTR(0);
    }
}

void GSLidarDriver::flushSerial() {
    if (!m_isConnected) {
        return;
    }

    size_t len = _comm->available();
    if (len) {
        _comm->readSize(len);
    }

    delay(20);
}

void GSLidarDriver::disconnect() {
    isAutoReconnect = false;

    if (!m_isConnected) {
        return ;
    }

    stop();
    delay(10);
    ScopedLocker l(_cmd_lock);

    if (_comm) {
        if (_comm->isOpen()) {
            _comm->closePort();
        }
    }

    m_isConnected = false;
}

void GSLidarDriver::disableDataGrabbing()
{
    if (m_isScanning) {
        m_isScanning = false;
        _dataEvent.set();
    }
    if (m_thread)
    {
        if (m_thread->joinable())
            m_thread->join();
        delete m_thread;
        m_thread = nullptr;
    }
    // _thread.join();
}

bool GSLidarDriver::isscanning() const {
    return m_isScanning;
}
bool GSLidarDriver::isconnected() const {
    return m_isConnected;
}

result_t GSLidarDriver::sendCommand(uint8_t cmd,
                                    const void *payload,
                                    size_t payloadsize)
{
    return sendCommand(0x00, cmd, payload, payloadsize);
//    uint8_t pkt_header[12];
//    gs_package_head *header = reinterpret_cast<gs_package_head * >(pkt_header);
//    uint8_t checksum = 0;

//    if (!m_isConnected) {
//        return RESULT_FAIL;
//    }

//    header->syncByte0 = LIDAR_CMD_SYNC_BYTE;
//    header->syncByte1 = LIDAR_CMD_SYNC_BYTE;
//    header->syncByte2 = LIDAR_CMD_SYNC_BYTE;
//    header->syncByte3 = LIDAR_CMD_SYNC_BYTE;
//    header->address = 0x00;
//    header->cmd_flag = cmd;
//    header->size = 0xffff&payloadsize;
//    sendData(pkt_header, 8) ;
//    checksum += cmd;
//    checksum += 0xff&header->size;
//    checksum += 0xff&(header->size>>8);
    
//    if (payloadsize && payload) {
//      for (size_t pos = 0; pos < payloadsize; ++pos) {
//        checksum += ((uint8_t *)payload)[pos];
//      }
//      uint8_t sizebyte = (uint8_t)(payloadsize);
//      sendData((const uint8_t *)payload, sizebyte);
//    }
    
//    sendData(&checksum, 1);

//    return RESULT_OK;
}

result_t GSLidarDriver::sendCommand(uint8_t addr,
                                    uint8_t cmd,
                                    const void *payload,
                                    size_t payloadsize)
{
    uint8_t pkt_header[12];
    gs_package_head *header = reinterpret_cast<gs_package_head * >(pkt_header);
    uint8_t checksum = 0;

    if (!m_isConnected) {
        return RESULT_FAIL;
    }

    header->syncByte0 = LIDAR_CMD_SYNC_BYTE;
    header->syncByte1 = LIDAR_CMD_SYNC_BYTE;
    header->syncByte2 = LIDAR_CMD_SYNC_BYTE;
    header->syncByte3 = LIDAR_CMD_SYNC_BYTE;
    header->address = addr;
    header->type = cmd;
    header->size = 0xffff&payloadsize;
    sendData(pkt_header, 8) ;
    checksum += addr;
    checksum += cmd;
    checksum += 0xff&header->size;
    checksum += 0xff&(header->size>>8);

    if (payloadsize && payload) {
      for (size_t pos = 0; pos < payloadsize; ++pos) {
        checksum += ((uint8_t *)payload)[pos];
      }
      uint8_t sizebyte = (uint8_t)(payloadsize);
      sendData((const uint8_t *)payload, sizebyte);
    }

    sendData(&checksum, 1);

    return RESULT_OK;
}

result_t GSLidarDriver::sendData(const uint8_t *data, size_t size) {
    if (!_comm || !_comm->isOpen()) {
        return RESULT_FAIL;
    }

    if (data == NULL || size == 0) {
        return RESULT_FAIL;
    }

    size_t r;

    while (size) 
    {
        r = _comm->writeData(data, size);
        if (!r) 
        {
            return RESULT_FAIL;
        }

        if (m_Debug)
        {
            debugh(data, r, "[send] ");
        }

        size -= r;
        data += r;
    }

    return RESULT_OK;
}

result_t GSLidarDriver::getData(uint8_t *data, size_t size) {
    if (!_comm || !_comm->isOpen()) {
        return RESULT_FAIL;
    }

    size_t r;
    while (size)
    {
        r = _comm->readData(data, size);
        if (!r)
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

result_t GSLidarDriver::waitResponseHeader(gs_package_head *header,
                                           uint32_t timeout) {
    int  recvPos     = 0;
    uint32_t startTs = getms();
    uint8_t  recvBuffer[sizeof(gs_package_head)];
    uint8_t  *headerBuffer = reinterpret_cast<uint8_t *>(header);
    uint32_t waitTime = 0;
    
    while ((waitTime = getms() - startTs) <= timeout) {
      size_t remainSize = sizeof(gs_package_head) - recvPos;
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
                  recvPos = 0;
                  continue;
              }
              break;
    
            case 1:
              if (currentByte != LIDAR_ANS_SYNC_BYTE1) {
                  recvPos = 0;
                  continue;
              }
              break;
    
            case 2:
              if (currentByte != LIDAR_ANS_SYNC_BYTE1) {
                  recvPos = 0;
                  continue;
              }
              break;
    
            case 3:
              if (currentByte != LIDAR_ANS_SYNC_BYTE1) {
                  recvPos = 0;
                  continue;
              }
              break;
    
            default:
              break;
        }
    
        headerBuffer[recvPos++] = currentByte;
    
        if (recvPos == sizeof(gs_package_head)) {
          return RESULT_OK;
        }
      }
    }

    return RESULT_FAIL;
}

result_t GSLidarDriver::waitResponseHeaderEx(
    gs_package_head *header, 
    uint8_t cmd, 
    uint32_t timeout)
{
    int recvPos = 0;
    uint32_t startTs = getms();
    uint8_t  recvBuffer[GSPACKEGEHEADSIZE];
    uint8_t  *headerBuffer = reinterpret_cast<uint8_t*>(header);
    uint32_t waitTime = 0;

    while ((waitTime = getms() - startTs) <= timeout)
    {
        size_t remainSize = GSPACKEGEHEADSIZE - recvPos;
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

        for (size_t pos = 0; pos < recvSize; ++pos) 
        {
            uint8_t currentByte = recvBuffer[pos];
            switch (recvPos) {
            case 0:
                if (currentByte != LIDAR_ANS_SYNC_BYTE1) {
                    recvPos = 0;
                    continue;
                }
                break;
            case 1:
                if (currentByte != LIDAR_ANS_SYNC_BYTE1) {
                    recvPos = 0;
                    continue;
                }
                break;
            case 2:
                if (currentByte != LIDAR_ANS_SYNC_BYTE1) {
                    recvPos = 0;
                    continue;
                }
                break;
            case 3:
                if (currentByte != LIDAR_ANS_SYNC_BYTE1) {
                    recvPos = 0;
                    continue;
                }
                break;
            case 4:
                if (currentByte == LIDAR_ANS_SYNC_BYTE1)
                    continue;
                break;
            case 5:
                if (currentByte != cmd) {
                    recvPos = 0;
                    continue;
                }
                break;
            default:
                break;
            }

            headerBuffer[recvPos++] = currentByte;

            if (recvPos == GSPACKEGEHEADSIZE) {
                return RESULT_OK;
            }
        }
    }

    return RESULT_FAIL;
}

result_t GSLidarDriver::waitForData(size_t data_count, uint32_t timeout,
                                    size_t *returned_size) {
    size_t length = 0;

    if (returned_size == NULL) {
        returned_size = (size_t *)&length;
    }

    return (result_t)_comm->waitfordata(data_count, timeout, returned_size);
}

result_t GSLidarDriver::checkAutoConnecting() 
{
    result_t ans = RESULT_FAIL;

    if (m_driverErrno != BlockError)
        setDriverError(TimeoutError);

    while (isAutoReconnect && isscanning())
    {
        {
            ScopedLocker l(_cmd_lock);
            if (_comm) {
                if (_comm->isOpen()) {
                    m_isConnected = false;
                    _comm->closePort();
                }
            }
        }
        delay(100); //延时

        while (isscanning() &&
               connect(m_port.c_str(), m_baudrate) != RESULT_OK)
        {
            setDriverError(NotOpenError);
            delay(300);
        }

        if (!isscanning()) {
            return RESULT_FAIL;
        }
        //判断是否已重连，如是则尝试启动雷达
        if (isconnected()) 
        {
            delay(100);
            ans = startAutoScan();
            if (IS_OK(ans)) {
                return ans;
            }
            else {
                setDriverError(DeviceNotFoundError);
            }
        }
    }

    return RESULT_FAIL;
}

int GSLidarDriver::cacheScanData()
{
    node_info      local_buf[GS_PACKMAXNODES];
    size_t         count = GS_PACKMAXNODES;
    size_t         scan_count = 0;
    result_t       ans = RESULT_FAIL;

    int timeout_count = 0;
    retryCount = 0;
    lastStamp = 0;

    m_isScanning = true;

  //打印帧间隔相关
//   std::map<int, uint32_t> ts;
//   for (int i=0; i<LIDAR_MAXCOUNT; ++i)
//     ts[i] = getms();

    while (m_isScanning)
    {
        count = GS_PACKMAXNODES;
        ans = waitScanData(local_buf, count);
        // Thread::needExit();
        if (!IS_OK(ans))
        {
            if (IS_FAIL(ans))
            {
                timeout_count ++;
            }
            else
            {
                timeout_count += 2;
                if (m_driverErrno != BlockError)
                    setDriverError(TimeoutError);
            }
            fprintf(stderr, "[GSLIDAR] Timeout count: %d\n", timeout_count);
            fflush(stderr);
            // 重连雷达
            if (!isAutoReconnect)
            {
                fprintf(stderr, "[GSLIDAR] Exit scanning thread\n");
                fflush(stderr);
                m_isScanning = false;
                return RESULT_FAIL;
            }
            else if (timeout_count > DEFAULT_TIMEOUT_COUNT)
            {
                ans = checkAutoConnecting();
                if (IS_OK(ans))
                {
                    timeout_count = 0;
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
            timeout_count = 0;
            retryCount = 0;

            {
            //数据存入数组
            ScopedLocker l(_lock);
            gs_module_nodes nodes;
            nodes.moduleNum = moduleNum;
            nodes.pointCount = count;
            memcpy(nodes.points, local_buf, count * SDKNODESIZE);
            datas.push_back(nodes);
            if (datas.size() > moduleCount)
              datas.pop_front();
            scan_count = 0;
            }
            // uint32_t t = getms();
            // uint32_t dt = t - ts[moduleNum];
            // if (dt > 150)
      	    //   core::common::warn("module[%d] time[%lld]ms", moduleNum, dt);
            // ts[moduleNum] = t;
        }
    }

    m_isScanning = false;

    return RESULT_OK;
}

result_t GSLidarDriver::waitPackage(node_info *node, uint32_t timeout)
{
    int pos = 0;
    uint32_t startTs = getms();
    uint32_t waitTime = 0;
    uint8_t *packageBuffer = (uint8_t *)&package;
    int package_recvPos = 0;
    uint16_t sample_lens = 0;
    uint16_t package_Sample_Num = 0;
    result_t ret = RESULT_FAIL;
    size_t recvSize = 0;
    size_t remainSize = 0;
    CheckSumCal = 0;

    if (nodeIndex == 0)
    {
        pos = 0;
        while ((waitTime = getms() - startTs) < timeout)
        {
            //解析协议头部分
            remainSize = GS_PACKHEADSIZE - pos;
            recvSize = 0;
            ret = waitForData(remainSize, timeout - waitTime, &recvSize);
            if (!IS_OK(ret))
                return ret;
            if (recvSize > remainSize)
                recvSize = remainSize;
            getData(globalRecvBuffer, recvSize);

PARSEHEAD:
            for (size_t i = 0; i < recvSize; ++i)
            {
                uint8_t c = globalRecvBuffer[i];
                switch (pos)
                {
                case 0:
                    if (c != LIDAR_ANS_SYNC_BYTE1)
                    {
                        pos = 0;
                        continue;
                    }
                    break;
                case 1:
                    if (c != LIDAR_ANS_SYNC_BYTE1)
                    {
                        pos = 0;
                        continue;
                    }
                    break;
                case 2:
                    if (c != LIDAR_ANS_SYNC_BYTE1)
                    {
                        pos = 0;
                        continue;
                    }
                    break;
                case 3:
                    if (c != LIDAR_ANS_SYNC_BYTE1)
                    {
                        pos = 0;
                        continue;
                    }
                    break;
                case 4:
                    if (c == LIDAR_ANS_SYNC_BYTE1) //过滤出现超过4个包头标识的情况
                        continue;
                    moduleNum = uint8_t(c >> 1); //模组地址转编号: 1, 2, 4
                    CheckSumCal = c;
                    break;
                case 5:
                    if (c != GS_LIDAR_ANS_SCAN)
                    {
                        pos = 0;
                        CheckSumCal = 0;
                        moduleNum = 0;
                        continue;
                    }
                    CheckSumCal += c;
                    break;
                case 6:
                    sample_lens |= 0x00ff & c;
                    CheckSumCal += c;
                    break;
                case 7:
                    sample_lens |= (0x00ff & c) << 8;
                    CheckSumCal += c;
                    break;
                default:
                    break;
                }

                packageBuffer[pos++] = c;

                // 如果解析到协议头
                if (pos == GS_PACKHEADSIZE)
                {
                    // 如果协议数据长度不对则跳过，继续解析协议头
                    if (!sample_lens || sample_lens >= GSPACKSIZE)
                    {
                        moduleNum = 0;
                        pos = 0;
                        continue;
                    }
                    package_Sample_Num = sample_lens + 1; // 环境2Bytes + 点云320Bytes + CRC
                    package_recvPos = pos;
                    nodeCount = (sample_lens - 2) / GSNODESIZE; //计算1包数据中的点数
                    // info("Sample num %d", (package_Sample_Num - 3) / 2);
                    pos = 0;
                    // 解析协议数据部分
                    while ((waitTime = getms() - startTs) <= timeout)
                    {
                        int offset = 0; // 缓存偏移量
                        // 如果解析协议头时接收数据长度超过定义的长度则认为是从校验和错误处跳转过来的
                        if (recvSize > GS_PACKHEADSIZE)
                        {
                            offset = i + 1;
                        }
                        else
                        {
                            remainSize = package_Sample_Num - pos;
                            recvSize = 0;
                            ret = waitForData(remainSize, timeout - waitTime, &recvSize);
                            if (!IS_OK(ret))
                                return ret;
                            if (recvSize > remainSize)
                                recvSize = remainSize;
                            getData(globalRecvBuffer, recvSize);
                        }

                        for (size_t j = offset; j < recvSize; ++j)
                        {
                            if (pos + 1 == package_Sample_Num)
                            {
                                CheckSum = globalRecvBuffer[recvSize - 1];       // crc
                                packageBuffer[package_recvPos + pos] = CheckSum; // crc
                                pos ++;
                                break;
                            }

                            CheckSumCal += globalRecvBuffer[j];
                            packageBuffer[package_recvPos + pos] = globalRecvBuffer[j];
                            pos ++;
                        }
                        
                        if (pos == package_Sample_Num)
                        {
                            pos = 0;
                            // 判断校验和是否一致
                            if (CheckSumCal != CheckSum)
                            {
                                CheckSumResult = false;
                                error("GS cs 0x%02X != 0x%02X", CheckSumCal, CheckSum);
                                // 如果校验和不一致，则需要跳转去当前缓存中查找协议头，
                                // 以免因当前数据包有缺失导致下一包数据解析失败
                                goto PARSEHEAD;
                            }
                            else
                            {
                                CheckSumResult = true;
                                if (lastStamp > 0)
                                {
                                    m_ScanFreq = 1000.0 / (getms() - lastStamp);
                                }
                                lastStamp = getms();
                            }
                            break;
                        }
                        recvSize = 0; //重置缓存数据大小
                    }

                    break;
                } // end if (pos == GS_PACKHEADSIZE)
            } //end for (size_t i = 0; i < recvSize; ++i)
            if (CheckSumResult)
                break;
        } //end while ((waitTime = getms() - startTs) <= timeout)

        if (CheckSumResult)
        {
            model = m_models[moduleNum]; //当前雷达型号
            if (m_Debug)
                debug("GS lidar module[%d] model[%u]", moduleNum, model);
            //根据雷达型号设置角度参数
            if (YDLIDAR_GS5 == model)
                m_pitchAngle = Angle_PAngle2;
            else
                m_pitchAngle = Angle_PAngle;
        }
    } //end if (nodeIndex == 0)

    (*node).stamp = getTime();
    
    if (CheckSumResult)
    {
        //第1个点时间戳使用上一帧的最后1个点的时间戳
        if (nodeIndex == 0)
            (*node).stamp = stamp ? stamp : getTime();
        else
            (*node).stamp = getTime();
        stamp = (*node).stamp;

        (*node).index = moduleNum;
        (*node).scanFreq = m_ScanFreq;
        (*node).qual = 0;
        (*node).sync = NODE_UNSYNC;

        if (YDLIDAR_GS1 == model)
        {
            //GS1低10位为距离，高6位为信号强度
            (*node).dist = uint16_t(package.nodes[nodeIndex].node & 0x03FF);
            //如果配置了信号强度则处理信号强度
            if (m_intensities)
                (*node).qual = uint16_t(package.nodes[nodeIndex].node >> 10);
        }
        else if (YDLIDAR_GS5 == model ||
            YDLIDAR_GS6 == model)
        {
            //GS5、GS6低11位为距离，高5位为信号强度
            (*node).dist = uint16_t(package.nodes[nodeIndex].node & 0x07FF);
            //如果配置了信号强度则处理信号强度
            if (m_intensities)
                (*node).qual = uint16_t(package.nodes[nodeIndex].node >> 11);
        }
        else
        {
            //GS2低9位为距离，高7位为信号强度
            (*node).dist = uint16_t(package.nodes[nodeIndex].node & 0x01FF);
            //如果配置了信号强度则处理信号强度
            if (m_intensities)
                (*node).qual = uint16_t(package.nodes[nodeIndex].node >> 9);
        }

        double sampleAngle = 0;
        if (node->dist > 0)
        {
            if (YDLIDAR_GS1 == model)
                angTransform2((*node).dist, nodeIndex, 
                    &sampleAngle, &(*node).dist);
            else if (YDLIDAR_GS6 == model)
                angTransform2((*node).dist, nodeCount - nodeIndex, 
                    &sampleAngle, &(*node).dist);
            else
                angTransform((*node).dist, nodeIndex, 
                    &sampleAngle, &(*node).dist);
        }

        if (sampleAngle < 0)
        {
            (*node).angle = (((uint16_t)(sampleAngle * 64 + 23040)) << LIDAR_RESP_ANGLE_SHIFT) +
                LIDAR_RESP_CHECKBIT;
        }
        else
        {
            if ((sampleAngle * 64) > 23040)
            {
                (*node).angle = (((uint16_t)(sampleAngle * 64 - 23040)) << LIDAR_RESP_ANGLE_SHIFT) +
                    LIDAR_RESP_CHECKBIT;
            }
            else
            {
                (*node).angle = (((uint16_t)(sampleAngle * 64)) << LIDAR_RESP_ANGLE_SHIFT) +
                    LIDAR_RESP_CHECKBIT;
            }
        }

        if (YDLIDAR_GS2 == model ||
            YDLIDAR_GS5 == model)
        {
            // 过滤左右相机超过0°的点
            if (nodeIndex < 80)
            { // CT_RingStart  CT_Normal
                if ((*node).angle <= 23041)
                {
                    (*node).dist = 0;
                }
            }
            else
            {
                if ((*node).angle > 23041)
                {
                    (*node).dist = 0;
                }
            }
        }

        //处理环境数据（2个字节分别存储在前2个点的is属性中）
        if (0 == nodeIndex)
            (*node).is = package.env & 0xFF;
        else if (1 == nodeIndex)
            (*node).is = package.env >> 8;

        // debug("%u 0x%X %.02f %.02f", nodeIndex, 
        //     package.nodes[nodeIndex].dist,
        //     sampleAngle, node->dist/1.0);
    }
    else
    {
        (*node).qual = 0;
        (*node).angle = LIDAR_RESP_CHECKBIT;
        (*node).dist = 0;
        (*node).scanFreq = 0;
        return RESULT_FAIL;
    }

    nodeIndex ++;
    if (nodeIndex >= nodeCount)
    {
        nodeIndex = 0;
        (*node).sync = NODE_SYNC;
        CheckSumResult = false;
    }

    return RESULT_OK;
}

void GSLidarDriver::angTransform(
    uint16_t dist, 
    int n, 
    double *dstTheta, 
    uint16_t *dstDist)
{
    double pixelU = n, Dist, theta, tempTheta, tempDist, tempX, tempY;
    uint8_t mdNum = moduleNum;
    if (n < nodeCount / 2)
    {
      pixelU = nodeCount / 2 - pixelU;
      if (b0[mdNum] > 1) {
          tempTheta = k0[mdNum] * pixelU - b0[mdNum];
      }
      else
      {
          tempTheta = atan(k0[mdNum] * pixelU - b0[mdNum]) * 180 / M_PI;
      }
      tempDist = (dist - Angle_Px) / cos(((m_pitchAngle + bias[mdNum]) - (tempTheta)) * M_PI / 180);
      tempTheta = tempTheta * M_PI / 180;
      tempX = cos((m_pitchAngle + bias[mdNum]) * M_PI / 180) * tempDist * cos(tempTheta) + 
        sin((m_pitchAngle + bias[mdNum]) * M_PI / 180) * (tempDist * sin(tempTheta));
      tempY = -sin((m_pitchAngle + bias[mdNum]) * M_PI / 180) * tempDist * cos(tempTheta) + 
        cos((m_pitchAngle + bias[mdNum]) * M_PI / 180) * (tempDist * sin(tempTheta));
      tempX = tempX + Angle_Px;
      tempY = tempY - Angle_Py; //5.315
      Dist = sqrt(tempX * tempX + tempY * tempY);
      if (!ISZERO(tempX))
        theta = atan(tempY / tempX) * 180 / M_PI;
    }
    else
    {
      pixelU = nodeCount - pixelU;
      if (b1[mdNum] > 1)
      {
          tempTheta = k1[mdNum] * pixelU - b1[mdNum];
      }
      else
      {
          tempTheta = atan(k1[mdNum] * pixelU - b1[mdNum]) * 180 / M_PI;
      }
      tempDist = (dist - Angle_Px) / cos(((m_pitchAngle + bias[mdNum]) + (tempTheta)) * M_PI / 180);
      tempTheta = tempTheta * M_PI / 180;
      tempX = cos(-(m_pitchAngle + bias[mdNum]) * M_PI / 180) * tempDist * cos(tempTheta) + 
        sin(-(m_pitchAngle + bias[mdNum]) * M_PI / 180) * (tempDist * sin(tempTheta));
      tempY = -sin(-(m_pitchAngle + bias[mdNum]) * M_PI / 180) * tempDist * cos(tempTheta) + 
        cos(-(m_pitchAngle + bias[mdNum]) * M_PI / 180) * (tempDist * sin(tempTheta));
      tempX = tempX + Angle_Px;
      tempY = tempY + Angle_Py; //5.315
      Dist = sqrt(tempX * tempX + tempY * tempY);
      if (!ISZERO(tempX))
        theta = atan(tempY / tempX) * 180 / M_PI;
    }
    if (theta < 0)
    {
      theta += 360;
    }
    *dstTheta = theta;
    *dstDist = Dist;
}

void GSLidarDriver::angTransform2(
    uint16_t dist, 
    int n, 
    double *dstTheta, 
    uint16_t *dstDist)
{
    double pixelU = nodeCount - n, Dist, theta, tempTheta;
    uint8_t mdNum = moduleNum;

    tempTheta = atan(k0[mdNum] * pixelU - b0[mdNum]) * 180 / M_PI;
    Dist = dist / cos(tempTheta * M_PI / 180);
    theta = tempTheta;

    if (theta < 0)
    {
      theta += 360;
    }
    *dstTheta = theta;
    *dstDist = Dist;
}

result_t GSLidarDriver::waitScanData(
    node_info *nodebuffer,
    size_t &count,
    uint32_t timeout)
{
    if (!m_isConnected)
    {
        count = 0;
        return RESULT_FAIL;
    }

    size_t recvNodeCount = 0;
    uint32_t startTs = getms();
    uint32_t waitTime = 0;
    result_t ans = RESULT_FAIL;

    while ((waitTime = getms() - startTs) < timeout && 
        recvNodeCount < count)
    {
        node_info node;
        memset(&node, 0, sizeof(node_info));
        ans = waitPackage(&node, timeout - waitTime);
        if (!IS_OK(ans))
        {
            count = recvNodeCount;
            return ans;
        }

        nodebuffer[recvNodeCount++] = node;

        if (!nodeIndex)
        {
            count = recvNodeCount;
            return RESULT_OK;
        }

        if (recvNodeCount == count)
        {
            return RESULT_OK;
        }
    }

    count = recvNodeCount;
    return RESULT_FAIL;
}

result_t GSLidarDriver::grabScanData(
    node_info *nodes,
    size_t &count,
    uint32_t timeout)
{
    uint32_t st = getms();
    uint32_t wt = 0;
    while ((wt = getms() - st) < timeout)
    {
        {
            ScopedLocker l(_lock);
            if (datas.size())
            {
                //从数组中取出点云数据
                gs_module_nodes ns = datas.front();
                datas.pop_front();
                size_t size = min(int(count), ns.pointCount);
                memcpy(nodes, ns.points, size * SDKNODESIZE);
                count = size;
                return RESULT_OK;
            }
        }
        delay(1); //延时
    }
    return RESULT_TIMEOUT;
}

result_t GSLidarDriver::ascendScanData(node_info *nodebuffer, size_t count) {
    float inc_origin_angle = (float)360.0 / count;
    int i = 0;

    for (i = 0; i < (int)count; i++) {
        if (nodebuffer[i].dist == 0) {
            continue;
        } else {
            while (i != 0) {
                i--;
                float expect_angle = (nodebuffer[i + 1].angle >>
                                                                             LIDAR_RESP_ANGLE_SHIFT) /
                        64.0f - inc_origin_angle;

                if (expect_angle < 0.0f) {
                    expect_angle = 0.0f;
                }

                uint16_t checkbit = nodebuffer[i].angle &
                        LIDAR_RESP_CHECKBIT;
                nodebuffer[i].angle = (((uint16_t)(expect_angle * 64.0f)) <<
                                                   LIDAR_RESP_ANGLE_SHIFT) + checkbit;
            }

            break;
        }
    }

    if (i == (int)count) {
        return RESULT_FAIL;
    }

    for (i = (int)count - 1; i >= 0; i--) {
        if (nodebuffer[i].dist == 0) {
            continue;
        } else {
            while (i != ((int)count - 1)) {
                i++;
                float expect_angle = (nodebuffer[i - 1].angle >>
                                                                             LIDAR_RESP_ANGLE_SHIFT) /
                        64.0f + inc_origin_angle;

                if (expect_angle > 360.0f) {
                    expect_angle -= 360.0f;
                }

                uint16_t checkbit = nodebuffer[i].angle &
                        LIDAR_RESP_CHECKBIT;
                nodebuffer[i].angle = (((uint16_t)(expect_angle * 64.0f)) <<
                                                   LIDAR_RESP_ANGLE_SHIFT) + checkbit;
            }

            break;
        }
    }

    float frontAngle = (nodebuffer[0].angle >>
                                                           LIDAR_RESP_ANGLE_SHIFT) / 64.0f;

    for (i = 1; i < (int)count; i++) {
        if (nodebuffer[i].dist == 0) {
            float expect_angle =  frontAngle + i * inc_origin_angle;

            if (expect_angle > 360.0f) {
                expect_angle -= 360.0f;
            }

            uint16_t checkbit = nodebuffer[i].angle &
                    LIDAR_RESP_CHECKBIT;
            nodebuffer[i].angle = (((uint16_t)(expect_angle * 64.0f)) <<
                                               LIDAR_RESP_ANGLE_SHIFT) + checkbit;
        }
    }

    size_t zero_pos = 0;
    float pre_degree = (nodebuffer[0].angle >>
                                                           LIDAR_RESP_ANGLE_SHIFT) / 64.0f;

    for (i = 1; i < (int)count ; ++i) {
        float degree = (nodebuffer[i].angle >>
                        LIDAR_RESP_ANGLE_SHIFT) / 64.0f;

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
/* get device parameters of gs lidar                                             */
/************************************************************************/
result_t GSLidarDriver::getDevicePara(gs_device_para &info, uint32_t timeout) {
  result_t  ans;
  uint8_t crcSum, mdNum;
  uint8_t *pInfo = reinterpret_cast<uint8_t *>(&info);

  if (!m_isConnected) {
    return RESULT_FAIL;
  }

  disableDataGrabbing();
  flushSerial();
  {
    ScopedLocker l(_cmd_lock);
    if ((ans = sendCommand(GS_LIDAR_CMD_GET_PARAMETER)) != RESULT_OK) {
      return ans;
    }
    gs_package_head h;
    for (int i = 0; i < LIDAR_MAXCOUNT && i < moduleCount; i++)
    {
        if ((ans = waitResponseHeaderEx(&h, GS_LIDAR_CMD_GET_PARAMETER, timeout)) != RESULT_OK) {
          return ans;
        }
        if (h.size < (sizeof(gs_device_para) - 1)) {
          return RESULT_FAIL;
        }
        if (waitForData(h.size+1, timeout) != RESULT_OK) {
          return RESULT_FAIL;
        }
        getData(reinterpret_cast<uint8_t *>(&info), sizeof(info));
        
        crcSum = 0;
        crcSum += h.address;
        crcSum += h.type;
        crcSum += 0xff & h.size;
        crcSum += 0xff & (h.size >> 8);
        for(int j = 0; j < h.size; j++) {
            crcSum += pInfo[j];
        }
        if(crcSum != info.crc) {
            return RESULT_FAIL;
        }

        mdNum = h.address >> 1; // 1,2,4
        if (mdNum > 2) {
            return RESULT_FAIL;
        }
        k0[mdNum] = info.k0 / 10000.00;
        k1[mdNum] = info.k1 / 10000.00;
        b0[mdNum] = info.b0 / 10000.00;
        b1[mdNum] = info.b1 / 10000.00;
        bias[mdNum] = double(info.bias) * 0.1;

        // debug("k0 %lf k1 %lf b0 %lf b1 %lf bias %lf", 
            // k0[mdNum], k1[mdNum], b0[mdNum], b1[mdNum], bias[mdNum]);
        delay(5);
    }
  }

  return RESULT_OK;
}

result_t GSLidarDriver::setDeviceAddress(uint32_t timeout)
{
    result_t ans;

    if (!m_isConnected) {
        return RESULT_FAIL;
    }

    if (m_SingleChannel) {
        return RESULT_OK;
    }

    //disableDataGrabbing();
    flushSerial();
    {
        ScopedLocker l(_cmd_lock);
        if ((ans = sendCommand(GS_LIDAR_CMD_GET_ADDRESS)) != RESULT_OK) {
            return ans;
        }
        gs_package_head h;
        if ((ans = waitResponseHeaderEx(&h, GS_LIDAR_CMD_GET_ADDRESS, timeout)) != RESULT_OK) {
            return ans;
        }
        moduleCount = (h.address >> 1) + 1;
        info("GS lidar count %u", moduleCount);
    }

    return RESULT_OK;
}

/************************************************************************/
/* the set to signal quality                                            */
/************************************************************************/
void GSLidarDriver::setIntensities(const bool &isintensities) 
{
    if (m_intensities != isintensities) {
        if (globalRecvBuffer) {
            delete[] globalRecvBuffer;
            globalRecvBuffer = NULL;
        }

        globalRecvBuffer = new uint8_t[GSPACKSIZE];
    }

    m_intensities = isintensities;

    if (m_intensities) {
        PackageSampleBytes = 2;
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
void GSLidarDriver::setAutoReconnect(const bool &enable) 
{
    isAutoReconnect = enable;
}

void GSLidarDriver::checkTransDelay() 
{
    //采样率
    trans_delay = _comm->getByteTime();
    sample_rate = 27 * 160;
    m_PointTime = 1e9 / sample_rate;
}

/************************************************************************/
/*  start to scan                                                       */
/************************************************************************/
result_t GSLidarDriver::startScan(bool force, uint32_t timeout) 
{
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

    //获取GS2参数
    gs_device_para gs2_info;
    ans = getDevicePara(gs2_info, 300);
    if (IS_OK(ans))
    {
        flushSerial();

        ScopedLocker l(_cmd_lock);
        if ((ans = sendCommand(GS_LIDAR_CMD_SCAN)) !=
            RESULT_OK) {
            return ans;
        }
        gs_package_head h;
        if ((ans = waitResponseHeaderEx(&h, GS_LIDAR_CMD_SCAN, timeout)) != RESULT_OK) {
            return ans;
        }
        //启动线程
        ans = createThread();
        m_isScanning = true;
    }

    return ans;
}

result_t GSLidarDriver::stopScan(uint32_t timeout) 
{
    UNUSED(timeout);
    result_t  ans;

    if (!m_isConnected) {
        return RESULT_FAIL;
    }

    ScopedLocker l(_cmd_lock);
    if ((ans = sendCommand(GS_LIDAR_CMD_STOP)) != RESULT_OK) {
      return ans;
    }
    gs_package_head h;
    if ((ans = waitResponseHeaderEx(&h, GS_LIDAR_CMD_STOP, timeout)) != RESULT_OK) {
        return ans;
    }
    delay(10);

    return RESULT_OK;
}

result_t GSLidarDriver::createThread()
{
    m_thread = new std::thread(&GSLidarDriver::cacheScanData, this);
    if (!m_thread)
    {
        error("Fail to create GS thread");
        return RESULT_FAIL;
    }

    info("Create GS thread 0x%X", m_thread->get_id());
    return RESULT_OK;
}

result_t GSLidarDriver::startAutoScan(bool force, uint32_t timeout) {
    result_t ans;

    if (!m_isConnected) {
        return RESULT_FAIL;
    }

    flushSerial();
    delay(10);
    {
        ScopedLocker l(_cmd_lock);
        if ((ans = sendCommand(GS_LIDAR_CMD_SCAN)) !=
                RESULT_OK) {
            return ans;
        }

        if (!m_SingleChannel) {
            gs_package_head h;
            if ((ans = waitResponseHeaderEx(&h, GS_LIDAR_CMD_SCAN, timeout)) != RESULT_OK) {
                return ans;
            }
        }
    }

    return RESULT_OK;
}

/************************************************************************/
/*   stop scan                                                   */
/************************************************************************/
result_t GSLidarDriver::stop() 
{
    disableDataGrabbing();
    stopScan();

    return RESULT_OK;
}

/************************************************************************/
/*  reset device                                                        */
/************************************************************************/
result_t GSLidarDriver::reset(uint8_t addr, uint32_t timeout) {
    UNUSED(timeout);
    result_t ans;

    if (!m_isConnected) {
        return RESULT_FAIL;
    }

    ScopedLocker l(_cmd_lock);

    if ((ans = sendCommand(addr, GS_LIDAR_CMD_RESET)) != RESULT_OK) {
        return ans;
    }

    return RESULT_OK;
}

std::string GSLidarDriver::getSDKVersion() {
    return YDLIDAR_SDK_VERSION_STR;
}

std::map<std::string, std::string> GSLidarDriver::lidarPortList() {
    std::vector<PortInfo> lst = list_ports();
    std::map<std::string, std::string> ports;

    for (std::vector<PortInfo>::iterator it = lst.begin(); it != lst.end(); it++) {
        std::string port = "ydlidar" + (*it).device_id;
        ports[port] = (*it).port;
    }

    return ports;
}

const char *GSLidarDriver::DescribeError(bool isTCP) 
{
  if (_comm) {
    return _comm->DescribeError();
  }
  return nullptr;
}

result_t GSLidarDriver::getHealth(device_health &, uint32_t) 
{
  return RESULT_OK;
}

result_t GSLidarDriver::getDeviceInfo(device_info &info, uint32_t timeout)
{
    result_t ret = RESULT_OK;

    if (!m_isConnected) {
        return RESULT_FAIL;
    }

    //获取设备信息（带雷达型号码）
    ret = getDeviceInfo2(info, timeout);
    if (!IS_OK(ret))
    {
        for (int i=0; i<moduleCount && i<LIDAR_MAXCOUNT; ++i)
            m_models[i] = YDLIDAR_GS2;
        info.model = YDLIDAR_GS2;
        //获取设备信息（不带雷达型号码）
        ret = getDeviceInfo1(info, timeout);
    }

    return ret;
}

result_t GSLidarDriver::getDeviceInfo(
    std::vector<device_info_ex> &dis,
    uint32_t timeout)
{
    //1、获取级联雷达数量
    result_t ret = setDeviceAddress(timeout);
    if (!IS_OK(ret))
    {
        error("Fail to get GS lidar count");
        return ret;
    }
    //2、获取设备信息（带雷达型号码）
    uint8_t c = moduleCount;
    ScopedLocker l(_lock);
    ret = sendCommand(GS_LIDAR_CMD_GET_VERSION3);
    if (!IS_OK(ret))
        return ret;
    for (uint8_t i=0; i<c; ++i)
    {
        gs_package_head head = {0};
        ret = waitResponseHeaderEx(&head, GS_LIDAR_CMD_GET_VERSION3, timeout);
        if (!IS_OK(ret))
            break;
        if (head.size < GSDEVINFO2SIZE)
        {
            ret = RESULT_FAIL;
            break;
        }
        ret = waitForData(head.size + 1, timeout);
        if (!IS_OK(ret))
            break;

        gs_device_info2 gsdi2;
        memset(&gsdi2, 0, GSDEVINFO2SIZE);
        getData(reinterpret_cast<uint8_t *>(&gsdi2), GSDEVINFO2SIZE);

        device_info_ex di;
        di.id = head.address >> 1;
        di.di.model = uint8_t(gsdi2.model);
        di.di.hardware_version = gsdi2.hwVersion;
        di.di.firmware_version = uint16_t((gsdi2.fwVersion & 0xFF) << 8) +
                              uint16_t(gsdi2.fwVersion >> 8);
        memcpy(di.di.serialnum, gsdi2.sn, SDK_SNLEN);
        dis.push_back(di);
    }
    if (IS_OK(ret))
        return ret;
    //3、获取设备信息（不带雷达型号码）
    ret = sendCommand(GS_LIDAR_CMD_GET_VERSION);
    for (uint8_t i=0; i<c; ++i)
    {
        gs_package_head head = {0};
        ret = waitResponseHeaderEx(&head, GS_LIDAR_CMD_GET_VERSION, timeout);
        if (!IS_OK(ret))
            return ret;
        if (head.size < GSDEVINFOSIZE) 
        {
            ret = RESULT_FAIL;
            break;
        }
        ret = waitForData(head.size + 1, timeout);
        if (!IS_OK(ret))
            break;

        gs_device_info gsdi = {0};
        getData(reinterpret_cast<uint8_t*>(&gsdi), sizeof(gsdi));

        device_info_ex di;
        di.id = head.address >> 1;
        di.di.model = YDLIDAR_GS2;
        di.di.hardware_version = gsdi.hwVersion;
        di.di.firmware_version = uint16_t((gsdi.fwVersion & 0xFF) << 8) +
                              uint16_t(gsdi.fwVersion >> 8);
        memcpy(di.di.serialnum, gsdi.sn, SDK_SNLEN);
        dis.push_back(di);
    }

    return ret;
}

result_t GSLidarDriver::getDeviceInfo1(device_info &info, uint32_t timeout)
{
    result_t ret = RESULT_FAIL;

    ScopedLocker l(_cmd_lock);
    if ((ret = sendCommand(GS_LIDAR_CMD_GET_VERSION)) != RESULT_OK) {
        return ret;
    }
    uint8_t c = std::min(moduleCount, uint8_t(LIDAR_MAXCOUNT));
    for (uint8_t i=0; i<c; ++i)
    {
        gs_package_head head;
        memset(&head, 0, GSPACKEGEHEADSIZE);
        if ((ret = waitResponseHeaderEx(&head, GS_LIDAR_CMD_GET_VERSION, timeout)) != RESULT_OK) {
            return ret;
        }
        if (head.size < sizeof(gs_device_info)) {
            return RESULT_FAIL;
        }
        if (waitForData(head.size + 1, timeout) != RESULT_OK) {
            return RESULT_FAIL;
        }

        gs_device_info di = {0};
        getData(reinterpret_cast<uint8_t*>(&di), sizeof(di));

        if (LIDAR_MODULE_1 == head.address)
        {
            info.hardware_version = di.hwVersion;
            info.firmware_version = uint16_t((di.fwVersion & 0xFF) << 8) +
                uint16_t(di.fwVersion >> 8);
            memcpy(info.serialnum, di.sn, SDK_SNLEN);
            // head.address; //雷达序号
            m_HasDeviceInfo |= EPT_Module | EPT_Base;
        }
    }

    return ret;
}

result_t GSLidarDriver::getDeviceInfo2(device_info &dev, uint32_t timeout)
{
    result_t ret = RESULT_FAIL;

    //获取设备信息，包含雷达型号（对外协议）
    ScopedLocker l(_lock);
    ret = sendCommand(GS_LIDAR_CMD_GET_VERSION3);
    if (!IS_OK(ret))
        return ret;
    uint8_t c = std::min(moduleCount, uint8_t(LIDAR_MAXCOUNT));
    for (uint8_t i=0; i<c; ++i)
    {
        gs_package_head head;
        memset(&head, 0, GSPACKEGEHEADSIZE);
        ret = waitResponseHeaderEx(&head, GS_LIDAR_CMD_GET_VERSION3, timeout);
        if (!IS_OK(ret))
            return ret;
        if (head.size < GSDEVINFO2SIZE)
            return RESULT_FAIL;
        ret = waitForData(head.size + 1, timeout);
        if (!IS_OK(ret))
            return ret;

        gs_device_info2 di;
        memset(&di, 0, GSDEVINFO2SIZE);
        getData(reinterpret_cast<uint8_t*>(&di), GSDEVINFO2SIZE);
        
        uint8_t id = uint8_t(head.address >> 1); //模组地址转编号: 1, 2, 4
        m_models[id] = di.model;
        info("Get Module[%d] Lidar model[%u]", id, di.model);
        if (LIDAR_MODULE_1 == head.address)
        {
            dev.model = uint8_t(di.model);
            dev.hardware_version = di.hwVersion;
            dev.firmware_version = uint16_t((di.fwVersion & 0xFF) << 8) +
                uint16_t(di.fwVersion >> 8);
            memcpy(dev.serialnum, di.sn, SDK_SNLEN);
            m_HasDeviceInfo |= EPT_Module | EPT_Base;
        }
    }

    return ret;
}

result_t GSLidarDriver::setWorkMode(int mode, uint8_t addr)
{
    result_t ans;
    uint32_t timeout = 300;
    string buf;

    if (!isconnected()) {
        return RESULT_FAIL;
    }

    //如果已经开启扫描，则先停止扫描
    if (isscanning())
    {
        disableDataGrabbing();
        delay(10);
        stopScan();
    }
    flushSerial();

    {
        ScopedLocker l(_cmd_lock);
        uint8_t m = uint8_t(mode);
        if ((ans = sendCommand(addr, GS_LIDAR_CMD_SET_MODE, &m, 1)) != RESULT_OK) {
            return ans;
        }
        gs_package_head response_header;
        if ((ans = waitResponseHeaderEx(&response_header, GS_LIDAR_CMD_SET_MODE, timeout)) != RESULT_OK) {
            return ans;
        }
        if (response_header.type != GS_LIDAR_CMD_SET_MODE) {
            return RESULT_FAIL;
        }
    }

    return RESULT_OK;
}

bool GSLidarDriver::ota()
{
    if (m_OtaName.empty())
    {
        error("[OTA] Not set OTA file");
        return false;
    }
    // 读取文件所有内容
    std::ifstream f;
    f.open(m_OtaName, ios::in | ios::binary);
    if (!f.is_open())
    {
        error("[OTA] Fail to open OTA file[%s]", m_OtaName.c_str());
        return false;
    }
    //读数据
    std::vector<uint8_t> data;
    while (!f.eof())
    {
        std::vector<uint8_t> d(DATA_LEN_PER_FRAME);
        memset(d.data(), GS_CMD_ZERO, d.size());
        f.read(reinterpret_cast<char*>(d.data()), d.size());
        int s = f.gcount(); //获取读取成功的字节数
        for (int i=0; i<s; ++i)
            data.push_back(d.at(i));
    }
    info("[OTA] File size [%.02lf]KB", data.size() / 1024.0);

    int count = moduleCount; // 雷达数量
    for (int i = 0; i < count; ++i)
    {
        uint8_t addr = 1 << i;
        // 开始OTA
        if (!startOta(addr))
        {
            error("[OTA] 0x%02X Fail to Start OTA", addr);
            return false;
        }

        // 下载数据
        if (!execOta(addr, data))
        {
            error("[OTA] 0x%02X Fail to download data", addr);
            return false;
        }

        // 停止OTA
        if (!stopOta(addr))
        {
            error("[OTA] 0x%02X Fail to Start OTA", addr);
            return false;
        }

        // 重启雷达
        if (!IS_OK(reset(addr, TIMEOUT_1S)))
        {
            error("[OTA] 0x%02X Fail to restart gs lidar", addr);
            return false;
        }

        info("[OTA] 0x%02X Success to finish OTA", addr);
    }

    return true;
}

bool GSLidarDriver::startOta(uint8_t addr)
{
    //发送启动OTA命令
    std::vector<uint8_t> d;
    uint8_t dsr[] = {0x00, 0x00,
                    0x73, 0x74, 0x61, 0x72, 0x74, 0x20, 0x64, 0x6F,
                    0x77, 0x6E, 0x6C, 0x6F, 0x61, 0x64, 0x00, 0x00};
    for (int i = 0; i < sizeof(dsr); ++i)
        d.push_back(dsr[i]);
    std::vector<uint8_t> dataRecv;
    bool ret = sendData(
        addr,
        GS_CMD_STARTIAP,
        d,
        GS_CMD_ACKIAP,
        dataRecv,
        TIMEOUT_500);
    if (ret)
    {
        ret = isOtaRespOk(
            addr,
            GS_CMD_STARTIAP,
            GS_CMD_ZERO,
            dataRecv);
    }

    return ret;
}

bool GSLidarDriver::execOta(uint8_t addr, const std::vector<uint8_t>& data)
{
    // 数据中固定部分（字符串“downloading”）
    uint8_t fix[] = {0x64, 0x6F, 0x77, 0x6E, 0x6C, 0x6F, 0x61, 0x64,
                     0x69, 0x6E, 0x67, 0x00, 0x00, 0x00, 0x00, 0x00};
    bool ret = false;
    // 计算固件分成的数据包数
    int n = data.size() % DATA_LEN_PER_FRAME;
    int m = data.size() / DATA_LEN_PER_FRAME + (n ? 1 : 0); 

    int percent = -1;
    for (int j = 0; j < m; ++j)
    {
        // 打印进度
        int p = int(j * 100.0 / m);
        if (p != percent)
        {
            percent = p;
            info("[OTA] Downloading [%d%%]", p);
        }

        std::vector<uint8_t> d;
        int offset = j * DATA_LEN_PER_FRAME; // 数据偏移量
        d.push_back(offset & 0xFF);
        d.push_back(offset >> 8);
        for (int i = 0; i < sizeof(fix); ++i)
            d.push_back(fix[i]);
        for (int i = offset; i < offset + DATA_LEN_PER_FRAME; ++i)
        {
            if (i < data.size())
                d.push_back(data.at(i));
            else
                d.push_back(GS_CMD_ZERO);
        }

        std::vector<uint8_t> dataRecv;
        ret = sendData(
            addr,
            GS_CMD_EXECIAP,
            d,
            GS_CMD_ACKIAP,
            dataRecv,
            TIMEOUT_500);
        if (ret)
        {
            ret = isOtaRespOk(
                addr,
                GS_CMD_EXECIAP,
                uint16_t(offset),
                dataRecv);
        }
        if (!ret)
        {
            error("[OTA] Fail to download [%d] package", j + 1);
            break;
        }
    }

    return ret;
}

bool GSLidarDriver::stopOta(uint8_t addr)
{
    std::vector<uint8_t> d;

    uint8_t dsr[] = {0x00, 0x00,
                     0x63, 0x6F, 0x6D, 0x70, 0x6C, 0x65, 0x74, 0x65,
                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    for (int i = 0; i < sizeof(dsr); ++i)
        d.push_back(dsr[i]);
    // 是否加密标识
    d.push_back(m_OtaEncode);
    d.push_back(GS_CMD_ZERO);
    d.push_back(GS_CMD_ZERO);
    d.push_back(GS_CMD_ZERO);

    std::vector<uint8_t> dataRecv;
    bool ret = sendData(
        addr,
        GS_CMD_STOPIAP,
        d,
        GS_CMD_ACKIAP,
        dataRecv,
        TIMEOUT_500);
    if (ret)
    {
        ret = isOtaRespOk(
            addr,
            GS_CMD_STOPIAP,
            GS_CMD_ZERO,
            dataRecv);
    }

    return ret;
}

bool GSLidarDriver::isOtaRespOk(
    uint8_t addr,
    uint8_t cmd,
    uint16_t offset,
    const std::vector<uint8_t> &data)
{
    std::vector<uint8_t> d;
    d.push_back(LIDAR_CMD_SYNC_BYTE);
    d.push_back(LIDAR_CMD_SYNC_BYTE);
    d.push_back(LIDAR_CMD_SYNC_BYTE);
    d.push_back(LIDAR_CMD_SYNC_BYTE);
    d.push_back(addr);
    d.push_back(GS_CMD_ACKIAP);
    uint16_t len = 4;
    d.push_back(len & 0xFF);
    d.push_back(len >> 8);
    d.push_back(offset & 0xFF);
    d.push_back(offset >> 8);
    d.push_back(cmd);
    d.push_back(GS_CMD_ACKOK);
    //计算8位校验和
    uint8_t cs = 0;
    for (int k=4; k<d.size(); ++k)
        cs += uint8_t(d.at(k));
    d.push_back(cs);

    return d == data;
}

bool GSLidarDriver::sendData(
    uint8_t addr,
    uint8_t cmd,
    const std::vector<uint8_t> &data,
    uint8_t cmdRecv,
    std::vector<uint8_t> &dataRecv,
    int timeout)
{
    std::vector<uint8_t> d;
    d.push_back(LIDAR_CMD_SYNC_BYTE);
    d.push_back(LIDAR_CMD_SYNC_BYTE);
    d.push_back(LIDAR_CMD_SYNC_BYTE);
    d.push_back(LIDAR_CMD_SYNC_BYTE);
    d.push_back(addr);
    d.push_back(cmd);
    uint16_t len = uint16_t(data.size());
    d.push_back(len & 0xFF);
    d.push_back(len >> 8);
    for (size_t i=0; i<data.size(); ++i)
        d.push_back(data.at(i));

    bool ret = false;
    std::vector<uint8_t> ds;
    //计算8位校验和
    uint8_t cs = 0;
    for (int k=4; k<d.size(); ++k)
        cs += uint8_t(d.at(k));
    d.push_back(cs);

    flushSerial();
    ScopedLocker l(_cmd_lock);
    result_t r = sendData(d.data(), d.size());
    if (!IS_OK(r))
        return ret;
    gs_package_head head = {0};
    r = waitResponseHeaderEx(&head, cmdRecv, timeout);
    if (!IS_OK(r))
        return ret;
    r = waitForData(head.size + 1, timeout);
    if (!IS_OK(r))
        return ret;
    std::vector<uint8_t> dRecv(GSPACKEGEHEADSIZE + head.size + 1);
    memcpy(&dRecv[0], &head, GSPACKEGEHEADSIZE);
    r = getData(&dRecv[GSPACKEGEHEADSIZE], head.size + 1);
    if (IS_OK(r))
    {
        dataRecv = dRecv;
        ret = true;
    }
        
    return ret;
}

}
