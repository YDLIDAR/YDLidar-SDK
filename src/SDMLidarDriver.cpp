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
#include <algorithm>
#include "SDMLidarDriver.h"
#include "core/serial/common.h"
#include "ydlidar_config.h"

using namespace impl;

namespace ydlidar
{

SDMLidarDriver::SDMLidarDriver() 
    : _serial(NULL)
{
    // 串口配置参数
    m_intensities = false;
    isAutoReconnect = true;
    isAutoconnting = false;
    m_baudrate = 230400;
    m_PointTime = 1e9 / 5000;
    retryCount = 0;
    m_SingleChannel = false;
    m_LidarType = TYPE_SDM;

    nodeIndex = 0;
    recvBuff = std::vector<uint8_t>(SDKSDMPCSSIZE, 0);

    scan_node_count = 0;
    scan_node_buf = new node_info[SDK_SDM_POINT_COUNT * 5];
}

SDMLidarDriver::~SDMLidarDriver()
{
    m_isScanning = false;
    isAutoReconnect = false;
    _thread.join();

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

    if (scan_node_buf)
    {
        delete[] scan_node_buf;
        scan_node_buf = NULL;
    }
}

result_t SDMLidarDriver::connect(const char *port, uint32_t baudrate)
{
    m_baudrate = baudrate;
    m_port = string(port);
    {
        ScopedLocker l(_cmd_lock);
        if (!_serial)
        {
            _serial = new serial::Serial(
                m_port,
                m_baudrate,
                serial::Timeout::simpleTimeout(DEFAULT_TIMEOUT));
        }
        if (!_serial->open())
        {
            return RESULT_FAIL;
        }

        m_isConnected = true;
    }

    stopScan();
    clearDTR();

    return RESULT_OK;
}

void SDMLidarDriver::setDTR()
{
    if (!m_isConnected)
        return;

    if (_serial)
        _serial->setDTR(1);
}

void SDMLidarDriver::clearDTR()
{
    if (!m_isConnected)
        return;

    if (_serial)
        _serial->setDTR(0);
}

void SDMLidarDriver::flushSerial()
{
    if (!m_isConnected)
    {
        return;
    }

    size_t len = _serial->available();
    if (len)
    {
        _serial->readSize(len);
    }

    delay(20);
}

void SDMLidarDriver::disconnect()
{
    isAutoReconnect = false;

    if (!m_isConnected)
    {
        return;
    }

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

void SDMLidarDriver::disableDataGrabbing()
{
    if (m_isScanning)
    {
        m_isScanning = false;
        _dataEvent.set();
    }
    _thread.join();
}

bool SDMLidarDriver::isscanning() const
{
    return m_isScanning;
}
bool SDMLidarDriver::isconnected() const
{
    return m_isConnected;
}

result_t SDMLidarDriver::sendCmd(
    uint8_t cmd,
    const uint8_t *data,
    size_t dataSize)
{
    if (!m_isConnected)
        return RESULT_FAIL;

    size_t size = SDKSDMHEADSIZE + dataSize;
    std::vector<uint8_t> buff(size + 1, 0);

    SdkSdmHead head;
    head.head0 = SDK_CMD_HEADFLAG0;
    head.head1 = SDK_CMD_HEADFLAG1;
    head.cmd = cmd;
    head.size = uint8_t(dataSize);
    memcpy(&buff[0], &head, SDKSDMHEADSIZE);

    if (data && dataSize)
        memcpy(&buff[SDKSDMHEADSIZE], data, dataSize);

    uint8_t cs = 0;
    for (size_t i = 0; i < size; ++i)
        cs += buff[i];
    buff[size] = cs;

    return sendData(buff.data(), buff.size());
}

result_t SDMLidarDriver::sendData(const uint8_t *data, size_t size)
{
    if (!_serial || !_serial->isOpen())
        return RESULT_FAIL;

    if (!data || !size)
        return RESULT_FAIL;

    size_t r = 0;
    while (size)
    {
        r = _serial->writeData(data, size);
        if (!r)
            return RESULT_FAIL;

        if (m_Debug)
        {
            printf("send: ");
            infoh(data, r);
        }

        size -= r;
        data += r;
    }

    return RESULT_OK;
}

result_t SDMLidarDriver::getData(uint8_t *data, size_t size)
{
    if (!_serial || !_serial->isOpen())
    {
        return RESULT_FAIL;
    }

    size_t r;

    while (size)
    {
        r = _serial->readData(data, size);
        if (!r)
            return RESULT_FAIL;

        if (m_Debug)
        {
            printf("recv: ");
            infoh(data, r);
        }

        size -= r;
        data += r;
    }

    return RESULT_OK;
}

result_t SDMLidarDriver::waitResp(
    uint8_t cmd,
    uint32_t timeout)
{
    std::vector<uint8_t> data;
    return waitResp(cmd, data, timeout);
}

result_t SDMLidarDriver::waitResp(
    uint8_t cmd,
    std::vector<uint8_t> &data,
    uint32_t timeout)
{
    int pos = 0;
    uint32_t st = getms();
    uint32_t wt = 0;
    std::vector<uint8_t> recvBuff(SDK_BUFFER_MAXLEN, 0);
    SdkSdmHead head;
    uint8_t *buff = reinterpret_cast<uint8_t*>(&head);
    uint8_t cs = 0;
    uint8_t dataSize = 0;

    while ((wt = getms() - st) <= timeout)
    {
        size_t srcSize = SDKSDMHEADSIZE - pos;
        size_t dstSize = 0;

        result_t ans = waitForData(srcSize, timeout - wt, &dstSize);
        if (!IS_OK(ans))
            return ans;

        getData(recvBuff.data(), dstSize);

        for (size_t i = 0; i < dstSize; ++i)
        {
            uint8_t c = recvBuff[i];
            switch (pos)
            {
            case 0:
                if (c != SDK_CMD_HEADFLAG0)
                {
                    pos = 0;
                    continue;
                }
                break;
            case 1:
                if (c != SDK_CMD_HEADFLAG1)
                {
                    pos = 0;
                    continue;
                }
                break;
            case 2:
                if (c != cmd) //判断解析到的命令字是否和指定命令字是否一致
                {
                    pos = 0;
                    cs = 0;
                    continue;
                }
                break;
            case 3:
                dataSize = c;
                break;
            default:
                break;
            }

            buff[pos++] = c;
            cs += c;
        }

        //如果找到协议头
        if (pos == SDKSDMHEADSIZE)
        {
            //获取剩余数据，并计算校验和
            size_t srcSize = dataSize + 1;
            size_t dstSize = 0;
            result_t ans = waitForData(srcSize, timeout - wt, &dstSize);
            if (!IS_OK(ans))
                return ans;
            getData(recvBuff.data(), dstSize);
            for (size_t i = 0; i < dataSize; ++i)
            {
                cs += recvBuff[i];
                data.push_back(recvBuff[i]); //数据存入输出参数
            }
            if (cs != recvBuff[dataSize]) //判断校验和是否一致
            {
                printf("[YDLIDAR] CMD CheckSum error calc[0x%02X] != src[0x%02X]\n",
                        cs, recvBuff[dataSize]);
                return RESULT_FAIL;
            }
            return RESULT_OK;
        }
    }

    return RESULT_FAIL;
}

result_t SDMLidarDriver::waitForData(
    size_t srcSize,
    uint32_t timeout,
    size_t *dstSize)
{
    size_t size = 0;
    if (!dstSize)
        dstSize = &size;

    result_t ret = _serial->waitfordata(srcSize, timeout, dstSize);
    if (IS_OK(ret))
    {
        if (*dstSize > srcSize)
            *dstSize = srcSize;
    }
    return ret;
}

result_t SDMLidarDriver::checkAutoConnecting()
{
    result_t ans = RESULT_FAIL;
    isAutoconnting = true;

    while (isAutoReconnect && isAutoconnting)
    {
        {
            ScopedLocker l(_cmd_lock);
            if (_serial)
            {
                if (_serial->isOpen() || m_isConnected)
                {
                    m_isConnected = false;
                    _serial->closePort();
                    delete _serial;
                    _serial = NULL;
                }
            }
        }
        retryCount ++;
        if (retryCount > 10)
        {
            retryCount = 10;
        }

        delay(100 * retryCount);
        int retryConnect = 0;

        while (isAutoReconnect &&
            connect(m_port.c_str(), m_baudrate) != RESULT_OK)
        {
            retryConnect++;
            if (retryConnect > 10)
            {
                retryConnect = 10;
            }

            delay(100);
        }

        if (!isAutoReconnect)
        {
            m_isScanning = false;
            return RESULT_FAIL;
        }

        if (isconnected())
        {
            delay(100);
            {
                ans = startAutoScan();
                if (!IS_OK(ans))
                {
                    ans = startAutoScan();
                }
            }

            if (IS_OK(ans))
            {
                isAutoconnting = false;
                return ans;
            }
        }
    }

    return RESULT_FAIL;
}

int SDMLidarDriver::cacheScanData()
{
    node_info local_buf[SDK_SDM_POINT_COUNT];
    size_t count = SDK_SDM_POINT_COUNT;
    size_t scan_count = 0;
    result_t ret = RESULT_FAIL;

    int timeout_count = 0;
    retryCount = 0;

    m_isScanning = true;

    while (m_isScanning)
    {
        count = SDK_SDM_POINT_COUNT;
        ret = waitScanData(local_buf, count);
        if (!IS_OK(ret)) // 如果解析点云失败
        {
            if (IS_FAIL(ret) ||
                timeout_count > DEFAULT_TIMEOUT_COUNT)
            {
                if (!isAutoReconnect)
                {
                    fprintf(stderr, "[YDLIDAR] Exit scanning thread!\n");
                    fflush(stderr);
                    m_isScanning = false;
                    return RESULT_FAIL;
                }
                else
                {
                    ret = checkAutoConnecting();
                    if (IS_OK(ret))
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
                timeout_count ++;
                fprintf(stderr, "[YDLIDAR] Timeout count %d\n", timeout_count);
                fflush(stderr);
            }
            continue;
        }
        else
        {
            timeout_count = 0;
            retryCount = 0;

            // printf("[YDLIDAR] SDM points Stored in buffer %lu\n", count);
            ScopedLocker l(_lock);
            memcpy(scan_node_buf, local_buf, sizeof(node_info) * SDK_SDM_POINT_COUNT);
            scan_node_count = SDK_SDM_POINT_COUNT; // 一个包固定1个点
            _dataEvent.set();
            scan_count = 0;
        }
    }

    m_isScanning = false;

    return RESULT_OK;
}

result_t SDMLidarDriver::waitPackage(node_info *node, uint32_t timeout)
{
    int pos = 0;
    uint32_t st = getms();
    uint32_t wt = 0;
    uint8_t dataSize = 0;
    uint8_t cs = 0;
    result_t ret = RESULT_FAIL;
    SdkSdmPcs pcs;
    uint8_t *buff = reinterpret_cast<uint8_t *>(&pcs);

    memset(node, 0, SDKNODESIZE);

    while ((wt = getms() - st) <= timeout)
    {
        size_t srcSize = SDKSDMHEADSIZE - pos;
        size_t dstSize = 0;

        result_t ans = waitForData(srcSize, timeout - wt, &dstSize);
        if (!IS_OK(ans))
            return ans;

        getData(recvBuff.data(), dstSize);

        for (size_t i = 0; i < dstSize; ++i)
        {
            uint8_t c = recvBuff[i];
            switch (pos)
            {
            case 0:
                if (c != SDK_CMD_HEADFLAG0)
                {
                    pos = 0;
                    continue;
                }
                break;
            case 1:
                if (c != SDK_CMD_HEADFLAG1)
                {
                    pos = 0;
                    continue;
                }
                break;
            case 2:
                if (c != SDK_CMD_STARTSCAN)
                {
                    pos = 0;
                    cs = 0;
                    continue;
                }
                break;
            case 3:
                dataSize = c;
                if (!c) // 如果数据长度无效则跳过
                {
                    pos = 0;
                    cs = 0;
                    continue;
                }
                break;
            default:
                break;
            }

            buff[pos++] = c;
            cs += c;
        }

        // 如果找到协议头
        if (pos == SDKSDMHEADSIZE)
        {
            // 获取剩余数据，并计算校验和
            size_t srcSize = dataSize + 1;
            size_t dstSize = 0;
            ret = waitForData(srcSize, timeout - wt, &dstSize);
            if (!IS_OK(ret))
                return ret;
            getData(recvBuff.data(), dstSize);
            for (size_t i = 0; i < dataSize; ++i)
            {
                cs += recvBuff[i];
            }
            if (cs != recvBuff[dataSize])
            {
                printf("[YDLIDAR] PC CheckSum error calc[0x%02X] != src[0x%02X]\n",
                       cs, recvBuff[dataSize]);
                return RESULT_FAIL;
            }
            memcpy(&buff[SDKSDMHEADSIZE], recvBuff.data(), dstSize);
            break;
        }
    }

    if (IS_OK(ret))
    {
        (*node).sync = NODE_SYNC;
        (*node).stamp = getTime();
        (*node).index = 0;
        (*node).scanFreq = m_ScanFreq;
        (*node).qual = 0;

        (*node).dist = pcs.point.dist;
        (*node).angle = 0;
        (*node).qual = pcs.point.intensity;
        (*node).is = pcs.point.env;
        return RESULT_OK;
    }
    else
    {
        return RESULT_FAIL;
    }
}

result_t SDMLidarDriver::waitScanData(
    node_info *nodes,
    size_t &count,
    uint32_t timeout)
{
    result_t ret = RESULT_FAIL;
    if (!m_isConnected)
    {
        count = 0;
        return ret;
    }

    size_t recvCount = 0;
    uint32_t st = getms();
    uint32_t wt = 0;

    while ((wt = getms() - st) <= timeout &&
        recvCount < count)
    {
        node_info node;
        memset(&node, 0, SDKNODESIZE);
        ret = waitPackage(&node, timeout - wt);
        if (!IS_OK(ret))
        {
            count = recvCount;
            return ret;
        }

        nodes[recvCount++] = node;

        if (recvCount == count)
            return RESULT_OK;
    }

    count = recvCount;
    return RESULT_FAIL;
}

result_t SDMLidarDriver::grabScanData(
    node_info *nodebuffer,
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
        memcpy(nodebuffer, scan_node_buf, size_to_copy * SDKNODESIZE);
        count = size_to_copy;
        scan_node_count = 0;
        return RESULT_OK;
    }
    default:
        count = 0;
        return RESULT_FAIL;
    }
}

/**
 * @brief 设置雷达异常自动重新连接 \n
 * @param[in] enable    是否开启自动重连:
 *     true	开启
 *	  false 关闭
    */
void SDMLidarDriver::setAutoReconnect(const bool &enable)
{
    isAutoReconnect = enable;
}

// void SDMLidarDriver::checkTransDelay()
// {
//     //采样率
//     trans_delay = _serial->getByteTime();
//     sample_rate = 27 * 160;
//     m_PointTime = 1e9 / sample_rate;
// }

/************************************************************************/
/*  start to scan                                                       */
/************************************************************************/
result_t SDMLidarDriver::startScan(bool force, uint32_t timeout)
{
    result_t ret = RESULT_FAIL;

    if (!m_isConnected)
        return RESULT_FAIL;
    if (m_isScanning)
        return RESULT_OK;

    stopScan(); //启动前先停止
    //启动前先设置扫描频率
    if (!IS_OK(setScanFreq(m_ScanFreq, 500)))
    {
        printf("[YDLIDAR] Set scan frequency %.01f failed!\n", m_ScanFreq);
        return RESULT_FAIL;
    }

    {
        //发送启动雷达命令
        ScopedLocker l(_cmd_lock);
        if ((ret = sendCmd(SDK_CMD_STARTSCAN)) != RESULT_OK)
            return ret;
        //双通雷达才发送启动命令
        if (!m_SingleChannel)
        {
            ret = waitResp(SDK_CMD_STARTSCAN, timeout);
            if (!IS_OK(ret))
            {
                printf("[YDLIDAR] Response to start scan error!\n");
                return ret;
            }
        }

        ret = createThread();
    }

    return ret;
}

result_t SDMLidarDriver::stopScan(uint32_t timeout)
{
    UNUSED(timeout);
    result_t ans;

    if (!m_isConnected)
        return RESULT_FAIL;

    ScopedLocker l(_cmd_lock);
    if ((ans = sendCmd(SDK_CMD_STOPSCAN)) != RESULT_OK)
    {
        return ans;
    }
    if ((ans = waitResp(SDK_CMD_STOPSCAN, timeout)) != RESULT_OK)
    {
        return ans;
    }

    delay(10);

    return RESULT_OK;
}

result_t SDMLidarDriver::createThread()
{
    // 如果线程已启动，则先退出线程
    if (_thread.getHandle())
    {
        m_isScanning = false;
        _thread.join();
    }
    _thread = CLASS_THREAD(SDMLidarDriver, cacheScanData);
    if (!_thread.getHandle())
    {
        printf("[YDLIDAR] Fail to create SDM thread\n");
        return RESULT_FAIL;
    }

    printf("[YDLIDAR] Create SDM thread 0x%X\n", _thread.getHandle());
    fflush(stdout);

    return RESULT_OK;
}

result_t SDMLidarDriver::startAutoScan(bool force, uint32_t timeout)
{
    result_t ans;

    if (!m_isConnected)
        return RESULT_FAIL;

    flushSerial();
    delay(10);
    {
        ScopedLocker l(_cmd_lock);
        if ((ans = sendCmd(SDK_CMD_STARTSCAN)) !=
            RESULT_OK)
        {
            return ans;
        }
        if (!m_SingleChannel)
        {
            if ((ans = waitResp(SDK_CMD_STARTSCAN, timeout)) != RESULT_OK)
            {
                return ans;
            }
        }
    }

    return RESULT_OK;
}

/************************************************************************/
/*   stop scan                                                   */
/************************************************************************/
result_t SDMLidarDriver::stop()
{
    if (isAutoconnting)
        isAutoReconnect = false;

    disableDataGrabbing();
    stopScan();
    flushSerial();

    return RESULT_OK;
}

/************************************************************************/
/*  reset device                                                        */
/************************************************************************/
result_t SDMLidarDriver::reset(uint8_t addr, uint32_t timeout)
{
    UNUSED(timeout);
    result_t ans;

    if (!m_isConnected)
    {
        return RESULT_FAIL;
    }

    ScopedLocker l(_cmd_lock);
    if ((ans = sendCmd(SDK_CMD_RESET)) != RESULT_OK)
    {
        return ans;
    }

    return RESULT_OK;
}

result_t SDMLidarDriver::enableFilter(bool yes)
{
    result_t ret = RESULT_FAIL;
    if (!m_isConnected)
        return ret;

    ScopedLocker l(_cmd_lock);
    uint8_t e = yes ? 0x01 : 0x00;
    if ((ret = sendCmd(SDK_CMD_SETFILTER, &e, 1)) != RESULT_OK)
        return ret;

    return ret;
}

result_t SDMLidarDriver::setScanFreq(float sf, uint32_t timeout)
{
    result_t ret = RESULT_FAIL;
    if (!m_isConnected)
        return ret;

    static float s_sfs[] = {10.01, 100.01, 200.01, 500.01, 1000.01, 1800.01};
    uint8_t d = 0x00;
    int size = sizeof(s_sfs) / sizeof(float);
    for (int i=0; i<size; ++i)
    {
        if (sf < s_sfs[i])
        {
            d = i;
            break;
        }
    }
    ScopedLocker l(_cmd_lock);
    if ((ret = sendCmd(SDK_CMD_SETFREQ, &d, 1)) != RESULT_OK)
        return ret;

    std::vector<uint8_t> data;
    ret = waitResp(SDK_CMD_SETFREQ, data, timeout);
    if (!IS_OK(ret))
        return ret;
    if (!data.size())
        return RESULT_FAIL;
    d = data.at(0);
    //根据返回的扫描频率更新当前扫描频率
    for (int i=0; i<size; ++i)
    {
        if (d == i)
        {
            m_ScanFreq = s_sfs[i];
            break;
        }
    }

    return ret;
}

std::string SDMLidarDriver::getSDKVersion()
{
    return YDLIDAR_SDK_VERSION_STR;
}

const char *SDMLidarDriver::DescribeError(bool isTCP)
{
    if (_serial)
    {
        return _serial->DescribeError();
    }
    return nullptr;
}

result_t SDMLidarDriver::getHealth(device_health &health, uint32_t timeout)
{
    return RESULT_OK;
}

result_t SDMLidarDriver::getDeviceInfo(device_info &info, uint32_t timeout)
{
    result_t ret = RESULT_OK;

    if (!m_isConnected)
        return RESULT_FAIL;

    disableDataGrabbing();

    ScopedLocker l(_cmd_lock);
    if ((ret = sendCmd(SDK_CMD_GETVERSION)) != RESULT_OK)
        return ret;

    std::vector<uint8_t> data;
    if ((ret = waitResp(SDK_CMD_GETVERSION, data, timeout)) != RESULT_OK)
        return ret;

    // printf("%s %llu\n", __FUNCTION__, data.size());

    if (data.size() == SDKSDMDEVICEINFOSIZE) //新版设备信息（带序列号）
    {
        SdkSdmDeviceInfo di;
        memcpy(&di, data.data(), data.size());
        info.model = di.model;
        info.hardware_version = di.hv;
        info.firmware_version = (uint16_t(di.fvm) << 8) | uint16_t(di.fvs);
        memcpy(info.serialnum, di.sn, SDK_SNLEN);
        return RESULT_OK;
    }
    else if (data.size() == 3) //旧版设备信息（不带序列号）
    {
        info.model = YDLIDAR_SDM15;
        info.hardware_version = data[0];
        info.firmware_version = (uint16_t(data[1]) << 8) | uint16_t(data[2]);
        return RESULT_OK;
    }
    else
    {
        return RESULT_FAIL;
    }
}

} //end namespace ydlidar
