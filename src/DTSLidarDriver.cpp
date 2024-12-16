#include <math.h>
#include <algorithm>
#include "DTSLidarDriver.h"
#include "core/serial/common.h"
#include "ydlidar_config.h"


using namespace impl;
namespace ydlidar
{

DTSLidarDriver::DTSLidarDriver()
    : _serial(NULL)
{
    //串口配置参数
    isAutoReconnect = true;
    isAutoconnting = false;
    m_baudrate = 921600;
    m_PointTime = 1e9 / 5000;
    retryCount = 0;
    m_SingleChannel = false;
    m_LidarType = TYPE_SDM18;

    nodeIndex = 0;
    recvBuff = std::vector<uint8_t>(SDKDTSPCSSIZE, 0);
    scan_node_count = 0;
    scan_node_buf = new node_info[SDK_DTS_POINT_COUNT * 5];
}

DTSLidarDriver::~DTSLidarDriver()
{
    m_isScanning = false;
    isAutoReconnect = false;
    _thread.join();

    {
        ScopedLocker l(_cmd_lock);
        if (_serial)
        {
            if (_serial->isOpen())
            {
                _serial->flush();
                _serial->closePort();
            }
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
        scan_node_count = 0;
    }
}

result_t DTSLidarDriver::connect(const char *port, uint32_t baudrate)
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
    //clearDTR();

    return RESULT_OK;
}

void DTSLidarDriver::disconnect()
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

result_t DTSLidarDriver::stopScan(uint32_t timeout)
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

result_t DTSLidarDriver::stop()
{
    if (isAutoconnting)
        isAutoReconnect = false;

    disableDataGrabbing();
    stopScan();
    flushSerial();

    return RESULT_OK;
}

/*
 * @brief 获取激光数据
 * @param nodebuffer out: 激光点信息
 * @param count      in: 一圈激光点数
 * @param timeout    in: 超时时间
 * @return
 */
result_t DTSLidarDriver::grabScanData(
    node_info *nodebuffer, 
    size_t &count, 
    uint32_t timeout)
{
    result_t ret = RESULT_FAIL;
    switch (_dataEvent.wait(timeout))
    {
    case Event::EVENT_TIMEOUT:
        count = 0;
        ret = RESULT_TIMEOUT;
        break;
    case Event::EVENT_OK:
    {
        ScopedLocker l(_lock);
        count = min(count, scan_node_count);
        memcpy(nodebuffer, scan_node_buf, count * SDKNODESIZE);
        scan_node_count = 0;
        ret = RESULT_OK;
        _dataEvent.set(false); //重置状态
        break;
    }
    default:
        count = 0;
        ret = RESULT_FAIL;
        break;
    }

    return ret;
}

/*
 * @brief 等待扫描数据
 * @param nodes   out:存储节点信息的数组
 * @param count   out:节点信息数组的大小，传入时表示期望接收的节点数量，返回时表示实际接收的节点数量
 * @param timeout in:超时时间（毫秒）
 * @return result_t 操作结果，成功返回RESULT_OK，失败返回RESULT_FAIL
 */
result_t DTSLidarDriver::waitScanData(
    node_info *nodes,
    size_t &count, 
    uint32_t timeout)
{
    result_t ret = RESULT_FAIL;
    // 检查是否已连接
    if (!m_isConnected)
    {
        count = 0;
        return RESULT_FAIL;
    }

    size_t recvCount = 0;
    uint32_t st = getms();
    uint32_t wt = 0;

    //循环等待接收节点数据
    while ((wt = getms() - st) < timeout && 
        recvCount < count)
    {
        node_info node;
        memset(&node, 0, SDKNODESIZE);
        //解包激光数据
        ret = waitPackage(&node, timeout - wt);
        if (!IS_OK(ret))
        {
            count = recvCount;
            return ret;
        }
        //单点
        nodes[recvCount++] = node;
        if (recvCount == count)
            return RESULT_OK;
    }

    count = recvCount;
    return RESULT_FAIL;
}

//激光数据解析线程
int DTSLidarDriver::cacheScanData()
{
    node_info local_buf[SDK_DTS_POINT_COUNT];
    size_t count = SDK_DTS_POINT_COUNT;
    result_t ret = RESULT_FAIL;
    int timeout_count = 0;
    retryCount = 0;
    m_isScanning = true;
    while (m_isScanning)
    {
        count = SDK_DTS_POINT_COUNT;
        ret = waitScanData(local_buf, count);
        //如果解析点云失败
        if (!IS_OK(ret))
        {
            if (timeout_count > DEFAULT_TIMEOUT_COUNT)
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
        }
        else
        {
            timeout_count = 0;
            retryCount = 0;

            ScopedLocker l(_lock);
            memcpy(scan_node_buf, local_buf, sizeof(node_info) * count);
            scan_node_count = count;
            _dataEvent.set();
        }
    }
    m_isScanning = false;
    return RESULT_OK;
}

/*
 *@brief 创建解析雷达数据线程
 *@note 创建解析雷达数据线程之前，必须使用startScan函数开启扫图成功
 */
result_t DTSLidarDriver::createThread()
{
    _thread = CLASS_THREAD(DTSLidarDriver, cacheScanData);
    if (!_thread.getHandle())
    {
        m_isScanning = false;
        printf("[YDLIDAR] Fail to create DTS thread\n");
        return RESULT_FAIL;
    }
    printf("[YDLIDAR] Create DTS thread [0x%X]\n", _thread.getHandle());
    fflush(stdout);
    m_isScanning = true;

    return RESULT_OK;
}

result_t DTSLidarDriver::startScan(bool force, uint32_t timeout)
{
    result_t ret = RESULT_FAIL;
    if (!m_isConnected)
        return RESULT_FAIL;
    if (m_isScanning)
        return RESULT_OK;
    //启动前先停止
    stopScan();
    //设置默认扫描频率(无)
    ret = setScanFreq(10.0, timeout);
    if (!IS_OK(ret))
    {
        printf("[YDLIDAR] Fail to setting scan frequency\n");
        return ret;
    }
    //获取校准参数
//    ret = getCalibParam(timeout);
//    if (!IS_OK(ret))
//    {
//        return ret;
//    }

    //发送启动雷达命令
    ScopedLocker l(_cmd_lock);
    if ((ret = sendCmd(SDK_CMD_STARTSCAN)) != RESULT_OK)
        return ret;
    //双通雷达才等待启动响应命令
    if (!m_SingleChannel)
    {
        ret = waitResp(SDK_CMD_STARTSCAN, timeout);
        if (ret != RESULT_OK)
        {
            printf("[YDLIDAR] Response to start scan error!\n");
            return ret;
        }
    }

    ret = createThread(); //创建线程
    return ret;
}

std::string DTSLidarDriver::getSDKVersion()
{
    return YDLIDAR_SDK_VERSION_STR;
}

const char *DTSLidarDriver::DescribeError(bool isTCP)
{
    if (_serial)
    {
        return _serial->DescribeError();
    }
    return nullptr;
}

bool DTSLidarDriver::isscanning() const
{
    return m_isScanning;
}

bool DTSLidarDriver::isconnected() const
{
    return m_isConnected;
}

void DTSLidarDriver::setAutoReconnect(const bool &enable)
{
    isAutoReconnect = enable;
}

result_t DTSLidarDriver::waitPackage(node_info *node, uint32_t timeout)
{
    int pos = 0;
    uint32_t st = getms();
    uint32_t wt = 0;
    uint16_t dataSize = 0; //data区数据的长度
    uint16_t cs = 0; //CRC
    result_t ret = RESULT_FAIL;
    vector<uint8_t> crCdata;
    memset(node, 0, SDKNODESIZE);
    while ((wt = getms() - st) < timeout)
    {
        size_t srcSize = SDKDTSHEADSIZE - pos;
        size_t dstSize = 0;

        //dstSize为实际获取接收到的数据大小
        result_t ans = waitForData(srcSize, timeout - wt, &dstSize);
        if (!IS_OK(ans))
            return ans;

        //从串口中读取指定大小的数据
        getData(recvBuff.data(), dstSize);

        for (size_t i = 0; i < dstSize; ++i)
        {
            uint8_t c = recvBuff[i];
            switch (pos)
            {
            case 0:
                if (c != SDK_CMD_HEADFLAG)
                {
                    pos = 0;
                    continue;
                }
                crCdata.clear();
                break;
            case 1:
                if (c != SDK_DTS_DEVNUM)
                {
                    pos = 0;
                    continue;
                }
                break;
            case 2:
                if (c != SDK_DTS_DEVTYPE)
                {
                    pos = 0;
                    continue;
                }
                break;
            case 3:
                if (c != SDK_CMD_STARTSCAN) //判断解析到的命令字是否和指定命令字是否一致
                {
                    pos = 0;
                    continue;
                }
                break;
            case 4:
                if (c != SDK_DTS_RESERVED) //判断解析到的命令字是否和指定命令字是否一致
                {
                    pos = 0;
                    continue;
                }
                break;
            case 5:
                dataSize = uint16_t(c) << 8; //取出data区数据的长度
                break;
            case 6:
                dataSize += uint16_t(c);
                break;
            default:
                break;
            }
            pos ++;
            crCdata.push_back(c);
        }

        //如果找到协议头
        if (pos == SDKDTSHEADSIZE)
        {
            pos = 0;
            //获取剩余数据，并计算校验码(data区数据长度 + CRC)
            size_t srcSize = dataSize + 2;
            size_t dstSize = 0;
            //dstSize为实际获取接收到的数据大小
            ret = waitForData(srcSize, timeout - wt, &dstSize);
            if (!IS_OK(ret))
                return ret;
            //从串口中读取指定大小的数据
            getData(recvBuff.data(), dstSize);
            //存储数据区的数据(用于计算CRC)
            for (size_t i = 0; i < dataSize; ++i)
            {
                crCdata.push_back(recvBuff[i]);
            }
            //计算校验码
            cs = calculateCrc(crCdata);
            //串口返回的CRC
            uint16_t csRaw = (recvBuff[dataSize] << 8) | recvBuff[dataSize+1];
            if (cs != csRaw)
            {
                printf("[YDLIDAR] CRC error calc[0x%04X] != src[0x%04X]\n",
                       cs, csRaw);
                fflush(stdout);
                return RESULT_FAIL;
            }
            break;
        }
    }

    if (IS_OK(ret))
    {
        (*node).sync = NODE_SYNC;
        (*node).stamp = getTime();
        (*node).index = 0;
        (*node).scanFreq = uint8_t(0);
        (*node).qual = 0;

        //取出主峰质心
        uint16_t mainPeakQuality = (recvBuff[7] << 8) | recvBuff[6];
        uint16_t qual = (recvBuff[11] << 8) | recvBuff[10];
        (*node).qual = qual;
        //主峰质心数据转十进制，再减去 b 值，然后再除以 k 值
        (*node).dist = mainPeakQuality;
        //这样是不是只有一个点
        (*node).angle = 0;
        //(*node).qual = 0;
        (*node).is = 0;
        return RESULT_OK;
    }
    else
    {
        return RESULT_FAIL;
    }
}

result_t DTSLidarDriver::sendCmd(uint8_t cmd, const uint8_t *data, size_t dataSize)
{
    if (!m_isConnected)
        return RESULT_FAIL;

    size_t size = SDKDTSHEADSIZE + dataSize;
    vector<uint8_t> buff(size + 2, 0);

    SdkDTSHead head;
    head.head = SDK_CMD_HEADFLAG;
    head.devNum = SDK_DTS_DEVNUM;
    head.devType = SDK_DTS_DEVTYPE;
    head.cmd = cmd;
    head.reserved  = SDK_DTS_RESERVED;
    head.size = uint16_t(dataSize);
    memcpy(&buff[0], &head, SDKDTSHEADSIZE);

    if (data && dataSize)
        memcpy(&buff[SDKDTSHEADSIZE], data, dataSize);

    //CRC校验
    uint16_t cs = calculateCrc(buff);
//    buff[0] = static_cast<uint8_t>(cs >> 8);  // 将高字节赋给下标为 0 的元素
//    buff[1] = static_cast<uint8_t>(cs);       // 将低字节赋给下标为 1 的元素
    buff[size] = static_cast<uint8_t>(cs >> 8);  //将高字节赋给下标为 0 的元素;
    buff[size+1] = static_cast<uint8_t>(cs);// 将低字节赋给下标为 1 的元素
    return sendData(buff.data(), buff.size());
}

result_t DTSLidarDriver::sendData(const uint8_t *data, size_t size)
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

result_t DTSLidarDriver::waitResp(
        uint8_t cmd, uint32_t timeout)
{
    std::vector<uint8_t> data;
    return waitResp(cmd, data, timeout);
}

/*
 * @brief DTSLidarDriver::waitResp
 * @param cmd      in:命令字
 * @param data     out:接收到的数据
 * @param timeout  in：超时时间
 * @return
 */
result_t DTSLidarDriver::waitResp(
        uint8_t cmd,
        std::vector<uint8_t> &data,
        uint32_t timeout)
{
    int pos = 0;
    uint32_t st = getms();
    uint32_t wt = 0;
    vector<uint8_t> recvBuff(SDK_DTS_BUFFLEN, 0);
    uint16_t cs = 0;
    uint8_t dataSize = 0;
    vector<uint8_t> crCdata;

    while ((wt = getms() - st) < timeout)
    {
        size_t srcSize = SDKDTSHEADSIZE - pos;
        size_t dstSize = 0;
        //dstSize为实际获取接收到的数据大小
        result_t ans = waitForData(srcSize, timeout - wt, &dstSize);
        if (!IS_OK(ans))
            return ans;
        //从串口中读取指定大小的数据
        getData(recvBuff.data(), dstSize);

        for (size_t i = 0; i < dstSize; ++i)
        {
            uint8_t c = recvBuff[i];
            switch (pos)
            {
            case 0:
                if (c != SDK_CMD_HEADFLAG)
                {
                    pos = 0;
                    continue;
                }
                crCdata.clear();
                break;
            case 1:
                if (c != SDK_DTS_DEVNUM)
                {
                    pos = 0;
                    continue;
                }
                break;
            case 2:
                if (c != SDK_DTS_DEVTYPE)
                {
                    pos = 0;
                    continue;
                }
                break;
            case 3:
                if (c != cmd) //判断解析到的命令字是否和指定命令字是否一致
                {
                    pos = 0;
                    continue;
                }
                break;
            case 4:
                if (c != SDK_DTS_RESERVED) //判断解析到的命令字是否和指定命令字是否一致
                {
                    pos = 0;
                    continue;
                }
                break;
            case 5:
                dataSize = uint16_t(c) << 8; //取出data区数据的长度
                break;
            case 6:
                dataSize += uint16_t(c);
                break;
            default:
                break;
            }

            pos ++;
            crCdata.push_back(c);
        }

        //如果找到协议头
        if (pos == SDKDTSHEADSIZE)
        {
            pos = 0;
            //获取剩余数据，并计算校验码(data区数据长度 + CRC)
            size_t srcSize = dataSize + 2;
            size_t dstSize = 0;
            //dstSize为实际获取接收到的数据大小
            result_t ans = waitForData(srcSize, timeout - wt, &dstSize);
            if (!IS_OK(ans))
                return ans;
            //从串口中读取指定大小的数据
            getData(recvBuff.data(), dstSize);

            for (size_t i = 0; i < dataSize; ++i)
            {
                crCdata.push_back(recvBuff[i]);
                data.push_back(recvBuff[i]); //数据存入输出参数
            }
            cs = calculateCrc(crCdata);
            //串口返回的CRC
            uint16_t csRaw = (recvBuff[dataSize] << 8) | recvBuff[dataSize+1];
            //判断CRC是否一致
            if (cs != csRaw)
            {
                printf("[YDLIDAR] CRC error calc[0x%04X] != src[0x%04X]\n",
                    cs, csRaw);
                return RESULT_FAIL;
            }
            return RESULT_OK;
        }
    }
    return RESULT_FAIL;
}

/*
 * 等待数据函数
 * @param srcSize  in:期望接收的数据大小
 * @param timeout  in:超时时间
 * @param dstSize  out:实际接收到的数据大小
 * @return 返回结果，表示等待数据的状态
 */
result_t DTSLidarDriver::waitForData(size_t srcSize, uint32_t timeout, size_t *dstSize)
{
    //用于存储实际接收到的数据大小
    size_t size = 0;

    //如果dstSize为空指针，则将其指向size变量
    if (!dstSize)
        dstSize = &size;

    //等待数据
    result_t ret = _serial->waitfordata(srcSize, timeout, dstSize);

    //如果实际接收到的数据大小大于期望大小，则将其截断为期望大小
    if (IS_OK(ret))
    {
        if (*dstSize > srcSize)
            *dstSize = srcSize;
    }
    return ret;
}

/*
 * @brief 从串口中读取指定大小的数据
 * @param data   out:存储从串口读取的数据
 * @param size   in:指定需要读取的数据大小
 * @return
 */
result_t DTSLidarDriver::getData(uint8_t *data, size_t size)
{
    //检查串口是否打开
    if (!_serial || !_serial->isOpen())
    {
        return RESULT_FAIL;
    }

    size_t r;

    //循环读取数据，直到读取完指定的大小
    while (size)
    {
        //从串口读取数据
        r = _serial->readData(data, size);

        //如果读取失败，则返回失败结果
        if (!r)
        {
            return RESULT_FAIL;
        }

        //如果开启了调试模式，则打印读取的数据
        if (m_Debug)
        {
            printf("recv: ");
            infoh(data, r);
        }

        //更新剩余的数据大小和数据指针
        size -= r;
        data += r;
    }

    return RESULT_OK;
}

//设置扫描频率(无)
result_t DTSLidarDriver::setScanFreq(float sf, uint32_t timeout)
{
    m_ScanFreq = 0;
    return RESULT_OK;
}

result_t DTSLidarDriver::getCalibParam(uint32_t timeout)
{
    ScopedLocker l(_cmd_lock);
    if (sendCmd(SDK_CMD_CALIBPARAM) != RESULT_OK)
    {
        printf("[YDLIDAR] Fail to send CalibParam cmd\n");
        return RESULT_FAIL;
    }
    vector<uint8_t> data;
    if (waitResp(SDK_CMD_CALIBPARAM, data, timeout) != RESULT_OK)
    {
        printf("[YDLIDAR] Fail to get CalibParam\n");
        return RESULT_FAIL;
    }

    //取出校准参数k,b的值
    memcpy(&k, &data[7], sizeof(float));
    memcpy(&b, &data[11], sizeof(float));

    printf("[YDLIDAR] CalibParam k[%f] b[%f]\n", k, b);

//    CalibParamInfo calibParamInfo;
//    memcpy(&calibParamInfo, data.data(), sizeof(CalibParamInfo));
//    k = calibParamInfo.k;
//    b = calibParamInfo.b;
    return RESULT_OK;
}

result_t DTSLidarDriver::checkAutoConnecting()
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
                    _serial = nullptr;
                }
            }
        }
        retryCount ++;
        if (retryCount > 10)
            retryCount = 10;

        int retryConnect = 0;
        while (isAutoReconnect &&
            connect(m_port.c_str(), m_baudrate) != RESULT_OK)
        {
            retryConnect ++;
            if (retryConnect > 5)
            {
                retryConnect = 5;
            }
            setDriverError(NotOpenError);

            delay(50 * retryConnect);
        }

        if (!isAutoReconnect)
        {
            m_isScanning = false;
            return RESULT_FAIL;
        }

        if (isconnected())
        {
            delay(10);
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

result_t DTSLidarDriver::startAutoScan(bool force, uint32_t timeout)
{
    result_t ans;

    if (!m_isConnected)
        return RESULT_FAIL;

    flushSerial();
    delay(10);
    {
        ScopedLocker l(_cmd_lock);
        if (sendCmd(SDK_CMD_STARTSCAN) != RESULT_OK)
        {
            return RESULT_FAIL;
        }
        if (!m_SingleChannel)
        {
            if ( waitResp(SDK_CMD_STARTSCAN, timeout) != RESULT_OK)
            {
                return RESULT_FAIL;
            }
        }
    }
    return RESULT_OK;
}

result_t DTSLidarDriver::getDeviceInfo(device_info &info, uint32_t timeout)
{
    if (!m_isConnected)
        return RESULT_FAIL;

    ScopedLocker l(_cmd_lock);
    if (sendCmd(SDK_CMD_CALIBPARAM) != RESULT_OK)
    {
        printf("[YDLIDAR] Fail to send CalibParam cmd\n");
        return RESULT_FAIL;
    }
    vector<uint8_t> data;
    if (waitResp(SDK_CMD_CALIBPARAM, data, timeout) != RESULT_OK)
    {
        printf("[YDLIDAR] Fail to get CalibParam\n");
        return RESULT_FAIL;
    }
    // infoh(data.data(), data.size());

    //取出校准参数k,b的值
    // memcpy(&k, &data[0], sizeof(float));
    // memcpy(&b, &data[4], sizeof(float));

    // printf("[YDLIDAR] CalibParam k[%f] b[%f]\n", k, b);

    memset(&info, 0, DEVICEINFOSIZE);
    info.model = YDLIDAR_SDM18;

    return RESULT_OK;
}

result_t DTSLidarDriver::getHealth(device_health &health, uint32_t timeout)
{
    health.status = 0;
    health.error_code = 0;
    UNUSED(timeout);
    return RESULT_OK;
}

const char *DTSLidarDriver::getErrorDesc(bool isTCP)
{
    UNUSED(isTCP);
    if (_serial)
    {
        return _serial->DescribeError();
    }
    return nullptr;
}

void DTSLidarDriver::disableDataGrabbing()
{
    if (m_isScanning)
    {
        m_isScanning = false;
        _dataEvent.set();
    }
    _thread.join();
}

void DTSLidarDriver::flushSerial()
{
    if (!m_isConnected)
        return;

    ScopedLocker l(_cmd_lock);
    size_t len = _serial->available();
    if (len)
    {
        _serial->readSize(len);
    }

    delay(20);
}

uint16_t DTSLidarDriver::calculateCrc(const vector<uint8_t> &data)
{
    uint16_t crc = 0xFFFF;  //初始值为0xFFFF

    for (const auto& byte : data)
    {
        crc ^= byte;
        for (int i = 0; i < 8; i++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

}

