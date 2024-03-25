#ifndef DTSLIDARDRIVER_H
#define DTSLIDARDRIVER_H
#include <stdlib.h>
#include <atomic>
#include <map>
#include "core/serial/serial.h"
#include "core/base/locker.h"
#include "core/base/thread.h"
#include "core/common/ydlidar_protocol.h"
#include "core/common/ydlidar_help.h"
#if !defined(__cplusplus)
#ifndef __cplusplus
#error "The YDLIDAR SDK requires a C++ compiler to be built"
#endif
#endif

#define SDK_DTS_POINT_COUNT 1
#define SDK_CMD_HEADFLAG 0xA5 //协议头标识1
#define SDK_CMD_STARTSCAN 0x01 //开启测距
#define SDK_CMD_STOPSCAN 0x02 //停止测距
#define SDK_CMD_CALIBPARAM 0x06 //获取校准参数
#define SDK_DTS_DEVNUM 0x03 //设备号
#define SDK_DTS_DEVTYPE 0x20 //设备类型
#define SDK_DTS_RESERVED 0x00 //保留位
#define SDK_DTS_BUFFLEN 100

//设置1字节对齐
#pragma pack(1)

//DTS雷达协议头
struct SdkDTSHead
{
    uint8_t head = 0; //包头
    uint8_t devNum = 0; //设备号
    uint8_t devType = 0; //设备类型
    uint8_t cmd = 0; //命令功能码
    uint8_t reserved = 0; //保留位
    uint16_t size = 0; //数据大小
};
#define SDKDTSHEADSIZE sizeof(SdkDTSHead)
//DTS雷达单点数据
struct SdkDTSPc
{

    uint16_t subPeakQuality = 0; //次峰质性
    uint16_t tempCode = 0; //温度码
    uint16_t subPeakIntensity = 0; //次峰强度
    uint16_t mainPeakQuality = 0; //主峰质性
    uint16_t mainPeakCalib = 0; //主峰校正
    uint16_t mainPeakIntensity = 0; //主峰强度
    uint16_t sunlitBase = 0; //阳光基底
};
//DTS雷达一包点云数据
struct SdkDTSPcs
{
    SdkDTSHead head;
    SdkDTSPc point; //一包数据只有单点
    uint16_t cs = 0;//校验码
};

#define SDKDTSPCSSIZE sizeof(SdkDTSPcs)
//取消设置1字节对齐
#pragma pack()

//校准参数结构体
struct CalibParamInfo
{
    float k = 0.0;
    float b = 0.0;
    char SN[9] = {0};
};

using namespace std;

namespace ydlidar
{
using namespace core;
using namespace core::serial;
using namespace core::base;

/*********************DTS雷达 ****************/
class DTSLidarDriver : public DriverInterface
{
public:
    DTSLidarDriver();
    virtual ~DTSLidarDriver();

    result_t connect(const char *port, uint32_t baudrate);
    void disconnect();
    result_t stopScan(uint32_t timeout = DEFAULT_TIMEOUT / 2);
    result_t stop();
    /*
     * @brief 获取激光数据
     * @param nodebuffer out: 激光点信息
     * @param count      in: 一圈激光点数
     * @param timeout    in: 超时时间
     * @return
     */
    result_t grabScanData(node_info *nodebuffer, size_t &count,
                          uint32_t timeout = DEFAULT_TIMEOUT);

    /*
     * @brief 等待扫描数据
     * @param nodes   out:存储节点信息的数组
     * @param count   out:节点信息数组的大小，传入时表示期望接收的节点数量，返回时表示实际接收的节点数量
     * @param timeout in:超时时间（毫秒）
     * @return result_t 操作结果，成功返回RESULT_OK，失败返回RESULT_FAIL
     */
    result_t waitScanData(node_info *nodes,
                          size_t &count,
                          uint32_t timeout = DEFAULT_TIMEOUT);

    //激光数据解析线程
    int cacheScanData();

    result_t createThread();

    result_t startScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT);

    virtual std::string getSDKVersion();

    virtual const char *DescribeError(bool isTCP = false);
    //雷达是否处于扫图状态
    bool isscanning() const;
    //雷达是否处于连接状态
    bool isconnected() const;

    /*
     * @brief 是否设置雷达异常自动重新连接
     * @param enable in:是否开启自动重连
     */
    void setAutoReconnect(const bool &enable);

    /*
     * @brief 解包激光数据 \n
     * @param[in] node 解包后激光点信息
     * @param[in] timeout     超时时间
     */
    result_t waitPackage(node_info *node, uint32_t timeout = DEFAULT_TIMEOUT);

    result_t sendCmd(uint8_t cmd,
                     const uint8_t *data = NULL,
                     size_t size = 0);
    //串口发送数据
    result_t sendData(const uint8_t *data, size_t size);

    //等待响应
    result_t waitResp(uint8_t cmd,
                     uint32_t timeout = DEFAULT_TIMEOUT);

    /*
     * @brief 等待数据(只获取响应data区的数据)
     * @param cmd      in:命令字
     * @param data     out:接收到的数据
     * @param timeout  in：超时时间
     * @return
     */
    result_t waitResp(uint8_t cmd,
                     std::vector<uint8_t> &data,
                     uint32_t timeout = DEFAULT_TIMEOUT);
    /*
     * 计算收到的数据的大小
     * @param srcSize  in:期望接收的数据大小
     * @param timeout  in:超时时间
     * @param dstSize  out:实际接收到的数据大小
     * @return 返回结果，表示等待数据的状态
     */
    result_t waitForData(size_t srcSize, uint32_t timeout = DEFAULT_TIMEOUT,
                         size_t *dstSize = NULL);
    /*
     * @brief 从串口中读取指定大小的数据
     * @param data   out:存储从串口读取的数据
     * @param size   in:指定需要读取的数据大小
     * @return
     */
    result_t getData(uint8_t *data, size_t size);

    //设置扫描频率(无)
    result_t setScanFreq(float sf, uint32_t timeout);
    //获取校准参数
    result_t getCalibParam(uint32_t timeout);
    //自动连接
    result_t checkAutoConnecting();
    //重新连接开启扫描
    result_t startAutoScan(bool force = false,
                           uint32_t timeout = DEFAULT_TIMEOUT);
    //获取设备信息
    virtual result_t getDeviceInfo(device_info &info,
                                   uint32_t timeout = DEFAULT_TIMEOUT);
    //获取健康状态
    virtual result_t getHealth(device_health &health,
                               uint32_t timeout = DEFAULT_TIMEOUT);
    //错误信息
    virtual const char *getErrorDesc(bool isTCP = false);
    //关闭数据获取通道
    void disableDataGrabbing();
    void flushSerial();
    //CRC校验码(CRC-16/MODBUS)
    uint16_t calculateCrc(const vector<uint8_t>& data);

private:
    serial::Serial *_serial = nullptr; //串口
    std::vector<uint8_t> recvBuff; //一包数据缓存
    float k = 0; //校准参数k
    float b = 0; //校准参数b
};
}
#endif // DTSLIDARDRIVER_H
