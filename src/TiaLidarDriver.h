#pragma once
#include <vector>
#include <math.h>
#include "core/base/thread.h"
#include "core/base/locker.h"
#include "core/common/ydlidar_protocol.h"
#include "core/common/ydlidar_datatype.h"
#include "core/common/DriverInterface.h"
#include "core/json/cJSON.h"


//雷达参数
struct EaiLidarBaseParam
{
    int type = 0; //雷达类型
};
//TIA雷达参数
struct EaiTiaParam : public EaiLidarBaseParam
{
    std::string ip = "192.168.0.11"; //IP地址
    int port = 8090; //端口号
    std::string mask = "255.255.255.0"; //子网掩码
    std::string gateway = "192.168.0.1"; //网关
    std::string mac; //MAC地址

    float scanFreq = .0f; //扫描频率，单位Hz
    float sampleRate = .0f; //采样率，单位K/s
};
//TIA-X雷达参数
struct EaiTiaXParamItem //项
{
    float angle = 45.0f; //反射镜安装角度（单位°）
    float dist = 30.0f; //反射镜安装距离（单位mm）
    float opd = 0.0f; //光程差（单位mm）
    float minAngle = 60.0f; //反射镜起始角度（单位°）
    float maxAngle = 100.0f; //反射镜结束角度（单位°）

    //根据参数修正角度和距离
    bool correct(float &angle, float &dist) const {
        //距离无效直接返回
        if (int(dist) <= 0)
            return false;
        bool has = angle > SDK_ANGLE180; //超过180°的标记
        //先判断角度值是否有效
        if (angle >= minAngle &&
            angle <= maxAngle) //左反射镜
        {
            if (has)
                angle = SDK_ANGLE360 - angle;
            float beta = SDK_ANGLE180 - this->angle - angle;
            float a = this->dist / sin(beta * M_PI / SDK_ANGLE180); //边长a
            float b = dist - a; //边长b
            float C = (SDK_ANGLE180 - 2 * beta) * M_PI / SDK_ANGLE180; //角C（弧度）
            float c = sqrt(a * a + b * b - 2 * a * b * cos(C)); //边长c
            //余弦定理计算
            float B = acos((a * a + c * c - b * b) / (2 * a * c)) *
                    SDK_ANGLE180 / M_PI; //角B（角度）
            if (has)
                angle = SDK_ANGLE360 - (B + angle);
            else
                angle += B;

            dist = c + this->opd; //距离增加光程差值

            return true;
        }
        return false;
    }
};
struct EaiTiaXParam : public EaiTiaParam
{
    EaiTiaXParamItem left; //左反射镜参数
    EaiTiaXParamItem right; //右反射镜参数
};

namespace ydlidar {
namespace core {
namespace network {
class CActiveSocket;
class CPassiveSocket;
}
}

using namespace core::common;
using namespace core::base;
using namespace core::network;

class TiaLidarDriver : public DriverInterface
{
public:
    explicit TiaLidarDriver();
    ~TiaLidarDriver();

    //连接雷达
    virtual result_t connect(const char *port_path, uint32_t baudrate = 8000);
    //错误描述
    virtual const char *DescribeError(bool isTCP = true);
    //断连雷达
    virtual void disconnect();
    //是否在扫描状态
    virtual bool isscanning() const;
    //是否在连接状态
    virtual bool isconnected() const;

    /*!
  * @brief 设置雷达是否带信号质量 \n
  * 连接成功后，必须使用::disconnect函数关闭
  * @param[in] isintensities    是否带信号质量:
  *     true	带信号质量
  *	  false 无信号质量
  * @note只有S4B(波特率是153600)雷达支持带信号质量, 别的型号雷达暂不支持
  */
    virtual void setIntensities(const int &i);

    /*!
  * @brief 设置雷达异常自动重新连接 \n
  * @param[in] enable    是否开启自动重连:
  *     true	开启
  *	  false 关闭
  */
    virtual void setAutoReconnect(const bool &enable);
    //获取雷达设备健康状态
    virtual result_t getHealth(
            device_health &health,
            uint32_t timeout = SDK_TIMEOUT);
    //获取设备信息
    virtual result_t getDeviceInfo(
            device_info &info,
            uint32_t timeout = SDK_TIMEOUT);
    // //启动雷达
    // virtual result_t start(
    //         bool force = false,
    //         uint32_t timeout = SDK_TIMEOUT);
    // //停止雷达
    // virtual result_t stop();
    /*!
  * @brief 获取激光数据 \n
  * @param[in] nodebuffer 激光点信息
  * @param[in] count      一圈激光点数
  * @param[in] timeout    超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       获取成功
  * @retval RESULT_FAILE    获取失败
  * @note 获取之前，必须使用::startScan函数开启扫描
  */
    virtual result_t grabScanData(node_info *nodebuffer, size_t &count,
                                  uint32_t timeout = SDK_TIMEOUT) ;
    /*!
  * @brief 获取激光雷达当前扫描频率 \n
  * @param[in] frequency    扫描频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
    virtual result_t getScanFrequency(scan_frequency &frequency,
                                      uint32_t timeout = SDK_TIMEOUT);

    /*!
  * @brief 获取激光雷达当前采样频率 \n
  * @param[in] frequency    采样频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
    virtual result_t getSamplingRate(
            sampling_rate &rate,
            uint32_t timeout = SDK_TIMEOUT);

    /*!
  * @brief 设置激光雷达当前采样频率 \n
  * @param[in] rate    　　　采样频率
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作
  */
    virtual result_t setSamplingRate(
            sampling_rate &rate,
            uint32_t timeout = SDK_TIMEOUT);

private:
    //连接TCP
    bool configConnect(const char *lidarIP, int tcpPort = 8090);
    //断连TCP
    void configDisconnect();
    //连接UDP
    bool dataConnect(const char *lidarIP, int localPort = 8000);
    //断连UDP
    void dataDisconnect();
    //启动扫描
    virtual result_t startScan(
            bool force = false,
            uint32_t timeout = SDK_TIMEOUT);
    //停止扫描
    virtual result_t stopScan(
            uint32_t timeout = SDK_TIMEOUT);
    //创建线程
    bool createThread();
    //销毁线程
    void deleteThread();
    //启动扫描
    result_t startAutoScan(
            bool force = false,
            uint32_t timeout = SDK_TIMEOUT);
    //停止扫描
    result_t stopAutoScan(
            uint32_t timeout = SDK_TIMEOUT);

    result_t getScanData(node_info* nodes, size_t& count);
    //检查自动连接
    result_t checkAutoConnecting();

    //解析扫描数据线程函数
    int parseScanDataThread();
    //定时获取转速等信息线程函数
    int parseParamInfoThread();

    //设置多个参数
    bool setParams(const cJSON* json);
    //设置单个参数
    bool setParam(const std::string& key, const float& value);
    bool setParam(cJSON* json);
    //获取多个参数
    bool getParams(const std::string& key, cJSON* &value);
    //发送数据
    bool sendData(cJSON* json);
    //等待响应
    bool waitResp(cJSON* &json, uint32_t timeout = SDK_TIMEOUT);

private:
    //TIA-X修正角度和距离
    bool correct(float& a, float& d);

private:
    int m_model = YDLIDAR_TIA; //雷达型号
    float m_lastAngle = 0.f;
    int m_port2 = 9000;
    /* Sockets for ydlidar */
    CActiveSocket *socket_cmd = nullptr;
    CPassiveSocket *socket_data = nullptr;
    uint8_t m_buff[TIA_PACKMAXBUFFS2]; //缓存
    uint64_t lastZeroTime = 0; //上一零位点时间
    uint8_t lastPackIndex = 0; //上一包包序号
    //TIA-X专用参数
    EaiTiaXParam m_param; //参数
    Thread _thread2; //参数线程
    Locker _lock2; //操作串口或网络的锁（不支持嵌套）
};

} //ydlidar
