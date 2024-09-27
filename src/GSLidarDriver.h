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

/** @page GSLidarDriver
 * GSLidarDriver API
    <table>
        <tr><th>Library     <td>GSLidarDriver
        <tr><th>File        <td>GSLidarDriver.h
        <tr><th>Author      <td>Tony [code at ydlidar com]
        <tr><th>Source      <td>https://github.com/ydlidar/YDLidar-SDK
        <tr><th>Version     <td>1.0.0
    </table>
    This GSLidarDriver support [TYPE_TRIANGLE](\ref LidarTypeID::TYPE_TRIANGLE) and [TYPE_TOF](\ref LidarTypeID::TYPE_TOF) LiDAR

* @copyright    Copyright (c) 2018-2020  EAIBOT
     Jump to the @link ::ydlidar::GSLidarDriver @endlink interface documentation.
*/
#ifndef GS2_YDLIDAR_DRIVER_H
#define GS2_YDLIDAR_DRIVER_H

#include <stdlib.h>
#include <atomic>
#include <map>
#include <list>
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

using namespace std;

namespace ydlidar
{

  using namespace core;
  using namespace core::serial;
  using namespace core::base;

  /*!
   * GS雷达操控类
   */
  class GSLidarDriver : public DriverInterface
  {
  public:
    /*!
     * A constructor.
     * A more elaborate description of the constructor.
     */
    GSLidarDriver(uint8_t type = YDLIDAR_TYPE_SERIAL);
    /*!
     * A destructor.
     * A more elaborate description of the destructor.
     */
    virtual ~GSLidarDriver();

    /*!
     * @brief 连接雷达 \n
     * 连接成功后，必须使用::disconnect函数关闭
     * @param[in] port_path 串口号
     * @param[in] baudrate 波特率，YDLIDAR-GS2 雷达波特率：961200
     * @return 返回连接状态
     * @retval 0     成功
     * @retval < 0   失败
     * @note连接成功后，必须使用::disconnect函数关闭
     * @see 函数::GSLidarDriver::disconnect (“::”是指定有连接功能,可以看文档里的disconnect变成绿,点击它可以跳转到disconnect.)
     */
    result_t connect(const char *port_path, uint32_t baudrate);

    /*!
     * @brief 断开雷达连接
     */
    void disconnect();

    /*!
     * @brief 获取当前SDK版本号 \n
     * 静态函数
     * @return 返回当前SKD 版本号
     */
    virtual std::string getSDKVersion();

    /*!
     * @brief lidarPortList 获取雷达端口
     * @return 在线雷达列表
     */
    static std::map<std::string, std::string> lidarPortList();

    /*!
     * @brief 扫图状态 \n
     * @return 返回当前雷达扫图状态
     * @retval true     正在扫图
     * @retval false    扫图关闭
     */
    bool isscanning() const;

    /*!
     * @brief 连接雷达状态 \n
     * @return 返回连接状态
     * @retval true     成功
     * @retval false    失败
     */
    bool isconnected() const;

    /*!
     * @brief 设置雷达是否带信号质量 \n
     * 连接成功后，必须使用::disconnect函数关闭
     * @param[in] isintensities    是否带信号质量:
     *     true	带信号质量
     *	  false 无信号质量
     * @note只有S4B(波特率是153600)雷达支持带信号质量, 别的型号雷达暂不支持
     */
    void setIntensities(const bool &isintensities);

    /*!
     * @brief 设置雷达异常自动重新连接 \n
     * @param[in] enable    是否开启自动重连:
     *     true	开启
     *	  false 关闭
     */
    void setAutoReconnect(const bool &enable);

    /*!
     * @brief 获取雷达设备信息 \n
     * @param[in] parameters     设备信息
     * @param[in] timeout  超时时间
     * @return 返回执行结果
     * @retval RESULT_OK       获取成功
     * @retval RESULT_FAILE or RESULT_TIMEOUT   获取失败
     */
    result_t getDevicePara(gs_device_para &info, uint32_t timeout = DEFAULT_TIMEOUT);

    /*!
     * @brief 配置雷达地址 \n
     * @param[in] timeout  超时时间
     * @return 返回执行结果
     * @retval RESULT_OK       配置成功
     * @retval RESULT_FAILE or RESULT_TIMEOUT   配置超时
     */
    result_t setDeviceAddress(uint32_t timeout = DEFAULT_TIMEOUT);

    /*!
     * @brief 开启扫描 \n
     * @param[in] force    扫描模式
     * @param[in] timeout  超时时间
     * @return 返回执行结果
     * @retval RESULT_OK       开启成功
     * @retval RESULT_FAILE    开启失败
     * @note 只用开启一次成功即可
     */
    result_t startScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT);

    /*!
     * @brief 关闭扫描 \n
     * @return 返回执行结果
     * @retval RESULT_OK       关闭成功
     * @retval RESULT_FAILE    关闭失败
     */
    result_t stop();

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
    result_t grabScanData(node_info *nodebuffer, size_t &count,
                          uint32_t timeout = DEFAULT_TIMEOUT);

    /*!
     * @brief 补偿激光角度 \n
     * 把角度限制在0到360度之间
     * @param[in] nodebuffer 激光点信息
     * @param[in] count      一圈激光点数
     * @return 返回执行结果
     * @retval RESULT_OK       成功
     * @retval RESULT_FAILE    失败
     * @note 补偿之前，必须使用::grabScanData函数获取激光数据成功
     */
    result_t ascendScanData(node_info *nodebuffer, size_t count);

    /*!
     * @brief 重置激光雷达 \n
     * @param[in] timeout      超时时间
     * @return 返回执行结果
     * @retval RESULT_OK       成功
     * @retval RESULT_FAILE    失败
     * @note 停止扫描后再执行当前操作, 如果在扫描中调用::stop函数停止扫描
     */
    result_t reset(uint8_t addr, uint32_t timeout = DEFAULT_TIMEOUT);

  protected:
    /*!
     * @brief 创建解析雷达数据线程 \n
     * @note 创建解析雷达数据线程之前，必须使用::startScan函数开启扫图成功
     */
    result_t createThread();

    /*!
     * @brief 重新连接开启扫描 \n
     * @param[in] force    扫描模式
     * @param[in] timeout  超时时间
     * @return 返回执行结果
     * @retval RESULT_OK       开启成功
     * @retval RESULT_FAILE    开启失败
     * @note sdk 自动重新连接调用
     */
    result_t startAutoScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT);

    /*!
     * @brief stopScan
     * @param timeout
     * @return
     */
    result_t stopScan(uint32_t timeout = DEFAULT_TIMEOUT);

    /*!
     * @brief waitDevicePackage
     * @param timeout
     * @return
     */
    result_t waitDevicePackage(uint32_t timeout = DEFAULT_TIMEOUT);
    /*!
     * @brief 解包激光数据 \n
     * @param[in] node 解包后激光点信息
     * @param[in] timeout     超时时间
     */
    result_t waitPackage(node_info *node, uint32_t timeout = DEFAULT_TIMEOUT);

    /*!
     * @brief 发送数据到雷达 \n
     * @param[in] nodebuffer 激光信息指针
     * @param[in] count      激光点数大小
     * @param[in] timeout      超时时间
     * @return 返回执行结果
     * @retval RESULT_OK       成功
     * @retval RESULT_TIMEOUT  等待超时
     * @retval RESULT_FAILE    失败
     */
    result_t waitScanData(node_info *nodebuffer, size_t &count,
                          uint32_t timeout = DEFAULT_TIMEOUT);

    /*!
     * @brief 激光数据解析线程 \n
     */
    int cacheScanData();

    /*!
     * @brief 发送数据到雷达 \n
     * @param[in] cmd 	 命名码
     * @param[in] payload      payload
     * @param[in] payloadsize      payloadsize
     * @return 返回执行结果
     * @retval RESULT_OK       成功
     * @retval RESULT_FAILE    失败
     */
    result_t sendCommand(uint8_t cmd,
                         const void *payload = NULL,
                         size_t payloadsize = 0);

    /*!
     * @brief 发送数据到雷达 \n
     * @param[in] addr 模组地址
     * @param[in] cmd 	 命名码
     * @param[in] payload      payload
     * @param[in] payloadsize      payloadsize
     * @return 返回执行结果
     * @retval RESULT_OK       成功
     * @retval RESULT_FAILE    失败
     */
    result_t sendCommand(uint8_t addr,
                         uint8_t cmd,
                         const void *payload = NULL,
                         size_t payloadsize = 0);

    /*!
     * @brief 等待激光数据包头 \n
     * @param[in] header 	 包头
     * @param[in] timeout      超时时间
     * @return 返回执行结果
     * @retval RESULT_OK       获取成功
     * @retval RESULT_TIMEOUT  等待超时
     * @retval RESULT_FAILE    获取失败
     * @note 当timeout = -1 时, 将一直等待
     */
    result_t waitResponseHeader(gs_package_head *header,
                                uint32_t timeout = DEFAULT_TIMEOUT);
    result_t waitResponseHeaderEx(gs_package_head *header,
                                  uint8_t cmd,
                                  uint32_t timeout = DEFAULT_TIMEOUT);

    /*!
     * @brief 等待固定数量串口数据 \n
     * @param[in] data_count 	 等待数据大小
     * @param[in] timeout    	 等待时间
     * @param[in] returned_size   实际数据大小
     * @return 返回执行结果
     * @retval RESULT_OK       获取成功
     * @retval RESULT_TIMEOUT  等待超时
     * @retval RESULT_FAILE    获取失败
     * @note 当timeout = -1 时, 将一直等待
     */
    result_t waitForData(size_t data_count, uint32_t timeout = DEFAULT_TIMEOUT,
                         size_t *returned_size = NULL);

    /*!
     * @brief 获取串口数据 \n
     * @param[in] data 	 数据指针
     * @param[in] size    数据大小
     * @return 返回执行结果
     * @retval RESULT_OK       获取成功
     * @retval RESULT_FAILE    获取失败
     */
    result_t getData(uint8_t *data, size_t size);

    /*!
     * @brief 串口发送数据 \n
     * @param[in] data 	 发送数据指针
     * @param[in] size    数据大小
     * @return 返回执行结果
     * @retval RESULT_OK       发送成功
     * @retval RESULT_FAILE    发送失败
     */
    result_t sendData(const uint8_t *data, size_t size);

    /*!
     * @brief checkTransDelay
     */
    void checkTransDelay();

    /*!
     * @brief 关闭数据获取通道 \n
     */
    void disableDataGrabbing();

    /*!
     * @brief 设置串口DTR \n
     */
    void setDTR();

    /*!
     * @brief 清除串口DTR \n
     */
    void clearDTR();

    /*!
     * @brief flushSerial
     */
    void flushSerial();

    /*!
     * @brief checkAutoConnecting
     */
    result_t checkAutoConnecting();

    /*!
     * @brief  换算得出点的距离和角度
     */
    void angTransform(uint16_t dist, int n, double *dstTheta, uint16_t *dstDist);
    void angTransform2(uint16_t dist, int n, double *dstTheta, uint16_t *dstDist);

    /**
     * @brief 串口错误信息
     * @param isTCP   TCP or UDP
     * @return error information
     */
    virtual const char *DescribeError(bool isTCP = false);

    /**
     * @brief GS2雷达没有健康信息\n
     * @return result status
     * @retval RESULT_OK success
     * @retval RESULT_FAILE or RESULT_TIMEOUT failed
     */
    virtual result_t getHealth(device_health &health, uint32_t timeout = DEFAULT_TIMEOUT);

    //获取设备信息
    virtual result_t getDeviceInfo(
      device_info &di, 
      uint32_t timeout = DEFAULT_TIMEOUT/4);
    //获取级联雷达设备信息
    virtual result_t getDeviceInfo(
      std::vector<device_info_ex> &dis,
      uint32_t timeout = DEFAULT_TIMEOUT);
    virtual result_t getDeviceInfo1(
      device_info &di, 
      uint32_t timeout = DEFAULT_TIMEOUT);
    virtual result_t getDeviceInfo2(
      device_info &di, 
      uint32_t timeout = DEFAULT_TIMEOUT);

    /**
     * @brief 设置雷达工作模式（目前只针对GS2雷达）
     * @param[in] mode 雷达工作模式，0为避障模式；1为延边模式
     * @param[in] addr 雷达地址，第1个雷达地址为0x01；第2个雷达地址为0x02；第3个雷达地址为0x04；
     * @return 成功返回RESULT_OK，否则返回非RESULT_OK
     */
    virtual result_t setWorkMode(int mode = 0, uint8_t addr = 0x00);

    // 开始OTA升级
    virtual bool ota();
    // 开始OTA
    bool startOta(uint8_t addr);
    // OTA升级中
    bool execOta(uint8_t addr, const std::vector<uint8_t>& data);
    // 停止OTA
    bool stopOta(uint8_t addr);
    //判断响应是否正常
    bool isOtaRespOk(uint8_t addr,
                     uint8_t cmd,
                     uint16_t offset,
                     const std::vector<uint8_t>& data);
    bool sendData(uint8_t addr,
                  uint8_t cmd,
                  const std::vector<uint8_t> &data,
                  uint8_t cmdRecv,
                  std::vector<uint8_t> &dataRecv,
                  int timeout = 500);

    // 未实现的虚函数
    virtual result_t getScanFrequency(scan_frequency &frequency, uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t setScanFrequencyDis(scan_frequency &frequency,
                                         uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t setScanFrequencyAdd(scan_frequency &frequency,
                                         uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t setScanFrequencyAddMic(scan_frequency &frequency,
                                            uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t setScanFrequencyDisMic(scan_frequency &frequency,
                                            uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t getSamplingRate(sampling_rate &rate,
                                     uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t setSamplingRate(sampling_rate &rate,
                                     uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t getZeroOffsetAngle(offset_angle &angle,
                                        uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t setScanHeartbeat(scan_heart_beat &beat,
                                      uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }

  public:
    enum
    {
      DEFAULT_TIMEOUT = 2000,    /**< 默认超时时间. */
      DEFAULT_HEART_BEAT = 1000, /**< 默认检测掉电功能时间. */
      MAX_SCAN_NODES = 160 * 3,     /**< 最大扫描点数. */
      DEFAULT_TIMEOUT_COUNT = 3, // 错误数
    };

  private:
    int PackageSampleBytes; //一个包包含的激光点数
    ChannelDevice *_comm = nullptr; //通讯对象
    uint32_t trans_delay; //串口传输一个byte时间
    int sample_rate; //采样频率

    gs_node_package package; //带信号质量协议包

    uint8_t CheckSum; //校验和
    uint8_t CheckSumCal;
    bool CheckSumResult;

    uint8_t *globalRecvBuffer = nullptr;

    double k0[LIDAR_MAXCOUNT];
    double k1[LIDAR_MAXCOUNT];
    double b0[LIDAR_MAXCOUNT];
    double b1[LIDAR_MAXCOUNT];
    double bias[LIDAR_MAXCOUNT];
    int m_models[LIDAR_MAXCOUNT] = {0};
    int model = YDLIDAR_GS2; //雷达型号
    uint8_t moduleNum = 0; // 模块编号
    uint8_t moduleCount = 1; // 当前模组数量
    int nodeCount = 0; //当前包点数
    uint64_t stamp = 0; //时间戳
    std::list<gs_module_nodes> datas; //各模组数据
    double m_pitchAngle = Angle_PAngle;
    uint32_t lastStamp = 0; //上一次时间
  };

} // namespace ydlidar

#endif // GS2_YDLIDAR_DRIVER_H
