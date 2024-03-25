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

/** @page SDMLidarDriver
 * SDMLidarDriver API
    <table>
        <tr><th>Library     <td>SDMLidarDriver
        <tr><th>File        <td>SDMLidarDriver.h
        <tr><th>Author      <td>ZhanYi [code at ydlidar com]
        <tr><th>Source      <td>https://github.com/ydlidar/YDLidar-SDK
        <tr><th>Version     <td>1.0.0
    </table>
    This SDMLidarDriver support [TYPE_SDM](\ref LidarTypeID::TYPE_SDM) LiDAR

* @copyright    Copyright (c) @2015-2023 EAIBOT
     Jump to the @link ::ydlidar::SDMLidarDriver @endlink interface documentation.
*/
#ifndef SDM_YDLIDAR_DRIVER_H
#define SDM_YDLIDAR_DRIVER_H

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

#define SDK_SDM_POINT_COUNT 1
#define SDK_CMD_HEADFLAG0 0xAA //协议头标识1
#define SDK_CMD_HEADFLAG1 0x55 //协议头标识2
#define SDK_CMD_STARTSCAN 0x60 //开启测距
#define SDK_CMD_STOPSCAN 0x61 //停止测距
#define SDK_CMD_GETVERSION 0x62 //获取版本信息
#define SDK_CMD_SELFCHECK 0x63 //自检
#define SDK_CMD_SETFREQ 0x64 //设置输出频率
#define SDK_CMD_SETFILTER 0x65 //设置滤波
#define SDK_CMD_SETBAUDRATE 0x66 //设置串口波特率
#define SDK_CMD_SETOUTPUT 0x67 //设置输出的数据格式
#define SDK_CMD_RESET 0x68 //恢复出厂设置
#define SDK_BUFFER_MAXLEN 100 //缓存长度

//设置1字节对齐
#pragma pack(1)

//SDM雷达协议头
struct SdkSdmHead {
    uint8_t head0 = 0;
    uint8_t head1 = 0;
    uint8_t cmd = 0;
    uint8_t size = 0;
};
#define SDKSDMHEADSIZE sizeof(SdkSdmHead)
//SDM雷达单点数据
struct SdkSdmPc {
    uint16_t dist = 0; //距离
    uint8_t intensity = 0; //强度
    uint8_t env = 0; //环境干扰数据
};
//SDM雷达一包点云数据
struct SdkSdmPcs {
    SdkSdmHead head;
    SdkSdmPc point; //一包数据只有单点
    uint8_t cs = 0;
};
#define SDKSDMPCSSIZE sizeof(SdkSdmPcs)
//SDM雷达设备信息
struct SdkSdmDeviceInfo {
    uint8_t model = 0; //雷达型号码
    uint8_t hv = 0; //硬件版本
    uint8_t fvm = 0; //固件主版本
    uint8_t fvs = 0; //固件子版本
    uint8_t sn[SDK_SNLEN] = {0}; //序列号
};
#define SDKSDMDEVICEINFOSIZE sizeof(SdkSdmDeviceInfo)

#pragma pack()


using namespace std;

namespace ydlidar 
{

using namespace core;
using namespace core::serial;
using namespace core::base;

/*!
* SDM操控类
*/
class SDMLidarDriver : public DriverInterface 
{
public:
  //构造函数
  SDMLidarDriver();
  //析构函数
  virtual ~SDMLidarDriver();
  /*!
  * @brief 连接雷达 \n
  * 连接成功后，必须使用::disconnect函数关闭
  * @param[in] port 串口号
  * @param[in] baudrate 波特率，YDLIDAR-SDM 雷达波特率：961200
  * @return 返回连接状态
  * @retval 0     成功
  * @retval < 0   失败
  * @note连接成功后，必须使用::disconnect函数关闭
  * @see 函数::GSLidarDriver::disconnect (“::”是指定有连接功能,可以看文档里的disconnect变成绿,点击它可以跳转到disconnect.)
  */
  result_t connect(const char *port, uint32_t baudrate);

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
  * @brief 设置雷达异常自动重新连接 \n
  * @param[in] enable    是否开启自动重连:
  *     true	开启
  *	  false 关闭
  */
  void setAutoReconnect(const bool &enable);

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
                        uint32_t timeout = DEFAULT_TIMEOUT) ;


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
  * @brief 重置激光雷达（恢复出厂设置） \n
  * @param[in] timeout      超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       成功
  * @retval RESULT_FAILE    失败
  * @note 停止扫描后再执行当前操作, 如果在扫描中需先调用stop函数停止扫描
  */
  result_t reset(uint8_t addr, uint32_t timeout = DEFAULT_TIMEOUT);

  //设置开启或关闭滤波功能
  result_t enableFilter(bool yes=true);

  //设置扫描频率
  result_t setScanFreq(float sf, uint32_t timeout);

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
  result_t startAutoScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT) ;

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
  result_t sendCmd(uint8_t cmd,
                       const uint8_t *data = NULL,
                       size_t size = 0);

  /*!
  * @brief 等待响应数据 \n
  * @param[in] cmd 命令字
  * @param[out] data 响应数据
  * @param[in] timeout 超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       获取成功
  * @retval RESULT_TIMEOUT  等待超时
  * @retval RESULT_FAILE    获取失败
  * @note 当timeout = -1 时, 将一直等待
  */
  result_t waitResp(uint8_t cmd,
                   uint32_t timeout = DEFAULT_TIMEOUT);
  result_t waitResp(uint8_t cmd,
                   std::vector<uint8_t> &data,
                   uint32_t timeout = DEFAULT_TIMEOUT);
  /*!
  * @brief 等待激光数据包头 \n
  * @param[out] head 包头
  * @param[in] cmd 命令字
  * @param[out] data 响应数据
  * @param[in] timeout 超时时间
  * @return 返回执行结果
  * @retval RESULT_OK       获取成功
  * @retval RESULT_TIMEOUT  等待超时
  * @retval RESULT_FAILE    获取失败
  * @note 当timeout = -1 时, 将一直等待
  */
  // result_t waitResHeader(SdkSdmHead *head,
  //                        uint8_t cmd,
  //                        uint32_t timeout = DEFAULT_TIMEOUT);
  // result_t waitResHeader(SdkSdmHead *head,
  //                        uint8_t cmd,
  //                        std::vector<uint8_t> &data,
  //                        uint32_t timeout = DEFAULT_TIMEOUT);

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
  /**
   * @brief 错误信息
   * @param isTCP TCP or UDP
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
    /**
   * @brief get Device information \n
   * @param[in] info     Device information
   * @param[in] timeout  timeout
   * @return result status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE or RESULT_TIMEOUT   failed
   */
  virtual result_t getDeviceInfo(device_info &info, uint32_t timeout = DEFAULT_TIMEOUT);

private:
  serial::Serial *_serial = nullptr; //串口
  std::vector<uint8_t> recvBuff; //一包数据缓存
  device_health health_;
};

} // namespace ydlidar

#endif // SDM_YDLIDAR_DRIVER_H
