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

/** @page GS1LidarDriver
 * GS1LidarDriver API
    <table>
        <tr><th>Library     <td>GS1LidarDriver
        <tr><th>File        <td>GS1LidarDriver.h
        <tr><th>Author      <td>Tony [code at ydlidar com]
        <tr><th>Source      <td>https://github.com/ydlidar/YDLidar-SDK
        <tr><th>Version     <td>1.0.0
    </table>
    This GS1LidarDriver support [TYPE_TRIANGLE](\ref LidarTypeID::TYPE_TRIANGLE) and [TYPE_TOF](\ref LidarTypeID::TYPE_TOF) LiDAR

* @copyright    Copyright (c) 2018-2020  EAIBOT
     Jump to the @link ::ydlidar::GS1LidarDriver @endlink interface documentation.
*/
#ifndef GS1_YDLIDAR_DRIVER_H
#define GS1_YDLIDAR_DRIVER_H

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


using namespace std;

namespace ydlidar {

using namespace core;
using namespace core::serial;
using namespace core::base;

/*!
* GS1操控类
*/
class GS1LidarDriver : public DriverInterface {
 public:
  /**
    * @brief Set and Get LiDAR single channel.
    * Whether LiDAR communication channel is a single-channel
    * @note For a single-channel LiDAR, if the settings are reversed.\n
    * an error will occur in obtaining device information and the LiDAR will Faied to Start.\n
    * For dual-channel LiDAR, if th setttings are reversed.\n
    * the device information cannot be obtained.\n
    * Set the single channel to match the LiDAR.
    * @remarks
    <table>
         <tr><th>G1/G2/G2A/G2C                          <td>false
         <tr><th>G4/G4B/G4PRO/G6/F4/F4PRO               <td>false
         <tr><th>S4/S4B/X4/R2/G4C                       <td>false
         <tr><th>S2/X2/X2L                              <td>false
         <tr><th>TG15/TG30/TG50                         <td>false
         <tr><th>TX8/TX20                               <td>false
         <tr><th>T5/T15                                 <td>false
         <tr><th>GS1                                    <td>true
     </table>
    * @see DriverInterface::setSingleChannel and DriverInterface::getSingleChannel
    */
  PropertyBuilderByName(bool, SingleChannel, private);
  /**
  * @brief Set and Get LiDAR Type.
  * @note Refer to the table below for the LiDAR Type.\n
  * Set the LiDAR Type to match the LiDAR.
  * @remarks
  <table>
       <tr><th>G1/G2A/G2/G2C                    <td>[TYPE_TRIANGLE](\ref LidarTypeID::TYPE_TRIANGLE)
       <tr><th>G4/G4B/G4C/G4PRO                 <td>[TYPE_TRIANGLE](\ref LidarTypeID::TYPE_TRIANGLE)
       <tr><th>G6/F4/F4PRO                      <td>[TYPE_TRIANGLE](\ref LidarTypeID::TYPE_TRIANGLE)
       <tr><th>S4/S4B/X4/R2/S2/X2/X2L           <td>[TYPE_TRIANGLE](\ref LidarTypeID::TYPE_TRIANGLE)
       <tr><th>TG15/TG30/TG50/TX8/TX20          <td>[TYPE_TOF](\ref LidarTypeID::TYPE_TOF)
       <tr><th>T5/T15                           <td>[TYPE_TOF_NET](\ref LidarTypeID::TYPE_TOF_NET)
       <tr><th>GS1                              <td>[TYPE_GS](\ref LidarTypeID::TYPE_GS)
   </table>
  * @see [LidarTypeID](\ref LidarTypeID)
  * @see DriverInterface::setLidarType and DriverInterface::getLidarType
  */
  PropertyBuilderByName(int, LidarType, private);
  /**
  * @brief Set and Get Sampling interval.
  * @note Negative correlation between sampling interval and lidar sampling rate.\n
  * sampling interval = 1e9 / sampling rate(/s)\n
  * Set the LiDAR sampling interval to match the LiDAR.
  * @see DriverInterface::setPointTime and DriverInterface::getPointTime
  */
  PropertyBuilderByName(uint32_t, PointTime,private);
  /*!
  * A constructor.
  * A more elaborate description of the constructor.
  */
  GS1LidarDriver();

  /*!
  * A destructor.
  * A more elaborate description of the destructor.
  */
  virtual ~GS1LidarDriver();

  /*!
  * @brief 连接雷达 \n
  * 连接成功后，必须使用::disconnect函数关闭
  * @param[in] port_path 串口号
  * @param[in] baudrate 波特率，YDLIDAR-GS1 雷达波特率：961200
  * @return 返回连接状态
  * @retval 0     成功
  * @retval < 0   失败
  * @note连接成功后，必须使用::disconnect函数关闭
  * @see 函数::GS1LidarDriver::disconnect (“::”是指定有连接功能,可以看文档里的disconnect变成绿,点击它可以跳转到disconnect.)
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
  result_t getDevicePara(gs_device_para &info,   uint32_t timeout = DEFAULT_TIMEOUT);

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
  result_t waitResponseHeader(gs_lidar_ans_header *header,
                              uint32_t timeout = DEFAULT_TIMEOUT);
  result_t waitResponseHeaderEx(gs_lidar_ans_header *header,
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

  void addPointsToVec(node_info *nodebuffer, size_t &count);

    /**
   * @brief 串口错误信息
   * @param isTCP   TCP or UDP
   * @return error information
   */
  virtual const char *DescribeError(bool isTCP = false);

    /**
   * @brief GS1雷达没有健康信息\n
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

    /**
   * @brief 设置雷达工作模式（目前只针对GS1雷达）
   * @param[in] mode 雷达工作模式，0为避障模式；1为延边模式
   * @param[in] addr 雷达地址，第1个雷达地址为0x01；第2个雷达地址为0x02；第3个雷达地址为0x04；
   * @return 成功返回RESULT_OK，否则返回非RESULT_OK
   */
  virtual result_t setWorkMode(int mode=0, uint8_t addr=0x00);

  //未实现的虚函数
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
  enum {
    DEFAULT_TIMEOUT = 2000,    /**< 默认超时时间. */
    DEFAULT_HEART_BEAT = 1000, /**< 默认检测掉电功能时间. */
    MAX_SCAN_NODES = 3600,	   /**< 最大扫描点数. */
    DEFAULT_TIMEOUT_COUNT = 1,
  };

  node_info      *scan_node_buf;    ///< 激光点信息
  size_t         scan_node_count;   ///< 激光点数
  Event          _dataEvent;        ///< 数据同步事件
  Locker         _lock;				///< 线程锁
  Locker         _serial_lock;		///< 串口锁
  Thread 	     _thread;		   ///< 线程id

 private:
  int PackageSampleBytes;            ///< 一个包包含的激光点数
  serial::Serial *_serial;			///< 串口
  bool m_intensities;				///< 信号质量状体
  uint32_t m_baudrate;				///< 波特率
  bool isSupportMotorDtrCtrl;	    ///< 是否支持电机控制
  uint32_t trans_delay;				///< 串口传输一个byte时间
  int m_sampling_rate;              ///< 采样频率
  int model;                        ///< 雷达型号
  int sample_rate;                  ///<

  gs1_node_package package;             ///< 带信号质量协议包

  uint16_t package_Sample_Index;    ///< 包采样点索引
  float IntervalSampleAngle;
  float IntervalSampleAngle_LastPackage;
  uint8_t CheckSum;                ///< 校验和
  uint8_t scan_frequence;           ///< 协议中雷达转速

  uint8_t CheckSumCal;
  uint16_t SampleNumlAndCTCal;
  uint16_t LastSampleAngleCal;
  bool CheckSumResult;
  uint16_t Valu8Tou16;

  std::string serial_port;///< 雷达端口
  uint8_t *globalRecvBuffer;
  int retryCount;
  bool has_device_header;
  uint8_t last_device_byte;
  int         asyncRecvPos;
  uint16_t    async_size;

  //singleChannel
  device_info info_;
  device_health health_;
  gs_lidar_ans_header header_;
  uint8_t  *headerBuffer;
  uint8_t  *infoBuffer;
  uint8_t  *healthBuffer;
  bool     get_device_info_success;
  bool     get_device_health_success;

  int package_index;
  uint8_t package_type;
  bool has_package_error;

  double  d_compensateK0[PackageMaxModuleNums];
  double  d_compensateK1[PackageMaxModuleNums];
  double  d_compensateB0[PackageMaxModuleNums];
  double  d_compensateB1[PackageMaxModuleNums];
  uint16_t  u_compensateK0[PackageMaxModuleNums];
  uint16_t  u_compensateK1[PackageMaxModuleNums];
  uint16_t  u_compensateB0[PackageMaxModuleNums];    
  uint16_t  u_compensateB1[PackageMaxModuleNums];
  double  bias[PackageMaxModuleNums];
  bool isValidPoint;
  uint8_t  package_Sample_Num;

  uint8_t   frameNum;  //帧序号
  uint8_t   moduleNum;  //模块编号
  bool      isPrepareToSend; //是否准备好发送

  std::vector<GS1_Multi_Package>  multi_package;
};

}// namespace ydlidar

#endif // GS1_YDLIDAR_DRIVER_H
