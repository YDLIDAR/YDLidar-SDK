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
/** @page YDlidarDriver
 * YDlidarDriver API
    <table>
        <tr><th>Library     <td>YDlidarDriver
        <tr><th>File        <td>ydlidar_driver.h
        <tr><th>Author      <td>Tony [code at ydlidar com]
        <tr><th>Source      <td>https://github.com/ydlidar/YDLidar-SDK
        <tr><th>Version     <td>1.0.0
    </table>
    This YDlidarDriver support [TYPE_TRIANGLE](\ref LidarTypeID::TYPE_TRIANGLE) and [TYPE_TOF](\ref LidarTypeID::TYPE_TOF) LiDAR

* @copyright    Copyright (c) 2018-2020  EAIBOT
     Jump to the @link ::ydlidar::YDlidarDriver @endlink interface documentation.
*/

#ifndef YDLIDAR_DRIVER_H
#define YDLIDAR_DRIVER_H
#include <stdlib.h>
#include <atomic>
#include <map>
#include <core/common/ChannelDevice.h>
#include <core/base/locker.h>
#include <core/base/thread.h>
#include <core/common/ydlidar_protocol.h>
#include <core/common/DriverInterface.h>
#include <core/common/ydlidar_help.h>

#if !defined(__cplusplus)
#ifndef __cplusplus
#error "The YDLIDAR SDK requires a C++ compiler to be built"
#endif
#endif


using namespace std;

namespace ydlidar {
using namespace core;
using namespace core::common;

/*!
* Class that provides a lidar interface.
*/
class YDlidarDriver : public DriverInterface {
 public:
  /*!
  * A constructor.
  * A more elaborate description of the constructor.
  */
  explicit YDlidarDriver(uint8_t type = YDLIDAR_TYPE_SERIAL);

  /*!
  * A destructor.
  * A more elaborate description of the destructor.
  */
  virtual ~YDlidarDriver();

  /**
   * @brief Connecting Lidar \n
   * After the connection if successful, you must use ::disconnect to close
   * @param[in] port_path    serial port
   * @param[in] baudrate    serial baudrate，YDLIDAR-SS：
   *     230400 G2-SS-1 R2-SS-1
   * @return connection status
   * @retval 0     success
   * @retval < 0   failed
   * @note After the connection if successful, you must use ::disconnect to close
   * @see function ::YDlidarDriver::disconnect ()
   */
  virtual result_t connect(const char *port_path, uint32_t baudrate);

  /**
   * @brief Returns a human-readable description of the given error code
   *  or the last error code of a socket or serial port
   * @param isTCP   TCP or UDP
   * @return error information
   */
  virtual const char *DescribeError(bool isTCP = true);


  /*!
  * @brief Disconnect the LiDAR.
  */
  virtual void disconnect();

  /**
  * @brief Get SDK Version \n
  * static function
  * @return Version
  */
  virtual std::string getSDKVersion();

  /**
   * @brief lidarPortList Get Lidar Port lists
   * @return online lidars
   */
  static std::map<std::string, std::string> lidarPortList();


  /**
   * @brief Is the Lidar in the scan \n
   * @return scanning status
   * @retval true     scanning
   * @retval false    non-scanning
   */
  virtual bool isscanning() const;

  /**
   * @brief Is it connected to the lidar \n
   * @return connection status
   * @retval true     connected
   * @retval false    Non-connected
   */
  virtual bool isconnected() const;

  /**
   * @brief Is there intensity \n
   * @param[in] isintensities    intentsity
   *   true	intensity
   *   false no intensity
   */
  virtual void setIntensities(const bool &isintensities);

  /**
   * @brief whether to support hot plug \n
   * @param[in] enable    hot plug :
   *   true	support
   *   false no support
   */
  virtual void setAutoReconnect(const bool &enable);

  /**
   * @brief get Health status \n
   * @return result status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE or RESULT_TIMEOUT   failed
   */
  virtual result_t getHealth(device_health &health,
                             uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief get Device information \n
   * @param[in] info     Device information
   * @param[in] timeout  timeout
   * @return result status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE or RESULT_TIMEOUT   failed
   */
  virtual result_t getDeviceInfo(device_info &info,
                                 uint32_t timeout = DEFAULT_TIMEOUT);
  
  //获取设备信息
  virtual bool getDeviceInfoEx(device_info &info, int type=EPT_Module);

  /**
   * @brief Turn on scanning \n
   * @param[in] force    Scan mode
   * @param[in] timeout  timeout
   * @return result status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   * @note Just turn it on once
   */
  virtual result_t startScan(bool force = false,
                             uint32_t timeout = DEFAULT_TIMEOUT) ;

  /**
   * @brief turn off scanning \n
   * @return result status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   */
  virtual result_t stop();

  /**
   * @brief Get a circle of laser data \n
   * @param[in] nodebuffer Laser data
   * @param[in] count      one circle of laser points
   * @param[in] timeout    timeout
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   * @note Before starting, you must start the start the scan successfully with the ::startScan function
   */
  virtual result_t grabScanData(node_info *nodebuffer, size_t &count,
                                uint32_t timeout = DEFAULT_TIMEOUT) ;

  /**
   * @brief Normalized angle \n
   * Normalize the angel between 0 and 360
   * @param[in] nodebuffer Laser data
   * @param[in] count      one circle of laser points
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   * @note Before the normalization, you must use the ::grabScanData function to get the laser data successfully.
   */
  result_t ascendScanData(node_info *nodebuffer, size_t count);

  /**
  * @brief reset lidar \n
  * @param[in] timeout      timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  * @note Non-scan state, perform currect operation.
  */
  result_t reset(uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief start motor \n
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   */
  result_t startMotor();

  /**
   * @brief stop motor \n
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   */
  result_t stopMotor();

  /**
  * @brief Get lidar scan frequency \n
  * @param[in] frequency    scanning frequency
  * @param[in] timeout      timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  * @note Non-scan state, perform currect operation.
  */
  virtual  result_t getScanFrequency(scan_frequency &frequency,
                                     uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief Increase the scanning frequency by 1.0 HZ \n
   * @param[in] frequency    scanning frequency
   * @param[in] timeout      timeout
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   * @note Non-scan state, perform currect operation.
   */
  virtual result_t setScanFrequencyAdd(scan_frequency &frequency,
                                       uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief Reduce the scanning frequency by 1.0 HZ \n
   * @param[in] frequency    scanning frequency
   * @param[in] timeout      timeout
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   * @note Non-scan state, perform currect operation.
   */
  virtual result_t setScanFrequencyDis(scan_frequency &frequency,
                                       uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief Increase the scanning frequency by 0.1 HZ \n
   * @param[in] frequency    scanning frequency
   * @param[in] timeout      timeout
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   * @note Non-scan state, perform currect operation.
   */
  virtual result_t setScanFrequencyAddMic(scan_frequency &frequency,
                                          uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief Reduce the scanning frequency by 0.1 HZ \n
   * @param[in] frequency    scanning frequency
   * @param[in] timeout      timeout
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   * @note Non-scan state, perform currect operation.
   */
  virtual result_t setScanFrequencyDisMic(scan_frequency &frequency,
                                          uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief Get lidar sampling frequency \n
   * @param[in] frequency    sampling frequency
   * @param[in] timeout      timeout
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   * @note Non-scan state, perform currect operation.
   */
  virtual result_t getSamplingRate(sampling_rate &rate,
                                   uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief Set the lidar sampling frequency \n
   * @param[in] rate    　　　sampling frequency
   * @param[in] timeout      timeout
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   * @note Non-scan state, perform currect operation.
   */
  virtual result_t setSamplingRate(sampling_rate &rate,
                                   uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief get lidar zero offset angle \n
   * @param[in] angle　　　   zero offset angle
   * @param[in] timeout      timeout
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   * @note Non-scan state, perform currect operation.
   */
  virtual result_t getZeroOffsetAngle(offset_angle &angle,
                                      uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief set lidar heart beat \n
   * @param[in] beat    	  heart beat status
   * @param[in] timeout      timeout
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   * @note Non-scan state, perform currect operation.
   */
  virtual result_t setScanHeartbeat(scan_heart_beat &beat,
                                    uint32_t timeout = DEFAULT_TIMEOUT);

 protected:

  /**
   * @brief get lidar zero offset angle \n
   * @param[in] angle　　　   zero offset angle
   * @param[in] timeout      timeout
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   * @note scanning state,perform currect operation.
   */
  result_t getAutoZeroOffsetAngle(offset_angle &angle,
                                  uint32_t timeout = DEFAULT_TIMEOUT);
  /**
  * @brief Data parsing thread \n
  * @note Before you create a dta parsing thread, you must use the ::startScan function to start the lidar scan successfully.
  */
  result_t createThread();

  /**
  * @brief Automatically reconnect the lidar \n
  * @param[in] force    scan model
  * @param[in] timeout  timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  * @note Lidar abnormality automatically reconnects.
  */
  result_t startAutoScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT) ;

  /*!
  * @brief stop Scanning state
  * @param timeout  timeout
  * @return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  */
  result_t stopScan(uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief waiting device information
   * @param timeout timeout
   * @return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   */
  result_t waitDevicePackage(uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief parseResponseHeader
   * @param packageBuffer
   * @param timeout
   * @return
   */
  result_t parseResponseHeader(uint8_t *packageBuffer,
                               uint32_t timeout = DEFAULT_TIMEOUT);

  /**
   * @brief parseResponseScanData
   * @param packageBuffer
   * @param timeout
   * @return
   */
  result_t parseResponseScanData(uint8_t *packageBuffer,
                                 uint32_t timeout = DEFAULT_TIMEOUT);

  //解析时间戳数据（云鲸雷达）
  bool parseStampData(uint32_t timeout = DEFAULT_TIMEOUT / 10);

  /**
  * @brief Unpacking \n
  * @param[in] node lidar point information
  * @param[in] timeout     timeout
  */
  result_t waitPackage(node_info *node, uint32_t timeout = DEFAULT_TIMEOUT);
  /**
  * @brief get unpacked data \n
  * @param[in] nodebuffer laser node
  * @param[in] count      lidar points size
  * @param[in] timeout      timeout
  * @return result status
  * @retval RESULT_OK       success
  * @retval RESULT_TIMEOUT  timeout
  * @retval RESULT_FAILE    failed
  */
  result_t waitScanData(node_info *nodebuffer, size_t &count,
                        uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief data parsing thread \n
  */
  int cacheScanData();

  /**
  * @brief send data to lidar \n
  * @param[in] cmd 	 command code
  * @param[in] payload      payload
  * @param[in] payloadsize      payloadsize
  * @return result status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  */
  result_t sendCommand(uint8_t cmd, const void *payload = NULL,
                       size_t payloadsize = 0);

  /**
  * @brief waiting for package header \n
  * @param[in] header 	 package header
  * @param[in] timeout      timeout
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_TIMEOUT  timeout
  * @retval RESULT_FAILE    failed
  * @note when timeout = -1, it will block...
  */
  result_t waitResponseHeader(lidar_ans_header *header,
                              uint32_t timeout = DEFAULT_TIMEOUT);

  /**
  * @brief Waiting for the specified size data from the lidar \n
  * @param[in] data_count 	 wait max data size
  * @param[in] timeout    	 timeout
  * @param[in] returned_size   really data size
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_TIMEOUT  wait timeout
  * @retval RESULT_FAILE    failed
  * @note when timeout = -1, it will block...
  */
  result_t waitForData(size_t data_count, uint32_t timeout = DEFAULT_TIMEOUT,
                       size_t *returned_size = NULL);

  /**
  * @brief get data from serial \n
  * @param[in] data 	 data
  * @param[in] size    date size
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  */
  result_t getData(uint8_t *data, size_t size);

  /**
  * @brief send data to serial \n
  * @param[in] data 	 data
  * @param[in] size    data size
  * @return return status
  * @retval RESULT_OK       success
  * @retval RESULT_FAILE    failed
  */
  result_t sendData(const uint8_t *data, size_t size);


  /*!
  * @brief checkTransDelay
  */
  void checkTransDelay();

  /**
  * @brief disable Data scan channel \n
  */
  void disableDataGrabbing();

  /*!
  * @brief set DTR \n
  */
  void setDTR();

  /*!
  * @brief clear DTR \n
  */
  void clearDTR();

  /*!
   * @brief flushSerial
   */
  void flushSerial();

  /*!
   * @brief checkAutoConnecting
   */
  result_t checkAutoConnecting(bool serialError = true);

  /**
   * @brief autoHeartBeat
   * @return
   */
  result_t autoHeartBeat();

  /**
   * @brief KeepLiveHeartBeat
   */
  void KeepLiveHeartBeat();

  /**
   * @brief CheckLaserStatus
   */
  void CheckLaserStatus();

  /**
   * @brief checkBlockStatus
   */
  void checkBlockStatus(uint8_t currentByte);

  /**
   * @brief calcuteCheckSum
   * @param node
   */
  void calcuteCheckSum(node_info *node);
  /**
   * @brief calcutePackageCT
   */
  void calcutePackageCT();
  /**
   * @brief parseNodeDebugFromBuffer
   */
  void parseNodeDebugFromBuffer(node_info *node);

  /**
   * @brief parseNodeFromeBuffer
   */
  void parseNodeFromeBuffer(node_info *node);

  //解析点云数据包头
  result_t parseHeader(
    uint8_t &zero, 
    uint32_t &headPos, 
    uint32_t timeout = DEFAULT_TIMEOUT / 2);
  //解析点云数据并判断带不带强度信息
  virtual result_t getIntensityFlag();
  //获取俯仰角值
  virtual bool getPitchAngle(float& pitch);

 private:
  /// package sample bytes
  int PackageSampleBytes;
  /// serial port
  ChannelDevice *_serial;
  /// sampling inteval
  uint32_t trans_delay;
  /// sampling rate
  int m_sampling_rate;
  /// LiDAR model
  int model;
  int sample_rate;

  /// has intensity protocol package
  tri_node_package2 package;
  /// TOF Lidar has intensity protocol package
  tof_node_package tof_package;
  /// non-intensity protocol package
  tri_node_package packages;

  float IntervalSampleAngle;
  float IntervalSampleAngle_LastPackage;
  /// First sample angle
  uint16_t FirstSampleAngle;
  /// last sample angle
  uint16_t LastSampleAngle;
  /// checksum
  uint16_t CheckSum;
  /// scan frequency
  uint8_t scan_frequence;

  uint16_t CheckSumCal;
  uint16_t SampleNumlAndCTCal;
  uint16_t LastSampleAngleCal;
  bool CheckSumResult;
  uint16_t Valu8Tou16;
  uint8_t ct;
  uint8_t nowPackageNum;
  uint8_t package_Sample_Num;

  uint8_t *globalRecvBuffer;
  bool has_device_header;
  uint8_t last_device_byte;
  int         asyncRecvPos;
  uint16_t    async_size;

  device_health health_;
  lidar_ans_header header_;
  uint8_t  *headerBuffer;
  uint8_t  *healthBuffer;
  bool get_device_health_success;

  int package_index;
  bool has_package_error;
  uint32_t m_heartbeat_ts;
  uint8_t m_BlockRevSize;

  uint32_t m_dataPos = 0; //记录当前解析到的数据的位置（解析是否带强度信息专用）
  uint64_t stamp = 0; //时间戳
  bool hasStamp = true; //是否有时间戳数据
};

}// namespace ydlidar

#endif // YDLIDAR_DRIVER_H
