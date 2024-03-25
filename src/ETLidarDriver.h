/*****************************************************************************
*  EAI TOF LIDAR DRIVER                                                      *
*  Copyright (C) 2018-2020 EAI TEAM  chushuifurong618@eaibot.com.                 *
*                                                                            *
*  This file is part of EAI TOF LIDAR DRIVER.                                *
*                                                                            *
*  @file     ETLidarDriver.h                                                 *
*  @brief    TOF LIDAR DRIVER                                                *
*  Details.                                                                  *
*                                                                            *
*  @author   Tony.Yang                                                       *
*  @email    chushuifurong618@eaibot.com                                     *
*  @version  1.0.0(Version)                                                    *
*  @date     chushuifurong618@eaibot.com                                     *
*                                                                            *
*                                                                            *
*----------------------------------------------------------------------------*
*  Remark         : Description                                              *
*----------------------------------------------------------------------------*
*  Change History :                                                          *
*  <Date>     | <Version> | <Author>       | <Description>                   *
*----------------------------------------------------------------------------*
*  2018/08/09 | 1.0.0     | Tony.Yang      | Create file                     *
*----------------------------------------------------------------------------*
*                                                                            *
*****************************************************************************/

/** @page ETLidarDriver
 * ETLidarDriver API
    <table>
        <tr><th>Library     <td>ETLidarDriver
        <tr><th>File        <td>ETLidarDriver.h
        <tr><th>Author      <td>Tony [code at ydlidar com]
        <tr><th>Source      <td>https://github.com/ydlidar/YDLidar-SDK
        <tr><th>Version     <td>1.0.0
    </table>
    This ETLidarDriver support [TYPE_TOF_NET](\ref LidarTypeID::TYPE_TOF_NET) LiDAR

* @copyright    Copyright (c) 2018-2020  EAIBOT
     Jump to the @link ::ydlidar::ETLidarDriver @endlink interface documentation.
*/

#pragma once

/* Header file to enable threading and ergo callback */
#include <core/base/thread.h>
#include <core/base/locker.h>
#include <vector>
#include <core/common/ydlidar_protocol.h>
#include <core/common/DriverInterface.h>
#include <core/common/ydlidar_help.h>
/* Header files for socket variable */


namespace ydlidar {
namespace core {
namespace network {
class CActiveSocket;
class CPassiveSocket;
}
}

using namespace core::common;
using namespace core::base;


class ETLidarDriver : public DriverInterface {
 public:

  /**
   * @brief ETLidarDriver
   * @param lidarIP
   * @param port
   */
  explicit ETLidarDriver();

  ~ETLidarDriver();


  /**
   * @brief Connecting Lidar \n
   * After the connection if successful, you must use ::disconnect to close
   * @param[in] port_path    Ip Address
   * @param[in] baudrate     network port
   * @return connection status
   * @retval 0     success
   * @retval < 0   failed
   * @note After the connection if successful, you must use ::disconnect to close
   * @see function ::YDlidarDriver::disconnect ()
   */
  virtual result_t connect(const char *port_path, uint32_t baudrate = 8000);


  /**
   * @brief Returns a human-readable description of the given error code
   *  or the last error code of a socket
   * @param isTCP   TCP or UDP
   * @return error information
   */
  virtual const char *DescribeError(bool isTCP = true);

  /**
  * @brief Disconnect from ETLidar device.
  */
  virtual void disconnect();

  /**
  * @brief Get SDK Version
  * @return version
  */
  virtual std::string getSDKVersion();

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
   * @brief Get lidar scan frequency \n
   * @param[in] frequency    scanning frequency
   * @param[in] timeout      timeout
   * @return return status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   * @note Non-scan state, perform currect operation.
   */
  virtual result_t getScanFrequency(scan_frequency &frequency,
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

  /**
  * @brief Get current scan configuration.
  * @returns scanCfg structure.
  */
  bool getScanCfg(lidarConfig &config, const std::string &ip_address = "");

  /**
  * @brief Get current scan update configuration.
  * @returns scanCfg structure.
  */
  lidarConfig getFinishedScanCfg();

  /**
   * @brief updateScanCfg
   * @param config
   */
  void updateScanCfg(const lidarConfig &config);

 private:
  /**
  * @brief Connect config port to ETLidar.
  * @param remote IP & port.
  */
  bool configPortConnect(const char *lidarIP, int tcpPort = 9000);

  /**
   * @brief disConfigConnect
   */
  void disConfigConnect();

  /**
  * @brief Set scan configuration.
  * @param cfg structure containing scan configuration.
  */
  void setScanCfg(const lidarConfig &config);


  /**
  * @brief Disconnect from ETLidar device.
  */
  char *configMessage(const char *descriptor, char *value = NULL);

  /**
  * @brief Start measurements.
  * After receiving this command ETLidar unit starts spinning laser and measuring.
  */
  bool startMeasure();

  /**
  * @brief Stop measurements.
  * After receiving this command ETLidar unit stop spinning laser and measuring.
  */
  bool stopMeasure();

  /**
  * @brief Connect data port to ETLidar.
  * @param remote IP & local port.
  */
  bool dataPortConnect(const char *lidarIP, int localPort = 8000);

  /**
   * @brief createThread
   * @return
   */
  result_t createThread();

  /**
   * @brief disableDataGrabbing
   */
  void disableDataGrabbing();
  /**
  * @brief Receive scan message.
  *
  * @param data pointer to lidarData buffer structure.
  */
  result_t getScanData();

  /**
   * @brief Turn on Lidar in Scanning thread \n
   * @param[in] force    Scan mode
   * @param[in] timeout  timeout
   * @return result status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   * @note Just turn it on once
   */
  result_t startAutoScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT) ;


  /**
   * @brief checkAutoConnecting
   * @return
   */
  result_t checkAutoConnecting();

  /**
   * @brief CheckLaserStatus
   */
  void CheckLaserStatus();

  /**
  * @brief parsing scan \n
  */
  int cacheScanData();

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
  * @brief Unpacking \n
  * @param[in] node lidar point information
  * @param[in] timeout     timeout
  */
  result_t waitPackage(node_info *node, uint32_t timeout = DEFAULT_TIMEOUT);


 private:
  /* Variable for LIDAR compatibility */
  Locker          _data_lock;       ///<
  lidarConfig     m_user_config;
  size_t          offset_len;
  int             m_AbnormalCheckCount;
  bool            m_force_update;
  float           m_lastAngle;
  float           m_currentAngle;
  /* ETLidar specific Variables */
  std::string               m_deviceIp;
  int                       port;
  int                       m_sampleRate;
  /* Sockets for ydlidar */
  core::network::CActiveSocket             *socket_cmd;
  core::network::CPassiveSocket            *socket_data;
  dataFrame                 frame;
  const char               *configValue[2] = {"0", "1"};
  bool m_isValidDevice;

};

} /* namespace */
