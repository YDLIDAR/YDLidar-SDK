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
/** @mainpage CYdLidar(YDLIDAR SDK API)
    <table>
        <tr><th>Library     <td>CYdLidar
        <tr><th>File        <td>CYdLidar.h
        <tr><th>Author      <td>Tony [code at ydlidar com]
        <tr><th>Source      <td>https://github.com/ydlidar/YDLidar-SDK
        <tr><th>Version     <td>1.0.0
        <tr><th>Sample      <td>[ydlidar sample](\ref samples/ydlidar_test.cpp)[G1 G2 G4 G6 S2 X2 X4)\n
        [tof sample](\ref samples/tof_test.cpp)[TG15 TG30 TG50 TX8 TX20]\n
        [etlidar sample](\ref samples/etlidar_test.cpp)[T5 T15]
    </table>
    This API calls Two LiDAR interface classes in the following sections:
        - @subpage YDlidarDriver
        - @subpage ETLidarDriver
        - @subpage YDLIDAR C API
* @copyright    Copyright (c) 2018-2020  EAIBOT

    Jump to the @link ::CYdLidar @endlink interface documentation.

*/
#ifndef CYDLIDAR_H
#define CYDLIDAR_H
#include <core/base/utils.h>
#include <core/common/ydlidar_def.h>
#include <core/common/DriverInterface.h>
#include <string>
#include <map>


class YDLIDAR_API CYdLidar {
 public:
  /**
   * @brief create object
   */
  CYdLidar();
  /**
   * @brief destroy object
   */
  virtual ~CYdLidar();

  /**
   * @brief set lidar properties
   * @param optname        option name
   * @param optval         option value
   * - std::string(or char*)
   * - int
   * - bool
   * - float
   * @param optlen         option length
   * - data type size
   * @return true if the Property is set successfully, otherwise false.
   * @see LidarProperty
   */
  bool setlidaropt(int optname, const void *optval, int optlen);

  /**
   * @brief get lidar property
   * @param optname         option name
   * @param optval          option value
   * - std::string(or char*)
   * - int
   * - bool
   * - float
   * @param optlen          option length
   * - data type size
   * @return true if the Property is get successfully, otherwise false.
   * @see LidarProperty
   */
  bool getlidaropt(int optname, void *optval, int optlen);

  /**
   * @brief Initialize the SDK and LiDAR.
   * @return true if successfully initialized, otherwise false.
   */
  bool initialize();

  /**
  * @brief Return LiDAR's version information in a numeric form.
  * @param version Pointer to a version structure for returning the version information.
  */
  void GetLidarVersion(LidarVersion &version);

  /**
   * @brief Start the device scanning routine which runs on a separate thread and enable motor.
   * @return true if successfully started, otherwise false.
   */
  bool turnOn();

  //判断是否已启动扫描
  bool isScanning() const;
  /**
   * @brief Get the LiDAR Scan Data. turnOn is successful before doProcessSimple scan data.
   * @param[out] outscan             LiDAR Scan Data
   * @param[out] hardwareError       hardware error status
   * @return true if successfully started, otherwise false.
   */
  bool doProcessSimple(LaserScan &outscan);
  /**
   * @brief Stop the device scanning thread and disable motor.
   * @return true if successfully Stoped, otherwise false.
   */
  bool turnOff();
  /**
   * @brief Uninitialize the SDK and Disconnect the LiDAR.
   */
  void disconnecting();

  /**
   * @brief Get the last error information of a (socket or serial)
   * @return a human-readable description of the given error information
   * or the last error information of a (socket or serial)
   */
  const char *DescribeError() const;

  /**
   * @brief getDriverError
   * @return
   */
  DriverError getDriverError() const;

  /**
   * @brief 设置雷达工作模式（目前只针对GS2雷达）
   * @param[in] mode 雷达工作模式
   * @param[in] addr 雷达地址
   * @return 成功返回true，否则返回false
   */
  bool setWorkMode(int mode, uint8_t addr=0x00);
  
  /**
   * @brief 是否开启阳光噪点过滤功能
   * @param[in] e true开启，false关闭
   * @return 无
   */
  void enableSunNoise(bool e=true);

  /**
   * @brief 是否开启玻璃噪点过滤功能
   * @param[in] e true开启，false关闭
   * @return 无
   */
  void enableGlassNoise(bool e=true);

  /**
   * @brief 获取用户版本（目前只针对三角雷达）
   * @param[out] version 用户版本
   * @return 成功返回true，否则返回false
   */
  bool getUserVersion(std::string &version);

  //设置是否优先获取底板设备信息
  void setBottomPriority(bool yes=true);
  //获取设备信息
  bool getDeviceInfo(device_info& di, int type);
  //获取级联设备信息
  bool getDeviceInfo(std::vector<device_info_ex>& dis);
  //设置是否自动识别强度（启用时会占用一定时间）
  void setAutoIntensity(bool yes=false);
  //获取俯仰角值（目前仅针对Tmini Plus（森合））
  bool getPitchAngle(float& pitch);

  //启用调试
  void setEnableDebug(bool yes) {m_Debug = yes;}

  //OTA功能相关
  //设置OTA文件路径
  void setOtaFile(const std::string& name) {
    otaName = name;
  }
  //设置OTA文件加密
  void setOtaEncode(bool e) {
    otaEncode = e;
  }
  //开始OTA升级
  bool ota();

 private:
  /**
   * @brief check LiDAR instance and connect to LiDAR,
   *  try to create a comms channel.
   * @return true if communication has been established with the device.
   *  If it's not false on error.
   */
  bool checkConnect();
  /**
   * @brief check LiDAR health state and device information
   * @return true if health status and device information has been obtained with the device.
   * If it's not, false on error
   */
  bool checkStatus();

  /**
   * @brief check LiDAR scan state
   * @return true if the normal scan runs with the device.
   * If it's not, false on error.
   */
  bool checkHardware();

  /**
   * @brief Get LiDAR Health state
   * @return true if the device is in good health, If it's not
   */
  bool getDeviceHealth();

  /**
   * @brief Get LiDAR Device information
   * @return true if the device information is correct, If it's not
   */
  bool getDeviceInfo();

  /**
   * @brief check LiDAR Scan frequency
   * @return true if successfully checked, otherwise false.
   */
  bool checkScanFrequency();

  /**
   * @brief checkHeartBeat
   * @return
   */
  bool checkHeartBeat();

  /*!
   * @brief check LiDAR sample rate
   */
  void checkSampleRate();

  /**
   * @brief check LiDAR Data state
   * @return true if LiDAR Data is Normal, otherwise false.
   */
  bool checkLidarAbnormal();

  /**
   * @brief Calculate LiDAR Sampling rate
   * @param count       LiDAR Points
   * @param scan_time   LiDAR scan time
   * @return true if successfully calculated, otherwise false.
   */
  bool calcSampleRate(int count, double scan_time);

  /**
   * @brief Check if the LiDAR Offset Angle is corrected.
   * @param serialNumber    LiDAR serial number
   */
  bool checkCalibrationAngle(const std::string &serialNumber);

  /**
    * @brief Whether the current LiDAR range is valid
    * @param reading    current LiDAR point range
    * @return true if within valid range, otherwise false.
    */
  bool isRangeValid(double reading) const;

  /**
    * @brief Whether the current LiDAR point is ignored
    * @param angle    current LiDAR point angle
    * @return true if within ignore array, otherwise false.
    */
  bool isRangeIgnore(double angle) const;

  /**
    * @brief handle single-channel LiDAR device information
    * @note Start LiDAR successfully, handle single channel LiDAR
    * Device Information
    */
  void handleSingleChannelDevice();

  /**
    * @brief Parse Version by Package Information
    * @param debug LiDAR Point CT Pakcage Information
    */
  bool getDeviceInfoByPackage(const LaserDebug &debug);

  /**
   * @brief Calculate real-time sampling frequency
   * @param frequency       LiDAR current Scan Frequency
   * @param count           LiDAR Points
   * @param tim_scan_end    Last Scan Point Time Stamp
   * @param tim_scan_start  First Scan Point Time Stamp
   */
  void resample(int frequency, int count, uint64_t tim_scan_end,
                uint64_t tim_scan_start);
  /**
   * @brief Get zero correction angle
   * @return zero correction angle
   */
  float getAngleOffset() const;

  /**
   * @brief isAngleOffsetCorrected
   * @return true if successfully corrected, otherwise false.
   */
  bool isAngleOffsetCorrected() const;

 private:
  int     m_FixedSize;              ///< Fixed LiDAR Points
  float   m_AngleOffset;            ///< Zero angle offset value
  bool    m_isAngleOffsetCorrected; ///< Has the Angle offset been corrected
  float   frequencyOffset;          ///< Fixed Scan Frequency Offset
  int     lidar_model;              ///< LiDAR Model
  uint8_t Major;                    ///< Firmware Major Version
  uint8_t Minjor;                   ///< Firmware Minjor Version
  ydlidar::core::common::DriverInterface *lidarPtr; ///< LiDAR Driver Interface pointer
  uint64_t m_PointTime;             ///< Time interval between two sampling point
  uint64_t last_node_time;          ///< Latest LiDAR Start Node Time
  node_info *global_nodes;          ///< global nodes buffer
  double last_frequency;            ///< Latest Scan Frequency
  uint64_t m_FristNodeTime;         ///< Calculate real-time sample rate start time
  uint64_t m_AllNode;               ///< Sum of sampling points
  std::map<int, int> SampleRateMap; ///< Sample Rate Map
  std::string m_SerialNumber;       ///< LiDAR serial number
  // int defalutSampleRate;            ///< LiDAR Default Sampling Rate
  std::vector<int> defalutSampleRate; //默认采样率可能是多个值
  float m_field_of_view;            ///< LiDAR Field of View Angle.
  LidarVersion m_LidarVersion;      ///< LiDAR Version information
  float zero_offset_angle_scale;   ///< LiDAR Zero Offset Angle

 private:
  std::string m_SerialPort;         ///< LiDAR serial port
  std::string m_IgnoreString;       ///< LiDAR ignore array string
  std::vector<float> m_IgnoreArray; ///< LiDAR ignore array

  bool m_FixedResolution;           ///< LiDAR fixed angle resolution
  bool m_Reversion;                 ///< LiDAR reversion
  bool m_Inverted;                  ///< LiDAR inverted
  bool m_AutoReconnect;             ///< LiDAR hot plug
  bool m_SingleChannel;             ///< LiDAR single channel
  bool m_Intensity;                 ///< LiDAR Intensity
  int m_IntensityBit;               ///< LiDAR Intensity bit
  bool m_AutoIntensity; //自动识别强度
  bool m_SupportMotorDtrCtrl;       ///< LiDAR Motor DTR
  bool m_SupportHearBeat;           ///< LiDAR HeartBeat

  int m_SerialBaudrate;             ///< LiDAR serial baudrate or network port
  int m_LidarType;                  ///< LiDAR type
  int m_DeviceType;                 ///< LiDAR device type
  float m_SampleRate;                 ///< LiDAR sample rate
  int m_SampleRatebyD1;             ///< LiDAR sample rate by d1
  int m_AbnormalCheckCount;         ///< LiDAR abnormal count

  float m_MaxAngle;                 ///< LiDAR maximum angle
  float m_MinAngle;                 ///< LiDAR minimum angle
  float m_MaxRange;                 ///< LiDAR maximum range
  float m_MinRange;                 ///< LiDAR minimum range
  float m_ScanFrequency;            ///< LiDAR scanning frequency
  bool m_Bottom = true; //是否底板优先
  bool m_Debug = false; //是否启用调试

  bool m_SunNoise = false; //阳光噪点过滤标识
  bool m_GlassNoise = false; //玻璃噪点过滤标识
  std::string otaName; //OTA文件路径
  bool otaEncode = true; //OTA是否加密
  uint64_t lastStamp = 0; //时间戳
};	// End of class
#endif // CYDLIDAR_H

//os
namespace ydlidar 
{
/**
 * @brief system signal initialize
 */
YDLIDAR_API void os_init();
/**
 * @brief Whether system signal is initialized.
 * @return
 */
YDLIDAR_API bool os_isOk();
/**
 * @brief shutdown system signal
 */
YDLIDAR_API void os_shutdown();
/**
 * @brief lidarPortList
 * @return
 */
YDLIDAR_API std::map<std::string, std::string> lidarPortList();
//打印logo字符
YDLIDAR_API void printLogo();

}


