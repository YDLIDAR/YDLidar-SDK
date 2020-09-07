#pragma once
#include <core/base/v8stdint.h>
#include <core/base/thread.h>
#include <core/base/locker.h>
#include <map>
#include "ydlidar_protocol.h"
#include "ydlidar_def.h"

namespace ydlidar {
namespace core {
using namespace base;
namespace common {

class DriverInterface {
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
         <tr><th>G4/G5/G4B/G4PRO/G6/G7/F4/F4PRO         <td>false
         <tr><th>S4/S4B/X4/R2/G4C                       <td>false
         <tr><th>S2/X2/X2L                              <td>true
         <tr><th>TG15/TG30/TG50                         <td>false
         <tr><th>TX8/TX20                               <td>true
         <tr><th>T5/T15                                 <td>false
         <tr><th>                                       <td>true
     </table>
    * @see DriverInterface::setSingleChannel and DriverInterface::getSingleChannel
    */
  PropertyBuilderByName(bool, SingleChannel, protected);
  /**
  * @brief Set and Get LiDAR Type.
  * @note Refer to the table below for the LiDAR Type.\n
  * Set the LiDAR Type to match the LiDAR.
  * @remarks
  <table>
       <tr><th>G1/G2A/G2/G2C                    <td>[TYPE_TRIANGLE](\ref LidarTypeID::TYPE_TRIANGLE)
       <tr><th>G4/G5/G4B/G4C/G4PRO              <td>[TYPE_TRIANGLE](\ref LidarTypeID::TYPE_TRIANGLE)
       <tr><th>G6/G7/F4/F4PRO                   <td>[TYPE_TRIANGLE](\ref LidarTypeID::TYPE_TRIANGLE)
       <tr><th>S4/S4B/X4/R2/S2/X2/X2L           <td>[TYPE_TRIANGLE](\ref LidarTypeID::TYPE_TRIANGLE)
       <tr><th>TG15/TG30/TG50/TX8/TX20          <td>[TYPE_TOF](\ref LidarTypeID::TYPE_TOF)
       <tr><th>T5/T15                           <td>[TYPE_TOF_NET](\ref LidarTypeID::TYPE_TOF_NET)
   </table>
  * @see [LidarTypeID](\ref LidarTypeID)
  * @see DriverInterface::setLidarType and DriverInterface::getLidarType
  */
  PropertyBuilderByName(int, LidarType, protected);
  /**
  * @brief Set and Get Sampling interval.
  * @note Negative correlation between sampling interval and lidar sampling rate.\n
  * sampling interval = 1e9 / sampling rate(/s)\n
  * Set the LiDAR sampling interval to match the LiDAR.
  * @see DriverInterface::setPointTime and DriverInterface::getPointTime
  */
  PropertyBuilderByName(uint32_t, PointTime, protected);
  /**
  * @brief Set and Get LiDAR Support Motor DTR.
  * @note The current paramter settings are only valid
  * if the LiDAR is connected to the serial port adapter via USB.\n
  * If the LiDAR does not have external motor enable line,
  * the current paramters do not need to be set.\n
  * Set the LiDAR Motro DTR to match the LiDAR.
  * @remarks
   <table>
        <tr><th>S4/S4B/S2/X2/X2L/X4                    <td>true
        <tr><th>TX8/TX20                               <td>true
        <tr><th>G4/G5/G4C/G4PRO/F4/F4PRO/G6/G7         <td>false
        <tr><th>G1/G2A/G2C/R2/G2/G4B                   <td>false
        <tr><th>TG15/TG30/TG50                         <td>false
        <tr><th>T5/T15                                 <td>false
    </table>
  * @see DriverInterface::setSupportMotorDtrCtrl and DriverInterface::getSupportMotorDtrCtrl
  */
  PropertyBuilderByName(bool, SupportMotorDtrCtrl, protected);

  /**
  * @brief Set and Get LiDAR HeartBeat function.
  * @note The current paramter settings are only valid
  * if the LiDAR is BigScreen.\n
  * Set the LiDAR HeartBeat to match the LiDAR.
  * @remarks
   <table>
        <tr><th>G4/G4PRO                               <td>false
    </table>
  * @see DriverInterface::setHeartBeat and DriverInterface::getHeartBeat
  */
  PropertyBuilderByName(bool, HeartBeat, protected);

  /**
   * @par Constructor
   *
   */
  DriverInterface() :  serial_port(""),
    m_baudrate(8000),
    m_intensities(false),
    scan_node_buf(NULL),
    scan_node_count(0),
    package_Sample_Index(0),
    retryCount(0),
    isAutoReconnect(true),
    isAutoconnting(false) {
    m_SingleChannel = false;
    m_LidarType = TYPE_TRIANGLE;
    m_PointTime = 0;
    m_SupportMotorDtrCtrl = true;
    m_HeartBeat  = false;
    m_isScanning = false;
    m_isConnected = false;
    m_config.motor_rpm = 1200;
    m_config.laserScanFrequency = 50;
    m_config.correction_angle = 20640;
    m_config.correction_distance = 6144;
    m_driverErrno         = NoError;
    m_InvalidNodeCount    = 0;
    m_BufferSize          = 0;
  }

  virtual ~DriverInterface() {}

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
  virtual result_t connect(const char *port_path, uint32_t baudrate = 8000) = 0;

  /**
   * @brief Returns a human-readable description of the given error code
   *  or the last error code of a socket or serial port
   * @param isTCP   TCP or UDP
   * @return error information
   */
  virtual const char *DescribeError(bool isTCP = true) = 0;

  static const char *DescribeDriverError(DriverError err) {
    char const *errorString = "Unknown error";

    switch (err) {
      case NoError:
        errorString = ("No error");
        break;

      case DeviceNotFoundError:
        errorString = ("Device is not found");
        break;

      case PermissionError:
        errorString = ("Device is not permission");
        break;

      case UnsupportedOperationError:
        errorString = ("unsupported operation");
        break;

      case NotOpenError:
        errorString = ("Device is not open");
        break;

      case TimeoutError:
        errorString = ("Operation timed out");
        break;

      case BlockError:
        errorString = ("Device Block");
        break;

      case NotBufferError:
        errorString = ("Device Failed");
        break;

      case TrembleError:
        errorString = ("Device Tremble");
        break;

      case LaserFailureError:
        errorString = ("Laser Failure");
        break;

      default:
        // an empty string will be interpreted as "Unknown error"
        break;
    }

    return errorString;
  }


  /*!
  * @brief Disconnect the LiDAR.
  */
  virtual void disconnect() = 0;

  /**
  * @brief Get SDK Version \n
  * static function
  * @return Version
  */
  virtual std::string getSDKVersion() = 0;

  /**
   * @brief Is the Lidar in the scan \n
   * @return scanning status
   * @retval true     scanning
   * @retval false    non-scanning
   */
  virtual bool isscanning() const = 0;

  /**
   * @brief Is it connected to the lidar \n
   * @return connection status
   * @retval true     connected
   * @retval false    Non-connected
   */
  virtual bool isconnected() const = 0;

  /**
   * @brief Is there intensity \n
   * @param[in] isintensities    intentsity
   *   true	intensity
   *   false no intensity
   */
  virtual void setIntensities(const bool &isintensities) = 0;

  /**
   * @brief whether to support hot plug \n
   * @param[in] enable    hot plug :
   *   true	support
   *   false no support
   */
  virtual void setAutoReconnect(const bool &enable) = 0;

  /**
  * @brief Get current scan update configuration.
  * @returns scanCfg structure.
  */
  virtual lidarConfig getFinishedScanCfg() const {
    return m_config;
  }

  /**
   * @brief get Health status \n
   * @return result status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE or RESULT_TIMEOUT   failed
   */
  virtual result_t getHealth(device_health &health,
                             uint32_t timeout = DEFAULT_TIMEOUT) = 0;

  /**
   * @brief get Device information \n
   * @param[in] info     Device information
   * @param[in] timeout  timeout
   * @return result status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE or RESULT_TIMEOUT   failed
   */
  virtual result_t getDeviceInfo(device_info &info,
                                 uint32_t timeout = DEFAULT_TIMEOUT) = 0;

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
                             uint32_t timeout = DEFAULT_TIMEOUT) = 0;

  /**
   * @brief turn off scanning \n
   * @return result status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE    failed
   */
  virtual result_t stop() = 0;


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
                                uint32_t timeout = DEFAULT_TIMEOUT) = 0 ;

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
                                    uint32_t timeout = DEFAULT_TIMEOUT) = 0;

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
                                       uint32_t timeout = DEFAULT_TIMEOUT) = 0;

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
                                       uint32_t timeout = DEFAULT_TIMEOUT) = 0;

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
                                          uint32_t timeout = DEFAULT_TIMEOUT) = 0;

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
                                          uint32_t timeout = DEFAULT_TIMEOUT) = 0;

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
                                   uint32_t timeout = DEFAULT_TIMEOUT) = 0;

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
                                   uint32_t timeout = DEFAULT_TIMEOUT) = 0;

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
                                      uint32_t timeout = DEFAULT_TIMEOUT) = 0;

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
                                    uint32_t timeout = DEFAULT_TIMEOUT) = 0;

  /**
   * @brief setDriverError
   * @param er
   */
  virtual void setDriverError(const DriverError &er) {
    ScopedLocker l(_error_lock);
    m_driverErrno = er;
  }

  /**
   * @brief getDriverError
   * @return
   */
  virtual DriverError getDriverError() {
    ScopedLocker l(_error_lock);
    return m_driverErrno;
  }

 public:
  enum {
    YDLIDAR_F4      = 1,/**< F4 LiDAR Model. */
    YDLIDAR_T1      = 2,/**< T1 LiDAR Model. */
    YDLIDAR_F2      = 3,/**< F2 LiDAR Model. */
    YDLIDAR_S4      = 4,/**< S4 LiDAR Model. */
    YDLIDAR_G4      = 5,/**< G4 LiDAR Model. */
    YDLIDAR_X4      = 6,/**< X4 LiDAR Model. */
    YDLIDAR_G4PRO   = 7,/**< G4PRO LiDAR Model. */
    YDLIDAR_F4PRO   = 8,/**< F4PRO LiDAR Model. */
    YDLIDAR_R2      = 9,/**< R2 LiDAR Model. */
    YDLIDAR_G10     = 10,/**< G10 LiDAR Model. */
    YDLIDAR_S4B     = 11,/**< S4B LiDAR Model. */
    YDLIDAR_S2      = 12,/**< S2 LiDAR Model. */
    YDLIDAR_G6      = 13,/**< G6 LiDAR Model. */
    YDLIDAR_G2A     = 14,/**< G2A LiDAR Model. */
    YDLIDAR_G2B     = 15,/**< G2 LiDAR Model. */
    YDLIDAR_G2C     = 16,/**< G2C LiDAR Model. */
    YDLIDAR_G4B     = 17,/**< G4B LiDAR Model. */
    YDLIDAR_G4C     = 18,/**< G4C LiDAR Model. */
    YDLIDAR_G1      = 19,/**< G1 LiDAR Model. */
    YDLIDAR_G5      = 20,/**< G5 LiDAR Model. */
    YDLIDAR_G7      = 21,/**< G7 LiDAR Model. */

    YDLIDAR_TG15    = 100,/**< TG15 LiDAR Model. */
    YDLIDAR_TG30    = 101,/**< T30 LiDAR Model. */
    YDLIDAR_TG50    = 102,/**< TG50 LiDAR Model. */

    YDLIDAR_T15     = 200,/**< T15 LiDAR Model. */
    YDLIDAR_Tail,
  };

  enum {
    YDLIDAR_RATE_4K = 0,/**< 4K sample rate code */
    YDLIDAR_RATE_8K = 1,/**< 8K sample rate code */
    YDLIDAR_RATE_9K = 2,/**< 9K sample rate code */
    YDLIDAR_RATE_10K = 3,/**< 10K sample rate code */
  };

 public:
  enum {
    DEFAULT_TIMEOUT = 2000,    /**< Default timeout. */
    DEFAULT_HEART_BEAT = 1000, /**< Default heartbeat timeout. */
    MAX_SCAN_NODES = 7200,	   /**< Default Max Scan Count. */
    DEFAULT_TIMEOUT_COUNT = 1, /**< Default Timeout Count. */
  };

 protected:
  /* Variable for LIDAR compatibility */
  /// LiDAR Scanning state
  bool            m_isScanning;
  /// LiDAR connected state
  bool            m_isConnected;
  /// Scan Data Event
  Event           _dataEvent;
  /// Data Locker
  Locker          _lock;
  /// Parse Data thread
  Thread          _thread;
  /// command locker
  Locker          _cmd_lock;
  /// driver error locker
  Locker          _error_lock;


  /// LiDAR com port or IP Address
  std::string serial_port;
  /// baudrate or IP port
  uint32_t m_baudrate;
  /// LiDAR intensity
  bool m_intensities;

  /// LiDAR Point pointer
  node_info      *scan_node_buf;
  /// LiDAR scan count
  size_t         scan_node_count;   //<! LiDAR Scan Count
  /// package sample index
  uint16_t package_Sample_Index;
  ///
  int retryCount;
  /// auto reconnect
  bool isAutoReconnect;
  /// auto connecting state
  bool isAutoconnting;
  lidarConfig     m_config;

  /// number of last error
  DriverError m_driverErrno;

  ///invalid node count
  int m_InvalidNodeCount;
  size_t m_BufferSize;

};

}//common
}//core
}//ydlidar
