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
/** @page YDLIDAR C API
 * YDLIDAR C API
    <table>
        <tr><th>Library     <td>ydlidar_sdk
        <tr><th>File        <td>ydlidar_sdk.h
        <tr><th>Author      <td>Tony [code at ydlidar com]
        <tr><th>Source      <td>https://github.com/ydlidar/YDLidar-SDK
        <tr><th>Version     <td>1.0.0
    </table>

* @copyright    Copyright (c) 2018-2020  EAIBOT
    Jump to the ydlidar_sdk.h interface documentation.
*/

#ifndef YDLIDAR_SDK_H_
#define YDLIDAR_SDK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <core/common/ydlidar_def.h>

/**
 * @ref "YDLIDAR_C_API"
 * @par YDLIDAR_C_API
 *
 */

/**
 * @brief create a Lidar instance
 * @note call ::lidarDestroy destroy
 * @return created instance
 */
YDLIDAR_API YDLidar *lidarCreate(void);

/**
 * @brief Destroy Lidar instance by ::lidarCreate create
 * @param lidar     CYdLidar instance
 */
YDLIDAR_API void lidarDestroy(YDLidar **lidar);

/**
 * @brief set lidar properties
 * @param lidar           a lidar instance
 * @param optname        option name
 * @todo string properties
 * - @ref LidarPropSerialPort
 * - @ref LidarPropIgnoreArray
 * @note set string property example
 * @code
 * CYdLidar laser;
 * std::string lidar_port = "/dev/ydlidar";
 * laser.setlidaropt(LidarPropSerialPort,lidar_port.c_str(), lidar_port.size());
 * @endcode
 * @todo int properties
 * - @ref LidarPropSerialBaudrate
 * - @ref LidarPropLidarType
 * - @ref LidarPropDeviceType
 * - @ref LidarPropSampleRate
 * @note set int property example
 * @code
 * CYdLidar laser;
 * int lidar_baudrate = 230400;
 * laser.setlidaropt(LidarPropSerialPort,&lidar_baudrate, sizeof(int));
 * @endcode
 * @todo bool properties
 * - @ref LidarPropFixedResolution
 * - @ref LidarPropReversion
 * - @ref LidarPropInverted
 * - @ref LidarPropAutoReconnect
 * - @ref LidarPropSingleChannel
 * - @ref LidarPropIntenstiy
 * @note set bool property example
 * @code
 * CYdLidar laser;
 * bool lidar_fixedresolution = true;
 * laser.setlidaropt(LidarPropSerialPort,&lidar_fixedresolution, sizeof(bool));
 * @endcode
 * @todo float properties
 * - @ref LidarPropMaxRange
 * - @ref LidarPropMinRange
 * - @ref LidarPropMaxAngle
 * - @ref LidarPropMinAngle
 * - @ref LidarPropScanFrequency
 * @note set float property example
 * @code
 * CYdLidar laser;
 * float lidar_maxrange = 16.0f;
 * laser.setlidaropt(LidarPropSerialPort,&lidar_maxrange, sizeof(float));
 * @endcode
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
YDLIDAR_API bool setlidaropt(YDLidar *lidar, int optname, const void *optval,
                             int optlen);

/**
 * @brief get lidar property
 * @param lidar           a lidar instance
 * @param optname         option name
 * @todo string properties
 * - @ref LidarPropSerialPort
 * - @ref LidarPropIgnoreArray
 * @note get string property example
 * @code
 * CYdLidar laser;
 * char lidar_port[30];
 * laser.getlidaropt(LidarPropSerialPort,lidar_port, sizeof(lidar_port));
 * @endcode
 * @todo int properties
 * - @ref LidarPropSerialBaudrate
 * - @ref LidarPropLidarType
 * - @ref LidarPropDeviceType
 * - @ref LidarPropSampleRate
 * @note get int property example
 * @code
 * CYdLidar laser;
 * int lidar_baudrate;
 * laser.getlidaropt(LidarPropSerialPort,&lidar_baudrate, sizeof(int));
 * @endcode
 * @todo bool properties
 * - @ref LidarPropFixedResolution
 * - @ref LidarPropReversion
 * - @ref LidarPropInverted
 * - @ref LidarPropAutoReconnect
 * - @ref LidarPropSingleChannel
 * - @ref LidarPropIntenstiy
 * @note get bool property example
 * @code
 * CYdLidar laser;
 * bool lidar_fixedresolution;
 * laser.getlidaropt(LidarPropSerialPort,&lidar_fixedresolution, sizeof(bool));
 * @endcode
 * @todo float properties
 * - @ref LidarPropMaxRange
 * - @ref LidarPropMinRange
 * - @ref LidarPropMaxAngle
 * - @ref LidarPropMinAngle
 * - @ref LidarPropScanFrequency
 * @note set float property example
 * @code
 * CYdLidar laser;
 * float lidar_maxrange;
 * laser.getlidaropt(LidarPropSerialPort,&lidar_maxrange, sizeof(float));
 * @endcode
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
YDLIDAR_API bool getlidaropt(YDLidar *lidar, int optname, void *optval,
                             int optlen);

/**
* Return SDK's version information in a numeric form.
* @param version Pointer to a version for returning the version information.
*/
YDLIDAR_API void GetSdkVersion(char *version);

/**
 * Initialize the SDK.
 * @return true if successfully initialized, otherwise false.
 */
YDLIDAR_API bool initialize(YDLidar *lidar);

/**
* @brief Return LiDAR's version information in a numeric form.
* @param version Pointer to a version structure for returning the version information.
*/
YDLIDAR_API void GetLidarVersion(YDLidar *lidar, LidarVersion *version);

/**
 * Start the device scanning routine which runs on a separate thread.
 * @return true if successfully started, otherwise false.
 */
YDLIDAR_API bool turnOn(YDLidar *lidar);

/**
 * @brief Get the LiDAR Scan Data. turnOn is successful before doProcessSimple scan data.
 * @param[in] lidar          LiDAR instance
 * @param[out] outscan       LiDAR Scan Data
 * @return true if successfully started, otherwise false.
 */
YDLIDAR_API bool doProcessSimple(YDLidar *lidar, LaserFan *outscan);
/**
 * @brief Stop the device scanning thread and disable motor.
 * @return true if successfully Stoped, otherwise false.
 */
YDLIDAR_API bool turnOff(YDLidar *lidar);
/**
 * @brief Uninitialize the SDK and Disconnect the LiDAR.
 */
YDLIDAR_API void disconnecting(YDLidar *lidar);

/**
 * @brief Get the last error information of a (socket or serial)
 * @return a human-readable description of the given error information
 * or the last error information of a (socket or serial)
 */
YDLIDAR_API const char *DescribeError(YDLidar *lidar);

/**
 * @brief initialize system signals
 */
YDLIDAR_API void os_init();
/**
 * @brief isOk
 * @return true if successfully initialize, otherwise false.
 */
YDLIDAR_API bool os_isOk();
/**
 * @brief os_shutdown
 */
YDLIDAR_API void os_shutdown();

/**
 * @brief get lidar serial port
 * @param ports serial port lists
 * @return valid port number
 */
YDLIDAR_API int lidarPortList(LidarPort *ports);

#ifdef __cplusplus
}
#endif

#endif  // YDLIDAR_SDK_H_
