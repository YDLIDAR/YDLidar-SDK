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

/**
 * @ref "Dataset"
 * @par Dataset
<table>
<tr><th>LIDAR      <th> Model  <th>  Baudrate <th>  SampleRate(K) <th> Range(m)  		<th>  Frequency(HZ) <th> Intenstiy(bit) <th> SingleChannel<th> voltage(V)
<tr><th> F4        <td> 1	   <td>  115200   <td>   4            <td>  0.12~12         <td> 5~12           <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> S4        <td> 4	   <td>  115200   <td>   4            <td>  0.10~8.0        <td> 5~12 (PWM)     <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> S4B       <td> 4/11   <td>  153600   <td>   4            <td>  0.10~8.0        <td> 5~12(PWM)      <td> true(8)        <td> false    	  <td> 4.8~5.2
<tr><th> S2        <td> 4/12   <td>  115200   <td>   3            <td>  0.10~8.0     	<td> 4~8(PWM)       <td> false          <td> true    	  <td> 4.8~5.2
<tr><th> G4        <td> 5	   <td>  230400   <td>   9/8/4        <td>  0.28/0.26/0.1~16<td> 5~12        	<td> false          <td> false    	  <td> 4.8~5.2
<tr><th> X4        <td> 6	   <td>  128000   <td>   5            <td>  0.12~10     	<td> 5~12(PWM)      <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> X2/X2L    <td> 6	   <td>  115200   <td>   3            <td>  0.10~8.0     	<td> 4~8(PWM)       <td> false          <td> true    	  <td> 4.8~5.2
<tr><th> G4PRO     <td> 7	   <td>  230400   <td>   9/8/4        <td>  0.28/0.26/0.1~16<td> 5~12        	<td> false          <td> false    	  <td> 4.8~5.2
<tr><th> F4PRO     <td> 8	   <td>  230400   <td>   4/6          <td>  0.12~12         <td> 5~12        	<td> false          <td> false    	  <td> 4.8~5.2
<tr><th> R2        <td> 9	   <td>  230400   <td>   5            <td>  0.12~16         <td> 5~12        	<td> false          <td> false    	  <td> 4.8~5.2
<tr><th> G6        <td> 13     <td>  512000   <td>   18/16/8      <td>  0.28/0.26/0.1~25<td> 5~12        	<td> false          <td> false    	  <td> 4.8~5.2
<tr><th> G2A       <td> 14	   <td>  230400   <td>   5            <td>  0.12~12         <td> 5~12      	    <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> G2        <td> 15     <td>  230400   <td>   5            <td>  0.28~16     	<td> 5~12      	    <td> true(8)        <td> false    	  <td> 4.8~5.2
<tr><th> G2C       <td> 16	   <td>  115200   <td>   4            <td>  0.1~12        	<td> 5~12      	    <td> false      	<td> false    	  <td> 4.8~5.2
<tr><th> G4B       <td> 17	   <td>  512000   <td>   10           <td>  0.12~16         <td> 5~12        	<td> true(10)       <td> false    	  <td> 4.8~5.2
<tr><th> G4C       <td> 18	   <td>  115200   <td>   4            <td>  0.1~12		    <td> 5~12           <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> G1        <td> 19	   <td>  230400   <td>   9            <td>  0.28~16         <td> 5~12      	    <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> G5        <td> 20	   <td>  230400   <td>   9/8/4        <td>  0.28/0.26/0.1~16<td> 5~12        	<td> false          <td> false    	  <td> 4.8~5.2
<tr><th> G7        <td> 21         <td>  512000   <td>   18/16/8      <td>  0.28/0.26/0.1~25<td> 5~12        	<td> false          <td> false    	  <td> 4.8~5.2
<tr><th> TX8    　 <td> 100	   <td>  115200   <td>   4            <td>  0.05~8      	<td> 4~8(PWM)       <td> false          <td> true      	  <td> 4.8~5.2
<tr><th> TX20    　<td> 100	   <td>  115200   <td>   4            <td>  0.05~20      	<td> 4~8(PWM)       <td> false          <td> true     	  <td> 4.8~5.2
<tr><th> TG15    　<td> 100	   <td>  512000   <td>   20/18/10     <td>  0.05~30      	<td> 3~16      	    <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> TG30    　<td> 101	   <td>  512000   <td>   20/18/10     <td>  0.05~30      	<td> 3~16      	    <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> TG50    　<td> 102	   <td>  512000   <td>   20/18/10     <td>  0.05~50      	<td> 3~16      	    <td> false          <td> false    	  <td> 4.8~5.2
<tr><th> T15     　<td> 200	   <td>  8000     <td>   20           <td>  0.05~30      	<td> 5~35      	    <td> true           <td> false    	  <td> 4.8~5.2
</table>
 */

/**
 * @par example: G4/G5 LiDAR
 * @code
  ///< Defining an CYdLidar instance.
  CYdLidar laser;
  //////////////////////string property/////////////////
  /// lidar port
  std::string port = "/dev/ydlidar";
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  /// ignore array
  std::string ignore_array;
  ignore_array.clear();
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                    ignore_array.size());

  //////////////////////int property/////////////////
  /// lidar baudrate
  int optval = 230400;
  laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
  /// tof lidar
  optval = TYPE_TRIANGLE;
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  optval = YDLIDAR_TYPE_SERIAL;
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = 9;
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = false;
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  b_optvalue = false;
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
  /// intensity
  b_optvalue = false;
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));

  /// unit: m
  f_optvalue = 16.f;
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.1f;
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  f_optvalue = 10.f
  laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));
 * @endcode
 */

/**
 * @par example: S2 LiDAR
 * @code
  ///< Defining an CYdLidar instance.
  CYdLidar laser;
  //////////////////////string property/////////////////
  /// lidar port
  std::string port = "/dev/ydlidar";
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  /// ignore array
  std::string ignore_array;
  ignore_array.clear();
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                    ignore_array.size());

  //////////////////////int property/////////////////
  /// lidar baudrate
  int optval = 115200;
  laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
  /// tof lidar
  optval = TYPE_TRIANGLE;
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  optval = YDLIDAR_TYPE_SERIAL;
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = 3;
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = false;
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  b_optvalue = true;
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
  /// intensity
  b_optvalue = false;
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  b_optvalue = true;
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));

  /// unit: m
  f_optvalue = 10.f;
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.1f;
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  f_optvalue = 6.f
  laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));
 * @endcode
 */


/**
 * @par example: TG30 LiDAR
 * @code
  ///< Defining an CYdLidar instance.
  CYdLidar laser;
  //////////////////////string property/////////////////
  /// lidar port
  std::string port = "/dev/ydlidar";
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  /// ignore array
  std::string ignore_array;
  ignore_array.clear();
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                    ignore_array.size());

  //////////////////////int property/////////////////
  /// lidar baudrate
  int optval = 512000;
  laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
  /// tof lidar
  optval = TYPE_TOF;
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  optval = YDLIDAR_TYPE_SERIAL;
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = 20;
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = false;
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  b_optvalue = false;
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
  /// intensity
  b_optvalue = false;
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  b_optvalue = false;
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));

  /// unit: m
  f_optvalue = 64.f;
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.05f;
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  f_optvalue = 10.f
  laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));
 * @endcode
 */

/**
 * @par example: TX8 LiDAR
 * @code
  ///< Defining an CYdLidar instance.
  CYdLidar laser;
  //////////////////////string property/////////////////
  /// lidar port
  std::string port = "/dev/ydlidar";
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  /// ignore array
  std::string ignore_array;
  ignore_array.clear();
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                    ignore_array.size());

  //////////////////////int property/////////////////
  /// lidar baudrate
  int optval = 115200;
  laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
  /// tof lidar
  optval = TYPE_TOF;
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  optval = YDLIDAR_TYPE_SERIAL;
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = 4;
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = false;
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  b_optvalue = true;
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
  /// intensity
  b_optvalue = false;
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  b_optvalue = true;
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));

  /// unit: m
  f_optvalue = 12.f;
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.05f;
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  f_optvalue = 6.f
  laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));
 * @endcode
 */


/**
 * @par example: T15 LiDAR
 * @code
  ///< Defining an CYdLidar instance.
  CYdLidar laser;
  //////////////////////string property/////////////////
  /// lidar port
  std::string ipaddress = "192.168.1.11";
  laser.setlidaropt(LidarPropSerialPort, ipaddress.c_str(), ipaddress.size());
  /// ignore array
  std::string ignore_array;
  ignore_array.clear();
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                    ignore_array.size());

  //////////////////////int property/////////////////
  /// lidar port
  int optval = 8000;
  laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
  /// tof lidar
  optval = TYPE_TOF_NET;
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  optval = YDLIDAR_TYPE_TCP;
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = 20;
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = false;
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  b_optvalue = false;
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
  /// intensity
  b_optvalue = true;
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  b_optvalue = false;
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));

  /// unit: m
  f_optvalue = 64.f;
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.05f;
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  f_optvalue = 20.f
  laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));
 * @endcode
 */


/**
 * @par LidarPropMaxRange
 * @brief Set and Get LiDAR Maximum effective range.
 * @note The effective range beyond the maxmum is set to zero.\n
 * the MaxRange should be greater than the MinRange.
 * @remarks unit: m
 * @see [LidarPropMaxRange](\ref LidarPropMaxRange)
 * @see [DataSet](\ref Dataset)
 * @see CYdLidar::lidarSetProp and CYdLidar::lidarGetProp
 */


/**
 * @par LidarPropMinRange
 * @brief Set and Get LiDAR Minimum effective range.
 * @note The effective range less than the minmum is set to zero.\n
 * the MinRange should be less than the MaxRange.
 * @remarks unit: m
 * @see  LidarPropMinRange
 * @see  [Dataset](\ref Dataset)
 * @see CYdLidar::lidarSetProp and CYdLidar::lidarGetProp
 */

/**
 * @par LidarPropMaxAngle
 * @brief Set and Get LiDAR Maximum effective angle.
 * @note The effective angle beyond the maxmum will be ignored.\n
 * the MaxAngle should be greater than the MinAngle
 * @remarks unit: degree, Range:-180~180
 * @see [Dataset](\ref Dataset)
 * @see CYdLidar::lidarSetProp and CYdLidar::lidarGetProp
 */

/**
 * @par LidarPropMinAngle
 * @brief Set and Get LiDAR Minimum effective angle.
 * @note The effective angle less than the minmum will be ignored.\n
 * the MinAngle should be less than the MaxAngle
 * @remarks unit: degree, Range:-180~180
 * @see [Dataset](\ref Dataset)
 * @see CYdLidar::lidarSetProp and CYdLidar::lidarGetProp
 */

/**
 * @par LidarPropSampleRate
 * @brief Set and Get LiDAR Sampling rate.
 * @note If the set sampling rate does no exist.
 * the actual sampling rate is the LiDAR's default sampling rate.\n
 * Set the sampling rate to match the LiDAR.
 * @remarks unit: kHz/s, Ranges: 2,3,4,5,6,8,9,10,16,18,20\n
 <table>
      <tr><th>G4/G5/F4            <td>4,8,9
      <tr><th>F4PRO               <td>4,6
      <tr><th>G6/G7               <td>8,16,18
      <tr><th>G4B                 <td>10
      <tr><th>G1                  <td>9
      <tr><th>G2A/G2/R2/X4        <td>5
      <tr><th>S4/S4B/G4C/TX8/TX20 <td>4
      <tr><th>G2C                 <td>4
      <tr><th>S2                  <td>3
      <tr><th>TG15/TG30/TG50      <td>10,18,20
      <tr><th>T5/T15              <td>20
  </table>
 * @see CYdLidar::lidarSetProp and CYdLidar::lidarGetProp
 */

/**
 * @par LidarPropScanFrequency
 * @brief Set and Get LiDAR Scan frequency.
 * @note If the LiDAR is a single channel,
 * the scanning frequency nneds to be adjusted by external PWM.\n
 * Set the scan frequency to match the LiDAR.
 * @remarks unit: Hz\n
 <table>
      <tr><th>S2/X2/X2L/TX8/TX20              <td>4~8(PWM)
      <tr><th>F4/F4PRO/G4/G5/G4PRO/R2         <td>5~12
      <tr><th>G6/G7/G2A/G2/G2C/G4B/G4C/G1     <td>5~12
      <tr><th>S4/S4B/X4                       <td>5~12(PWM)
      <tr><th>TG15/TG30/TG50                  <td>3~16
      <tr><th>T5/T15                          <td>5~40
  </table>
 * @see CYdLidar::lidarSetProp and CYdLidar::lidarGetProp
 */

/**
 * @par LidarPropFixedResolution
 * @brief Set and Get LiDAR Fixed angluar resolution.\n
 * @note The Lidar scanning frequency will change slightly due to various reasons.
 * so the number of points per circle will also change slightly.\n
 * if a fixed angluar resolution is required.
 * a fixed number of points is required.
 * @details If set to true,
 * the angle_increment of the fixed angle resolution in LaserConfig will be a fixed value.
 * @see CYdLidar::lidarSetProp and CYdLidar::lidarGetProp
 */

/**
 * @par LidarPropReversion
 * @brief Set and Get LiDAR Reversion.\n
 * true: LiDAR data rotated 180 degrees.\n
 * false: Keep raw Data.\n
 * default: false\n
 * @note Refer to the table below for the LiDAR Reversion.\n
 * This is currently related to your coordinate system and install direction.
 * Whether to reverse it depends on your actual scene.
 * @par Reversion Table
 <table>
      <tr><th>LiDAR                           <th>reversion
      <tr><th>G1/G2/G2A/G2C/F4/F4PRO/R2       <td>true
      <tr><th>G4/G5/G4PRO/G4B/G4C/G6/G7       <td>true
      <tr><th>TG15/TG30/TG50                  <td>true
      <tr><th>T5/T15                          <td>true
      <tr><th>S2/X2/X2L/X4/S4/S4B             <td>false
      <tr><th>TX8/TX20                        <td>false
  </table>
 * @see CYdLidar::lidarSetProp and CYdLidar::lidarGetProp
 */

/**
 * @par LidarPropInverted
 * @brief Set and Get LiDAR inverted.\n
 * true: Data is counterclockwise\n
 * false: Data is clockwise\n
 * Default: clockwise
 * @note If set to true, LiDAR data direction is positive counterclockwise.
 * otherwise it is positive clockwise.
 * @see CYdLidar::lidarSetProp and CYdLidar::lidarGetProp
 */

/**
 * @par LidarPropAutoReconnect
 * @brief Set and Get LiDAR Automatically reconnect flag.\n
 * Whether to support hot plug.
 * @see CYdLidar::lidarSetProp and CYdLidar::lidarGetProp
 */

/**
* @par LidarPropSerialBaudrate
* @brief Set and Get LiDAR baudrate or network port.
* @note Refer to the table below for the LiDAR Baud Rate.\n
* Set the baudrate or network port to match the LiDAR.
* @remarks
<table>
     <tr><th>F4/S2/X2/X2L/S4/TX8/TX20/G4C        <td>115200
     <tr><th>X4                                  <td>128000
     <tr><th>S4B                                 <td>153600
     <tr><th>G1/G2/R2/G4/G5/G4PRO/F4PRO          <td>230400
     <tr><th>G2A/G2C                             <td>230400
     <tr><th>G6/G7/G4B/TG15/TG30/TG50            <td>512000
     <tr><th>T5/T15(network)                     <td>8000
 </table>
* @see CYdLidar::lidarSetProp and CYdLidar::lidarGetProp
*/

/**
 * @par LidarPropAbnormalCheckCount
 * @brief Set and Get LiDAR Maximum number of abnormal checks.
 * @note When the LiDAR Turn On, if the number of times of abnormal data acquisition
 * is greater than the current AbnormalCheckCount, the LiDAR Fails to Turn On.\n
 * @details The Minimum abnormal value is Two,
 * if it is less than the Minimum Value, it will be set to the Mimimum Value.\n
 * @see CYdLidar::lidarSetProp and CYdLidar::lidarGetProp
 */

/**
 * @par LidarPropSerialPort
 * @brief Set and Get LiDAR Serial port or network IP address.
 * @note If it is serial port,
 * your need to ensure that the serial port had read and write permissions.\n
 * If it is a network, make sure the network can ping.\n
 * @see CYdLidar::lidarSetProp and CYdLidar::lidarGetProp
 */

/**
 * @par LidarPropIgnoreArray
 * @brief Set and Get LiDAR  filtering angle area.
 * @note If the LiDAR angle is in the IgnoreArray,
 * the current range will be set to zero.\n
 * Filtering angles need to appear in pairs.\n
 * @details The purpose of the current paramter is to filter out the angular area set by user\n
 * @par example: Filters 10 degrees to 30 degrees and 80 degrees to 90 degrees.
 * @code
 *    CYdLidar laser;//Defining an CYdLidar instance.
 *    std::string ignore_array= "10.0, 30.0, 80.0, 90.0";
 *    laser.lidarSetProp(LidarPropIgnoreArray, ignore_array);
 * @endcode
 * @see CYdLidar::lidarSetProp and CYdLidar::lidarGetProp
 */

/**
 * @par LidarPropSingleChannel
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
 * @see CYdLidar::lidarSetProp and CYdLidar::lidarGetProp
 */

/**
* @par LidarPropLidarType
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
* @see CYdLidar::lidarSetProp and CYdLidar::lidarGetProp
*/


/**
 * @par LidarPropIntensity
 * @brief Set and Get LiDAR Intensity.
 * @note If the settings are reversed.
 * the LiDAR cannot parse the data correctly.\n
 * Set the Intensity to match the LiDAR.
 * @remarks
 <table>
      <tr><th>S4B/G2/G4B                             <td>true
      <tr><th>G4/G5/G4C/G4PRO/F4/F4PRO/G6/G7         <td>false
      <tr><th>G1/G2A/G2C/R2                          <td>false
      <tr><th>S2/X2/X2L/X4                           <td>false
      <tr><th>TG15/TG30/TG50                         <td>false
      <tr><th>TX8/TX20                               <td>false
      <tr><th>T5/T15                                 <td>true
      <tr><th>                                       <td>false
  </table>
 * @see CYdLidar::lidarSetProp and CYdLidar::lidarGetProp
 */


/**
* @par LidarPropDeviceType
* @brief Set and Get LiDAR connection Type.
* @note If you connect the LiDAR through the network to serial port adapter board.\n
* you need to set the current connection type to YDLIDAR_TYPE_TCP.\n
* otherwise set connection type to YDLIDAR_TYPE_SERIAL.\n
* Set the LiDAR connection Type to match the LiDAR.
* @see CYdLidar::lidarSetProp and CYdLidar::lidarGetProp
*/


/**
* @par LidarPropSupportMotorDtrCtrl
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
* @see CYdLidar::lidarSetProp and CYdLidar::lidarGetProp
*/

/**
* @par LidarPropSupportHeartBeat
* @brief Set and Get LiDAR Support HeartBeat.
* @note The current paramter settings are only valid
* if the LiDAR is BigScreen or Interactive Lidar.\n
* Set the LiDAR HeartBeat to match the LiDAR.
* @remarks
 <table>
      <tr><th>G4/G4PRO         <td>true
  </table>
* @see CYdLidar::lidarSetProp and CYdLidar::lidarGetProp
*/


/// Provides a platform independent class to for LiDAR development.
/// This class is designed to serial or socket communication development in a
/// platform independent manner.
/// - LiDAR types
///  -# ydlidar::YDlidarDriver Class
///  -# ydlidar::ETLidarDriver Class
///

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
   * @note set float property example, Must be float type, not double type.
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
  bool setlidaropt(int optname, const void *optval, int optlen);

  /**
   * @brief get lidar property
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
  bool  turnOn();
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
  bool  turnOff();
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

 private:
  /**
   * @brief check LiDAR instance and connect to LiDAR,
   *  try to create a comms channel.
   * @return true if communication has been established with the device.
   *  If it's not false on error.
   */
  bool  checkCOMMs();
  /**
   * @brief check LiDAR health state and device information
   * @return true if health status and device information has been obtained with the device.
   * If it's not, false on error
   */
  bool  checkStatus();

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
  bool CalculateSampleRate(int count, double scan_time);

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
    * @param debug  LiDAR Point CT Pakcage Information
    */
  void handleVersionInfoByPackage(const LaserDebug &debug);

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
  bool    isScanning;               ///< LiDAR is Scanning
  int     m_FixedSize;              ///< Fixed LiDAR Points
  float   m_AngleOffset;            ///< Zero angle offset value
  bool    m_isAngleOffsetCorrected; ///< Has the Angle offset been corrected
  float   frequencyOffset;          ///< Fixed Scan Frequency Offset
  int     lidar_model;              ///< LiDAR Model
  uint8_t Major;                    ///< Firmware Major Version
  uint8_t Minjor;                   ///< Firmware Minjor Version
  ydlidar::core::common::DriverInterface
  *lidarPtr;        ///< LiDAR Driver Interface pointer
  uint64_t m_PointTime;             ///< Time interval between two sampling point
  uint64_t last_node_time;          ///< Latest LiDAR Start Node Time
  node_info *global_nodes;          ///< global nodes buffer
  double last_frequency;            ///< Latest Scan Frequency
  uint64_t m_FristNodeTime;         ///< Calculate real-time sample rate start time
  uint64_t m_AllNode;               ///< Sum of sampling points
  std::map<int, int> SampleRateMap; ///< Sample Rate Map
  std::string m_SerialNumber;       ///< LiDAR serial number
  int defalutSampleRate;            ///< LiDAR Default Sampling Rate
  bool m_parsingCompleted;          ///< LiDAR Version Information is successfully parsed
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
  bool m_SupportMotorDtrCtrl;       ///< LiDAR Motor DTR
  bool m_SupportHearBeat;           ///< LiDAR HeartBeat

  int m_SerialBaudrate;             ///< LiDAR serial baudrate or network port
  int m_LidarType;                  ///< LiDAR type
  int m_DeviceType;                 ///< LiDAR device type
  int m_SampleRate;                 ///< LiDAR sample rate
  int m_SampleRatebyD1;             ///< LiDAR sample rate by d1
  int m_AbnormalCheckCount;         ///< LiDAR abnormal count

  float m_MaxAngle;                 ///< LiDAR maximum angle
  float m_MinAngle;                 ///< LiDAR minimum angle
  float m_MaxRange;                 ///< LiDAR maximum range
  float m_MinRange;                 ///< LiDAR minimum range
  float m_ScanFrequency;            ///< LiDAR scanning frequency


};	// End of class
#endif // CYDLIDAR_H

//os
namespace ydlidar {
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

}


