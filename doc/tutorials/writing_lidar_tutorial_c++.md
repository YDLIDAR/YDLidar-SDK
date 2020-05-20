# WritingLidarTutorial(c++)
Description: This tutorial covers how to write a lidar tutorial in C++.
Tutorial Level: BEGINNER
Next Tutorial: [Examining the simple lidar tutorial](examine_the_simple_lidar_tutorial.md)

## Table of Contents

- [Writing a Simple lidar tutorial (C++)](#writing-a-simple-lidar-tutorial-(C++))
    - [create beginner_tutorials directories](#create-beginner_tutorials-directories)
    - [The Code Explained](#the-code-explained)
- [Building your project](#building-your-project)

## Writing a Simple lidar tutorial (C++)
Description: This tutorial covers how to write a LiDAR data console program in C++.
Tutorial Level: BEGINNER

### create beginner_tutorials directories
```shell
mkdir beginner_tutorials
cd beginner_tutorials
```
### Create the lidar_tutorial.cpp file within the beginner_tutorials project and paste the following inside it: 
[https://github.com/YDLIDAR/ydlidar_tutorials/blob/master/cpp_tutorials/lidar_tutorial/lidar_tutorial.cpp](https://github.com/YDLIDAR/ydlidar_tutorials/blob/master/cpp_tutorials/lidar_tutorial/lidar_tutorial.cpp)

```c++
#include "CYdLidar.h"
#include <string>
using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

int main(int argc, char *argv[]) {
  // init system signal
  ydlidar::os_init();

  CYdLidar laser;
  //////////////////////string property/////////////////
  /// Lidar ports
  std::map<std::string, std::string> ports = ydlidar::lidarPortList();
  std::string port = "/dev/ydlidar";
  if(!ports.empty()) {
      port = ports.begin()->second;
  }
  /// lidar port
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
  b_optvalue = false;
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));
  /// HeartBeat
  b_optvalue = false;
  laser.setlidaropt(LidarPropSupportHeartBeat, &b_optvalue, sizeof(bool));


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
  f_optvalue = 10.f;
  laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

  // initialize SDK and LiDAR
  bool ret = laser.initialize();
  if (ret) {//success
    //Start the device scanning routine which runs on a separate thread and enable motor.
    ret = laser.turnOn();
  } else {
    fprintf(stderr, "%s\n", laser.DescribeError());
    fflush(stderr);
  }

  // Turn On success and loop  
  while (ret && ydlidar::os_isOk()) {
    LaserScan scan;
    if (laser.doProcessSimple(scan)) {
      fprintf(stdout, "Scan received[%llu]: %u ranges is [%f]Hz\n",
              scan.stamp,
              (unsigned int)scan.points.size(), 1.0 / scan.config.scan_time);
      fflush(stdout);
    } else {
      fprintf(stderr, "Failed to get Lidar Data\n");
      fflush(stderr);
    }
  }
  // Stop the device scanning thread and disable motor.
  laser.turnOff();
  // Uninitialize the SDK and Disconnect the LiDAR.
  laser.disconnecting();
  return 0;
}
```

### The Code Explained
Now, let's break the code down. 
```c++
#include "CYdLidar.h"
```
CYdLidar.h is a convenience include that includes all the headers necessary to use the most common public pieces of the YDLIDAR SDK. 

```c++
  ydlidar::os_init();
```
Initialize system signal. install a SIGINT handler which provides Ctrl-C handling

```c++
  CYdLidar laser;
```
Create a handle to this Lidar. 

```c++
 //////////////////////string property/////////////////
  /// Lidar ports
  std::map<std::string, std::string> ports = ydlidar::lidarPortList();
  std::string port = "/dev/ydlidar";
  if(!ports.empty()) {
      port = ports.begin()->second;
  }
```
Query avaliable Lidar ports.


```c++
  /// lidar port
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  /// ignore array
  std::string ignore_array;
  ignore_array.clear();
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                    ignore_array.size());
```
Set Lidar string property paramters.

```c++
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

```
Set Lidar string int paramters.


```c++
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

```
Set Lidar bool property paramters.


```c++
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
  f_optvalue = 10.f;
  laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));
```
Set Lidar float property paramters.

```c++
  // initialize SDK and LiDAR
  bool ret = laser.initialize();
```
Initialize the SDK and LiDAR.

`initialize` will return false if:
+ Serial port does not correspond to the actual Lidar.
+ Serial port does not have read and write permissions.
+ Lidar baud rate settings error.
+ Incorrect Lidar type setting.

```c++
  if (ret) {//success
    //Start the device scanning routine which runs on a separate thread and enable motor.
    ret = laser.turnOn();
  } else {
    fprintf(stderr, "%s\n", laser.DescribeError());
    fflush(stderr);
  }
```
Start the device scanning routine which runs on a separate thread and enable motor.

`turnOn` will return false if:
+ Lidar stall.
+ Lidar power suppy is unstable.


```c++
  // Turn On success and loop  
  while (ret && ydlidar::os_isOk()) {
```
By `ydlidar::os_init()` will install a SIGINT handler which provides Ctrl-C handling which will cause `ydlidar::os_isOk()` to return false if that happens.

`ydlidar::os_isOk()` will return false if:
+ a SIGINT is received (Ctrl-C)
+ ydlidar::os_shutdown() has been called by another part of the application.

Once `ydlidar::os_isOk()` returns false, Loop exit.

```c++
    LaserScan scan;
    if (laser.doProcessSimple(scan)) {
      fprintf(stdout, "Scan received[%llu]: %u ranges is [%f]Hz\n",
              scan.stamp,
              (unsigned int)scan.points.size(), 1.0 / scan.config.scan_time);
      fflush(stdout);
    } else {
      fprintf(stderr, "Failed to get Lidar Data\n");
      fflush(stderr);
    }
```
Get the LiDAR Scan Data.

```c++
  // Stop the device scanning thread and disable motor.
  laser.turnOff();
```
Stop the device scanning thread and disable motor.

```c++
  // Uninitialize the SDK and Disconnect the LiDAR.
  laser.disconnecting();
```
Uninitialize the SDK and Disconnect the LiDAR.

## Building your project
You need to create a CMakeLists.txt file.

The generated CMakeLists.txt should look like this:
[https://github.com/YDLIDAR/ydlidar_tutorials/blob/master/cpp_tutorials/lidar_tutorial/CMakeLists.txt](https://github.com/YDLIDAR/ydlidar_tutorials/blob/master/cpp_tutorials/lidar_tutorial/CMakeLists.txt)

```cmake
cmake_minimum_required(VERSION 2.8)
PROJECT(lidar_tutorial)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(-std=c++11) # Use C++11


#Include directories
include_directories(
     ${CMAKE_SOURCE_DIR}
)
############## YDLIDAR SDK START#####################################
#find ydlidar_sdk package
find_package(ydlidar_sdk REQUIRED)
#Include directories
include_directories(
  ${YDLIDAR_SDK_INCLUDE_DIRS}
)

#link library directories
link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} lidar_tutorial.cpp)

#Link your project to ydlidar_sdk library.
target_link_libraries(${PROJECT_NAME} ${YDLIDAR_SDK_LIBRARIES})

############## YDLIDAR SDK END#####################################
```
This will create one executable, lidar_tutorial, which by default will go into package directory of your build space.

Linux:
  * `YDLIDAR_SDK_LIBRARIES` includes `ydlidar_sdk pthread rt`
  * If you need the pthread library at the end of the compilation flag,
   you need to put `YDLIDAR_SDK_LIBRARIES` at the end.


you can use the following variable to depend on all necessary targets: 

```cmake
target_link_libraries(${PROJECT_NAME} ${YDLIDAR_SDK_LIBRARIES})
```
Now run cmake: 
```cmake
# In your project directory
mkdir build
cd build
cmake ..
make j4  
```
Now that you have written a simple lidar tutorial, let's [examine the simple lidar tutorial](examine_the_simple_lidar_tutorial.md). 
