# WritingLidarTutorial(C)
Description: This tutorial covers how to write a lidar tutorial in C.
Tutorial Level: BEGINNER
Next Tutorial: [Examining the simple lidar tutorial](examine_the_simple_lidar_tutorial.md)

## Table of Contents

- [Writing a Simple lidar tutorial (C)](#writing-a-simple-lidar-tutorial-(C))
    - [create beginner_tutorials directories](#create-beginner_tutorials-directories)
    - [The Code Explained](#the-code-explained)
- [Building your project](#building-your-project)

## Writing a Simple lidar tutorial (C)
Description: This tutorial covers how to write a LiDAR data console program in C.
Tutorial Level: BEGINNER

### create beginner_tutorials directories
```shell
mkdir beginner_tutorials
cd beginner_tutorials
```
### Create the lidar_tutorial.cpp file within the beginner_tutorials project and paste the following inside it: 
[https://github.com/YDLIDAR/ydlidar_tutorials/blob/master/c_tutorials/lidar_tutorial/lidar_tutorial.c](https://github.com/YDLIDAR/ydlidar_tutorials/blob/master/c_tutorials/lidar_tutorial/lidar_tutorial.c)

```c
//
// The MIT License (MIT)
//
// Copyright (c) 2019 EAIBOT. All rights reserved.
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#include "ydlidar_sdk.h"

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

int main(int argc, const char *argv[]) {

    os_init();
    YDLidar *laser = lidarCreate();
    //string prop
    char port[50] = "/dev/ydlidar";
    LidarPort ports;
    int size = lidarPortList(&ports);
    int i = 0;
    for(i =0; i < size; i++) {
        printf("port: %s\n", ports.port[i].data);
        /// last port
        strcpy(port, ports.port[i].data);
    }
    setlidaropt(laser, LidarPropSerialPort, port, sizeof(port));
    strcpy(port, "");
    setlidaropt(laser, LidarPropIgnoreArray,port, sizeof(port));

    //int prop
    int i_optvalue = 512000;
    setlidaropt(laser, LidarPropSerialBaudrate, &i_optvalue, sizeof(int));
    i_optvalue = TYPE_TOF;
    setlidaropt(laser, LidarPropLidarType, &i_optvalue, sizeof(int));
    i_optvalue = YDLIDAR_TYPE_SERIAL;
    setlidaropt(laser, LidarPropDeviceType, &i_optvalue, sizeof(int));
    i_optvalue = 20;
    setlidaropt(laser, LidarPropSampleRate, &i_optvalue, sizeof(int));

    //bool prop
    bool b_optval = true;
    setlidaropt(laser, LidarPropAutoReconnect, &b_optval, sizeof(bool));
    b_optval = false;
    setlidaropt(laser, LidarPropSingleChannel, &b_optval, sizeof(bool));
    setlidaropt(laser, LidarPropIntenstiy, &b_optval, sizeof(bool));
    setlidaropt(laser, LidarPropInverted, &b_optval, sizeof(bool));
    setlidaropt(laser, LidarPropReversion, &b_optval, sizeof(bool));
    setlidaropt(laser, LidarPropSupportMotorDtrCtrl, &b_optval, sizeof(bool));
    setlidaropt(laser, LidarPropFixedResolution, &b_optval, sizeof(bool));

    //float prop
    float f_optval = 10.f;
    setlidaropt(laser, LidarPropScanFrequency, &f_optval, sizeof(float));
    f_optval = 180.0f;
    setlidaropt(laser, LidarPropMaxAngle, &f_optval, sizeof(float));
    f_optval = -180.0f;
    setlidaropt(laser, LidarPropMinAngle, &f_optval, sizeof(float));
    f_optval = 64.f;
    setlidaropt(laser, LidarPropMaxRange, &f_optval, sizeof(float));
    f_optval = 0.05f;
    setlidaropt(laser, LidarPropMinRange, &f_optval, sizeof(float));

    getlidaropt(laser, LidarPropSerialBaudrate,&i_optvalue, sizeof(int));
    printf("baudrate: %d\n", i_optvalue);

    bool ret = initialize(laser);
    if(ret) {
        ret = turnOn(laser);
    }

    LaserFan scan;
    LaserFanInit(&scan);
    while (ret && os_isOk()) {
        if(doProcessSimple(laser, &scan)) {
            fprintf(stdout, "Scan received[%llu]: %u ranges is [%f]Hz\n",
                    scan.stamp,
                    (unsigned int)scan.npoints, 1.0 / scan.config.scan_time);
            fflush(stdout);
        } else {
            fprintf(stderr, "Failed to get Lidar Data\n");
            fflush(stderr);
        }
    }
    LaserFanDestroy(&scan);
    turnOff(laser);
    disconnecting(laser);
    lidarDestroy(&laser);
    return 0;

}
```

### The Code Explained
Now, let's break the code down. 
```c++
#include "ydlidar_sdk.h"
```
ydlidar_sdk.h is a convenience include that includes all the headers necessary to use the most common public pieces of the YDLIDAR SDK. 

```c
  os_init();
```
Initialize system signal. install a SIGINT handler which provides Ctrl-C handling

```c
  YDLidar *laser = lidarCreate();
```
Create a handle to this Lidar. 

```c
  //string prop
  char port[50] = "/dev/ydlidar";
  LidarPort ports;
  int size = lidarPortList(&ports);
  int i = 0;
  for(i =0; i < size; i++) {
      printf("port: %s\n", ports.port[i].data);
      /// last port
      strcpy(port, ports.port[i].data);
  }
```
Query avaliable Lidar ports.


```c
  setlidaropt(laser, LidarPropSerialPort, port, sizeof(port));
  strcpy(port, "");
  setlidaropt(laser, LidarPropIgnoreArray,port, sizeof(port));
```
Set Lidar string property paramters.

```c
  //int prop
  int i_optvalue = 512000;
  setlidaropt(laser, LidarPropSerialBaudrate, &i_optvalue, sizeof(int));
  i_optvalue = TYPE_TOF;
  setlidaropt(laser, LidarPropLidarType, &i_optvalue, sizeof(int));
  i_optvalue = YDLIDAR_TYPE_SERIAL;
  setlidaropt(laser, LidarPropDeviceType, &i_optvalue, sizeof(int));
  i_optvalue = 20;
  setlidaropt(laser, LidarPropSampleRate, &i_optvalue, sizeof(int));
```
Set Lidar string int paramters.


```c
  //bool prop
  bool b_optval = true;
  setlidaropt(laser, LidarPropAutoReconnect, &b_optval, sizeof(bool));
  b_optval = false;
  setlidaropt(laser, LidarPropSingleChannel, &b_optval, sizeof(bool));
  setlidaropt(laser, LidarPropIntenstiy, &b_optval, sizeof(bool));
  setlidaropt(laser, LidarPropInverted, &b_optval, sizeof(bool));
  setlidaropt(laser, LidarPropReversion, &b_optval, sizeof(bool));
  setlidaropt(laser, LidarPropSupportMotorDtrCtrl, &b_optval, sizeof(bool));
  setlidaropt(laser, LidarPropFixedResolution, &b_optval, sizeof(bool));
```
Set Lidar bool property paramters.


```c
  //float prop
  float f_optval = 10.f;
  setlidaropt(laser, LidarPropScanFrequency, &f_optval, sizeof(float));
  f_optval = 180.0f;
  setlidaropt(laser, LidarPropMaxAngle, &f_optval, sizeof(float));
  f_optval = -180.0f;
  setlidaropt(laser, LidarPropMinAngle, &f_optval, sizeof(float));
  f_optval = 64.f;
  setlidaropt(laser, LidarPropMaxRange, &f_optval, sizeof(float));
  f_optval = 0.05f;
  setlidaropt(laser, LidarPropMinRange, &f_optval, sizeof(float));
```
Set Lidar float property paramters.

```c
  // initialize SDK and LiDAR
  bool ret = initialize(laser);
```
Initialize the SDK and LiDAR.

`initialize` will return false if:
+ Serial port does not correspond to the actual Lidar.
+ Serial port does not have read and write permissions.
+ Lidar baud rate settings error.
+ Incorrect Lidar type setting.

```c
  if(ret) {
    ret = turnOn(laser);
  }
```
Start the device scanning routine which runs on a separate thread and enable motor.

`turnOn` will return false if:
+ Lidar stall.
+ Lidar power suppy is unstable.


```c
  // Turn On success and loop  
  LaserFan scan;
  LaserFanInit(&scan);
  while (ret && os_isOk()) {
```
By `os_init()` will install a SIGINT handler which provides Ctrl-C handling which will cause `os_isOk()` to return false if that happens.

`os_isOk()` will return false if:
+ a SIGINT is received (Ctrl-C)
+ ydlidar::os_shutdown() has been called by another part of the application.

Once `os_isOk()` returns false, Loop exit.
Note: 
+ `LaserFan` need to be initialized with `LaserFanInit`
- After `LaserFan` leaves the Scope, it need to be destroyed with `LaserFanDestroy`, otherwisw it will leak memory.

```c
    if(doProcessSimple(laser, &scan)) {
        fprintf(stdout, "Scan received[%llu]: %u ranges is [%f]Hz\n",
                scan.stamp,
                (unsigned int)scan.npoints, 1.0 / scan.config.scan_time);
        fflush(stdout);
    } else {
        fprintf(stderr, "Failed to get Lidar Data\n");
        fflush(stderr);
    }
```
Get the LiDAR Scan Data.

```c
  LaserFanDestroy(&scan);
```
Destroy LaserFan, Free up memory.
Note:
  + After `LaserFan` leaves the Scope, it need to be destroyed with `LaserFanDestroy`, otherwisw it will leak memory.

```c
  // Stop the device scanning thread and disable motor.
  turnOff(laser);
```
Stop the device scanning thread and disable motor.

```c
  // Uninitialize the SDK and Disconnect the LiDAR.
  disconnecting(laser);
```
Uninitialize the SDK and Disconnect the LiDAR.

```
  lidarDestroy(&laser);
```
Destroy YDLidar, Free up memory.

## Building your project
You need to create a CMakeLists.txt file.

The generated CMakeLists.txt should look like this:
[https://github.com/YDLIDAR/ydlidar_tutorials/blob/master/c_tutorials/lidar_tutorial/CMakeLists.txt](https://github.com/YDLIDAR/ydlidar_tutorials/blob/master/c_tutorials/lidar_tutorial/CMakeLists.txt)

```cmake
cmake_minimum_required(VERSION 2.8)
PROJECT(lidar_tutorial C)

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

add_executable(${PROJECT_NAME} lidar_tutorial.c)
#Link your project to ydlidar_sdk library.
target_link_libraries(${PROJECT_NAME} ${YDLIDAR_SDK_LIBRARIES} -lstdc++ -lm)

############## YDLIDAR SDK END#####################################
```
This will create one executable, lidar_tutorial, which by default will go into package directory of your build space.

ydlidar_sdk dependent libraries
  * C++ standard library.
  * math library.

Note:
* GCC CCLDFLAGS requires "-lstdc++ -lm".

you can use the following variable to depend on all necessary targets: 

```cmake
target_link_libraries(${PROJECT_NAME} ${YDLIDAR_SDK_LIBRARIES} -lstdc++ -lm)
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
