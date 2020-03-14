# WritingLidarTutorial(python)
Description: This tutorial covers how to write a lidar tutorial in Python.
Tutorial Level: BEGINNER
Next Tutorial: [Examining the simple lidar tutorial](examine_the_simple_lidar_tutorial.md)

## Table of Contents

- [Writing a Simple lidar tutorial (Python)](#writing-a-simple-lidar-tutorial-(python))
    - [create beginner_tutorials directories](#create-beginner_tutorials-directories)
    - [The Code Explained](#the-code-explained)
    
## Writing a Simple lidar tutorial (Python)
Description: This tutorial covers how to write a LiDAR data console program in Python.
Tutorial Level: BEGINNER

### create beginner_tutorials directories
```shell
mkdir beginner_tutorials
cd beginner_tutorials
```
### Create the lidar_tutorial.py file within the beginner_tutorials project and paste the following inside it: 
[https://github.com/YDLIDAR/ydlidar_tutorials/blob/master/pyhton_tutorials/lidar_tutorial/lidar_tutorial.py](https://github.com/YDLIDAR/ydlidar_tutorials/blob/master/python_tutorials/lidar_tutorial/lidar_tutorial.py)

```python
import os
import ydlidar

if __name__ == "__main__":
    ydlidar.os_init();
    laser = ydlidar.CYdLidar();
    ports = ydlidar.lidarPortList();
    port = "/dev/ydlidar";
    for key, value in ports.items():
        port = value;
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 512000);
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF);
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL);
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0);
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 20);
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, False);

    ret = laser.initialize();
    if ret:
        ret = laser.turnOn();
        scan = ydlidar.LaserScan()
        while ret and ydlidar.os_isOk() :
            r = laser.doProcessSimple(scan);
            if r:
                print("Scan received[",scan.stamp,"]:",scan.points.size(),"ranges is [",1.0/scan.config.scan_time,"]Hz");
            else :
                print("Failed to get Lidar Data.")
        laser.turnOff();
    laser.disconnecting();
```

### The Code Explained
Now, let's break the code down. 
```python
import ydlidar
```
You need to import ydlidar if you are writing a YDLIDAR SDK.  

```python
  ydlidar.os_init();
```
Initialize system signal. install a SIGINT handler which provides Ctrl-C handling

```python
  laser = ydlidar.CYdLidar();
```
Create a handle to this Lidar. 

```python
    ports = ydlidar.lidarPortList();
    port = "/dev/ydlidar";
    for key, value in ports.items():
      port = value;
```
Query avaliable Lidar ports.

```python
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 512000);
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF);
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL);
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0);
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 20);
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, False);
```
Set Lidar property paramters.

```c++
  // initialize SDK and LiDAR
  ret = laser.initialize();
```
Initialize the SDK and LiDAR.

`initialize` will return false if:
+ Serial port does not correspond to the actual Lidar.
+ Serial port does not have read and write permissions.
+ Lidar baud rate settings error.
+ Incorrect Lidar type setting.

```python 
      if ret:
        ret = laser.turnOn();
```
Start the device scanning routine which runs on a separate thread and enable motor.

`turnOn` will return false if:
+ Lidar stall.
+ Lidar power suppy is unstable.


```python
  // Turn On success and loop  
  while ret and ydlidar.os_isOk() :
```
By `ydlidar.os_isOk()` will install a SIGINT handler which provides Ctrl-C handling which will cause `ydlidar.os_isOk()` to return false if that happens.

`ydlidar.os_isOk()` will return false if:
+ a SIGINT is received (Ctrl-C)
+ ydlidar.os_shutdown() has been called by another part of the application.

Once `ydlidar.os_isOk()` returns false, Loop exit.

```pyhton
      r = laser.doProcessSimple(scan);
      if r:
        print("Scan received[",scan.stamp,"]:",scan.points.size(),"ranges is [",1.0/scan.config.scan_time,"]Hz");
      else :
        print("Failed to get Lidar Data.")
```
Get the LiDAR Scan Data.

```python
  // Stop the device scanning thread and disable motor.
  laser.turnOff();
```
Stop the device scanning thread and disable motor.

```pyhton
  // Uninitialize the SDK and Disconnect the LiDAR.
  laser.disconnecting();
```
Uninitialize the SDK and Disconnect the LiDAR.


Now that you have written a simple lidar tutorial, let's [examine the simple lidar tutorial](examine_the_simple_lidar_tutorial.md). 
