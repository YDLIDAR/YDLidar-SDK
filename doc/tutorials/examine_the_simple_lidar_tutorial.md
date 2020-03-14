# Examining the Simple Lidar Tutorial
Description: This tutorial examines running the simple lidar tutorial.
Tutorial Level: BEGINNER
Previous Tutorial: Writing a Simple Lidar Tutorial ([c](writing_lidar_tutorial_c.md))([python](writing_lidar_tutorial_python.md)) ([c++](writing_lidar_tutorial_c++.md))

## Table of Contents
- [Running the Lidar Turorial](#running-the-lidar-turorial)

### Running the Lidar Turorial
In the last tutorial we made a tutorial called "lidar_tutorial". Let's run it: 
```shell
./lidar_tutorial                (C++)(C)
python lidar_tutorial.py        (Python)
```
You will see something similar to: 

        YDLidar SDK initializing
        YDLidar SDK has been initialized
        [YDLIDAR]:SDK Version: 1.0.0
        LiDAR successfully connected
        [YDLIDAR]:Lidar running correctly ! The health status: good
        [YDLIDAR] Connection established in [/dev/ttyUSB0][230400]:
        Firmware version: 1.3
        Hardware version: 1
        Model: G4
        Serial: 2020010200010001
        [YDLIDAR INFO] Current Scan Frequency: 10.000000Hz
        LiDAR init success!
        [YDLIDAR]:Fixed Size: 900
        [YDLIDAR]:Sample Rate: 9K
        [YDLIDAR INFO] Current Sampling Rate : 9K
        [YDLIDAR INFO] Now YDLIDAR is scanning ......
        Scan received[1582955714469712000]: 964 ranges is [9.342144]Hz
        Scan received[1582955714607423000]: 954 ranges is [9.429421]Hz
        Scan received[1582955714742073000]: 949 ranges is [9.485030]Hz
        Scan received[1582955714875723000]: 946 ranges is [9.513148]Hz
        Scan received[1582955715008873000]: 943 ranges is [9.547170]Hz
        Scan received[1582955715141423000]: 938 ranges is [9.593015]Hz
        Scan received[1582955715273253000]: 933 ranges is [9.651109]Hz
        Scan received[1582955715404003000]: 930 ranges is [9.686395]Hz
        Scan received[1582955715534183000]: 928 ranges is [9.698230]Hz
        Scan received[1582955715664261000]: 919 ranges is [9.794232]Hz
        Scan received[1582955715792611000]: 906 ranges is [9.936508]Hz


When you are done, press Ctrl-C to terminate both the lidar tutorial. 

Note: ERROR
* ./lidar_tutorial: error while loading shared libraries: libydlidar_sdk.so: cannot open shared object file: No such file or directory
if the above error occurs, the  operation is as follows:

```shell
cat /etc/ld.so.conf
        include ld.so.conf.d/*.conf
## ydlidar_sdk library path added to ld.so.conf.
echo "/usr/local/lib" >> /etc/ld.so.conf
sudo ldconfig
```
OR
```shell
## ydlidar_sdk library path added to LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH   
```