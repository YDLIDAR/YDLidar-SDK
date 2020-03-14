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
#include <string.h>
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
