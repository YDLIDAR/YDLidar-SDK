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

#include <sstream>
#include "ydlidar_sdk.h"
#include "CYdLidar.h"
#include "ydlidar_config.h"

YDLidar *lidarCreate() {
  CYdLidar *lidar = new CYdLidar();
  YDLidar *instance = new YDLidar;
  instance->lidar = NULL;
  instance->lidar = (void *)lidar;
  return instance;
}

void lidarDestroy(YDLidar **lidar) {
  if (lidar == NULL || *lidar == NULL) {
    return;
  }

  CYdLidar *drv = static_cast<CYdLidar *>((*lidar)->lidar);

  if (drv) {
    delete drv;
    drv = NULL;
  }

  (*lidar)->lidar = NULL;
  delete *lidar;
  *lidar = NULL;
  return;
}

bool setlidaropt(YDLidar *lidar, int optname, const void *optval, int optlen) {
  if (lidar == NULL || lidar->lidar == NULL || optval == NULL) {
    return false;
  }

  CYdLidar *drv = static_cast<CYdLidar *>(lidar->lidar);

  if (drv) {
    return drv->setlidaropt(optname, optval, optlen);
  }

  return false;
}

bool getlidaropt(YDLidar *lidar, int optname, void *optval, int optlen) {
  if (lidar == NULL || lidar->lidar == NULL || optval == NULL) {
    return false;
  }

  CYdLidar *drv = static_cast<CYdLidar *>(lidar->lidar);

  if (drv) {
    return drv->getlidaropt(optname, optval, optlen);
  }

  return false;
}


void GetSdkVersion(char *version) {
  strcpy(version, YDLIDAR_SDK_VERSION_STR);
}

bool initialize(YDLidar *lidar) {
  if (lidar == NULL || lidar->lidar == NULL) {
    return false;
  }

  CYdLidar *drv = static_cast<CYdLidar *>(lidar->lidar);

  if (drv) {
    return drv->initialize();
  }

  return false;
}

void GetLidarVersion(YDLidar *lidar, LidarVersion *version) {
  if (lidar == NULL || lidar->lidar == NULL || version == NULL) {
    return;
  }

  CYdLidar *drv = static_cast<CYdLidar *>(lidar->lidar);

  if (drv) {
    drv->GetLidarVersion(*version);
  }
}

bool turnOn(YDLidar *lidar) {
  if (lidar == NULL || lidar->lidar == NULL) {
    return false;
  }

  CYdLidar *drv = static_cast<CYdLidar *>(lidar->lidar);

  if (drv) {
    return drv->turnOn();
  }

  return false;
}

bool doProcessSimple(YDLidar *lidar, LaserFan *outscan) {
  if (lidar == NULL || lidar->lidar == NULL || outscan == NULL) {
    return false;
  }

  LaserFanDestroy(outscan);
  outscan->npoints = 0;

  CYdLidar *drv = static_cast<CYdLidar *>(lidar->lidar);

  if (drv) {
    LaserScan scan;
    bool ret = drv->doProcessSimple(scan);
    outscan->config = scan.config;
    outscan->stamp = scan.stamp;
    outscan->npoints = scan.points.size();
    outscan->points = (LaserPoint *)malloc(sizeof(LaserPoint) * outscan->npoints);
    std::copy(scan.points.begin(), scan.points.end(), outscan->points);
    return ret;
  }

  return false;
}

bool turnOff(YDLidar *lidar) {
  if (lidar == NULL || lidar->lidar == NULL) {
    return false;
  }

  CYdLidar *drv = static_cast<CYdLidar *>(lidar->lidar);

  if (drv) {
    return drv->turnOff();
  }

  return false;
}

void disconnecting(YDLidar *lidar) {
  if (lidar == NULL || lidar->lidar == NULL) {
    return;
  }

  CYdLidar *drv = static_cast<CYdLidar *>(lidar->lidar);

  if (drv) {
    drv->disconnecting();
  }
}

const char *DescribeError(YDLidar *lidar) {
  char const *value = "";

  if (lidar == NULL || lidar->lidar == NULL) {
    return value;
  }

  CYdLidar *drv = static_cast<CYdLidar *>(lidar->lidar);

  if (drv) {
    return drv->DescribeError();
  }

  return value;
}

void os_init() {
  ydlidar::os_init();
}

bool os_isOk() {
  return ydlidar::os_isOk();
}

void os_shutdown() {
  ydlidar::os_shutdown();

}

int lidarPortList(LidarPort *ports) {
  if (ports == NULL) {
    return 0;
  }

  memset(ports, 0, sizeof(LidarPort));
  std::map<std::string, std::string> lists = ydlidar::lidarPortList();
  std::map<std::string, std::string>::iterator it;

  int i = 0;

  for (it = lists.begin(); it != lists.end(); it++) {
    string_t port;
    memset(&port, 0, sizeof(string_t));

    if (i < sizeof(LidarPort) / sizeof(string_t)) {
      memcpy(ports->port[i].data, it->second.c_str(), it->second.size());
    }

    i++;
  }

  return i;

}
