#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include <math.h>
#include <core/base/timer.h>
#include "CYdLidar.h"
#include "core/common/ydlidar_help.h"
#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

using namespace std;
using namespace ydlidar;

#define LIDAR0 0
#define LIDAR1 1
#define LIDAR2 2

//GS安装参数
struct YdGsRigParam
{
  float high = .0; //安装离地高度，单位mm
  float laserPitch = .0; //激光器倾斜角度，单位度
  float modulePitch = .0; //模组安装倾斜角度，单位度
};
//GS外参项
struct YdGsOutParamItem
{
  float k0 = .0;
  float b0 = .0;
  float k1 = .0;
  float b1 = .0;

  bool isNull() const {
    return ISZERO(k0) && ISZERO(b0) && ISZERO(k1) && ISZERO(b1);
  }
};
//GS外参
struct YdGsOutParam
{
  YdGsRigParam rp;
  YdGsOutParamItem items[GS_PACKMAXNODES];

  YdGsOutParam() {
    memset(items, 0, sizeof(items));
  }
};
//2D点
struct Yd2DPoint
{
  float x = .0; //x值，单位mm；弧度值，单位弧度
  float y = .0; //y值，单位mm；距离值，单位mm

  Yd2DPoint(float x = .0, float y = .0) : x(x), y(y) {
  }
  Yd2DPoint toAngular() const {
    return Yd2DPoint(y * cos(x), y * sin(x));
  } //极坐标转直角坐标
  Yd2DPoint toPolar() const {
    float theta = .0;
    if (!ISZERO(x)) // x不为0时
    {
        theta = atan(y / x);
        if (x > 0.0)
        {
            if (y < 0.0)
                theta += (M_PI * 2.0);
        }
        else
        {
            theta += M_PI;
        }
    }
    return Yd2DPoint(theta, sqrt(x * x + y * y));
  } //直角坐标转极坐标
  static float toRadian(float d) {
    return d * M_PI / 180.0;
  } //角度转弧度
  static float toDegree(float r) {
    return r * 180.0 / M_PI;
  } //弧度转角度
};
//3D点
struct Yd3DPoint
{
  float x = .0; //x值，单位mm
  float y = .0; //y值，单位mm
  float z = .0; //z值，单位mm
};
struct Yd3DPoints
{
  uint8_t id = 0; //模组序号
  std::vector<Yd3DPoint> points; //3D点云
};


bool parseCsv(const std::string& name, YdGsOutParam& op);
bool to3D(const LaserScan& scan, const YdGsOutParam& op, Yd3DPoints& out);

int main(int argc, char *argv[]) 
{
  printf("__   ______  _     ___ ____    _    ____  \n");
  printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
  printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
  printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
  printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
  printf("\n");
  fflush(stdout);

  ydlidar::os_init();

  //串口号或IP地址
  std::string port;
  std::map<std::string, std::string> ports = ydlidar::lidarPortList();
  std::map<std::string, std::string>::iterator it;
  ports["IP1"] = "192.168.1.200";
  ports["IP2"] = "192.168.152.1";
  if (ports.size() == 1) {
    port = ports.begin()->second;
  } else {
    int id = 0;
    for (it = ports.begin(); it != ports.end(); it++) {
      printf("[%d] %s %s\n", id, it->first.c_str(), it->second.c_str());
      id++;
    }

    if (ports.empty()) {
      printf("Not Lidar was detected. Please enter the lidar serial port:");
      std::cin >> port;
    } else {
      while (ydlidar::os_isOk()) {
        printf("Please select the lidar port:");
        std::string number;
        std::cin >> number;

        if ((size_t)atoi(number.c_str()) >= ports.size()) {
          continue;
        }

        it = ports.begin();
        id = atoi(number.c_str());

        while (id) {
          id--;
          it++;
        }
        port = it->second;
        break;
      }
    }
  }

  int baudrate = 921600;
  std::map<int, int> baudrateList;
  baudrateList[0] = 8000; //网络端口
  baudrateList[1] = 921600; //串口波特率
  printf("Baudrate:\n");
  for (std::map<int, int>::iterator it = baudrateList.begin();
       it != baudrateList.end(); it++) {
    printf("[%d] %d\n", it->first, it->second);
  }
  while (ydlidar::os_isOk()) 
  {
    printf("Please select the lidar baudrate:");
    std::string number;
    std::cin >> number;

    if ((size_t)atoi(number.c_str()) > baudrateList.size()) {
      continue;
    }

    baudrate = baudrateList[atoi(number.c_str())];
    break;
  }

  bool isSingleChannel = false;
  float frequency = 28.0;

  CYdLidar laser;
  //////////////////////string property/////////////////
  /// lidar port
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  /// ignore array
  std::string ignore_array;
  ignore_array.clear();
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(), ignore_array.size());
  //////////////////////int property/////////////////
  /// lidar baudrate
  laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
  /// gs lidar
  int optval = TYPE_GS;
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type (YDLIDAR_TYPE_TCP,YDLIDAR_TYPE_SERIAL)
  optval = (baudrate == baudrateList[0]) ? YDLIDAR_TYPE_TCP : YDLIDAR_TYPE_SERIAL; 
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = isSingleChannel ? 3 : 4;
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
  /// Intenstiy bit count
  optval = 8;
  laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));
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
  laser.setlidaropt(LidarPropSingleChannel, &isSingleChannel, sizeof(bool));
  /// intensity
  b_optvalue = true;
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  b_optvalue = true;
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));
  /// HeartBeat
  b_optvalue = false;
  laser.setlidaropt(LidarPropSupportHeartBeat, &b_optvalue, sizeof(bool));
  //////////////////////float property/////////////////
  // unit: °
  float f_optvalue = 180.0f;
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  // unit: m
  f_optvalue = 1.f;
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.025f;
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  // unit: Hz
  laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

  //是否启用调试
  laser.setEnableDebug(false); 

  //雷达初始化
  bool ret = laser.initialize();
  if (!ret)
  {
    fprintf(stderr, "Fail to initialize %s\n", laser.DescribeError());
    fflush(stderr);
    return -1;
  }
  //设置雷达工作模式（0表示避障模式，1表示沿边模式）
  ret &= laser.setWorkMode(0, LIDAR_MODULE_1);
  ret &= laser.setWorkMode(0, LIDAR_MODULE_2);
  ret &= laser.setWorkMode(1, LIDAR_MODULE_3);
  if (!ret)
  {
    fprintf(stderr, "Fail to set work mode %s\n", laser.DescribeError());
    fflush(stderr);
    return -1;
  }
  //启动扫描
  ret = laser.turnOn();
  if (!ret)
  {
    fprintf(stderr, "Fail to turn on %s\n", laser.DescribeError());
    fflush(stderr);
    return -1;
  }

  //初始化外参（需要根据实际情况设置参数）
  YdGsOutParam ops[LIDAR_MAXCOUNT];
  ops[LIDAR0].rp.high = 60.0;
  ops[LIDAR0].rp.laserPitch = 17.0;
  ops[LIDAR0].rp.modulePitch = 10.0;
  ops[LIDAR1].rp.high = 60.0;
  ops[LIDAR1].rp.laserPitch = 17.0;
  ops[LIDAR1].rp.modulePitch = 10.0;
  ops[LIDAR2].rp.high = 58.0;
  ops[LIDAR2].rp.laserPitch = 17.0;
  ops[LIDAR2].rp.modulePitch = 15.7;
  //从文件中解析外参
  if (!parseCsv("../examples/data/lidar0.csv", ops[LIDAR0]) ||
    !parseCsv("../examples/data/lidar1.csv", ops[LIDAR1]) ||
    !parseCsv("../examples/data/lidar2.csv", ops[LIDAR2]))
  {
    return -1;
  }

  LaserScan scan;
  Yd3DPoints out;
  while (ret && ydlidar::os_isOk())
  {
    if (laser.doProcessSimple(scan))
    {
      printf("Module [%d] [%d] points in [%.02f]Hz\n",
        scan.moduleNum,
        int(scan.points.size()),
        scan.scanFreq);
      //打印2D点云
      // for (size_t i = 0; i < scan.points.size(); ++i)
      // {
      //   const LaserPoint &p = scan.points.at(i);
      //   printf("%d a %.01f r %.01f\n", int(i), p.angle * 180.0f / M_PI, p.range * 1000.0f);
      // }
      // fflush(stdout);
      //转3D点云
      if (to3D(scan, ops[scan.moduleNum], out))
      {
        //打印3D点云
        for (size_t i=0; i<out.points.size(); ++i)
        {
          const Yd3DPoint& p = out.points.at(i);
          printf("i %d x %.01f y %.01f z %.01f\n", int(i), p.x, p.y, p.z);
        }
        fflush(stdout);
      }
    }
    else
    {
      fprintf(stderr, "Failed to get Lidar Data\n");
      fflush(stderr);
    }
  }

  laser.turnOff();
  laser.disconnecting();
  return 0;
}

#include <iostream>
#include <fstream>
#include <cstdio>
bool parseCsv(const std::string& name, YdGsOutParam& op)
{
  std::ifstream file(name); // 打开文件
  if (!file.is_open()) {
      std::cerr << "无法打开文件 " << name << std::endl;
      return false;
  }
  // 逐行读取文件内容
  std::string line;
  std::string format = "%f %f %f %f";
  int index = 0;
  while (getline(file, line)) 
  { 
    if (line.empty())
      continue;
    if (std::sscanf(line.c_str(), format.c_str(), 
      &op.items[index].b0, 
      &op.items[index].b1,
      &op.items[index].k0,
      &op.items[index].k1) == 4) //解析成功
    {
      //打印参数
      // std::cout << op.items[index].b0 << " "
      //   << op.items[index].b1 << " "
      //   << op.items[index].k0 << " "
      //   << op.items[index].k1 << std::endl;
      index ++;
      if (index >= GS_PACKMAXNODES)
        break;
    }
    else //解析失败
    {
      std::cout << "解析异常 " << line << std::endl;
    }
  }

  file.close();
  return true;
}

//2D点云转3D点云
bool to3D(const LaserScan& scan, const YdGsOutParam& op, Yd3DPoints& out)
{
  int size = scan.points.size();
  if (size > GS_PACKMAXNODES)
  {
    std::cout << "点云数过大 " << size << std::endl;
    return false;
  }
  out.id = scan.moduleNum;
  out.points.resize(size);

  for (int i=0; i<size; ++i)
  {
    const LaserPoint &p = scan.points.at(i);
    //如果点云距离无效则跳过
    if (ISZERO(p.range))
      continue;
    //如果外参无效则跳过
    const YdGsOutParamItem& item = op.items[i];
    if (item.isNull())
      continue;
    Yd3DPoint& point = out.points[i];
    //2d数据
    Yd2DPoint p2 = Yd2DPoint(p.angle, p.range * 1000.0).toAngular();
    if (LIDAR2 != out.id) //左前或右前雷达
    {
      //矫正数据（计算公式可参考文档）
      float xOffset = p2.x / cos(Yd2DPoint::toRadian(op.rp.laserPitch));
      float x = xOffset * cos(Yd2DPoint::toRadian(op.rp.laserPitch + op.rp.modulePitch));
      float y = xOffset * sin(Yd2DPoint::toRadian(op.rp.laserPitch + op.rp.modulePitch));
      float r = sqrt(x * x + p2.y * p2.y);
      float a = asin(p2.y / r);
      // p2 = Yd2DPoint(a, r).toAngular(); //极坐标转直角坐标
      // point.x = p2.x;
      // point.y = p2.y;
      // point.z = op.rp.high - y;
      //标定数据
      a = a;
      r = item.k1 *
          (r - item.b0) /
          item.k0 +
          item.b1;
      p2 = Yd2DPoint(a, r).toAngular(); //极坐标转直角坐标
      point.x = p2.x;
      point.y = p2.y;
      point.z = op.rp.high - p2.x *
        tan(Yd2DPoint::toRadian(op.rp.laserPitch + op.rp.modulePitch));
    }
    else //沿边雷达
    {
      //矫正数据（计算公式可参考文档）
      float xOffset = p2.x / cos(Yd2DPoint::toRadian(op.rp.laserPitch));
      float r = sqrt(p2.y * p2.y + xOffset * xOffset);
      float a = asin(p2.y / r) - Yd2DPoint::toRadian(op.rp.modulePitch);
      // p2 = Yd2DPoint(a, r).toAngular(); //极坐标转直角坐标
      // point.x = p2.x;
      // point.y = p2.y;
      // point.z = .0;
      //标定数据
      a = a;
      r = item.k1 *
          (r - item.b0) /
          item.k0 +
          item.b1;
      p2 = Yd2DPoint(a, r).toAngular(); //极坐标转直角坐标
      point.x = p2.x;
      point.y = p2.y;
      point.z = .0;
    }
  }

  return true;
}