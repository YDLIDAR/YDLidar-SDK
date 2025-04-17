#include <map>
#include <math.h>
#include "core/math/angles.h"
#include "StrongLightFilter.h"

#define MIN_VALUE 1e-8
#define IS_ZERO(v) (abs(v) < MIN_VALUE)
#define IS_EQUAL(v1, v2) IS_ZERO(v1 - v2)

StrongLightFilter::StrongLightFilter()
{
}

StrongLightFilter::~StrongLightFilter()
{
}

void StrongLightFilter::filter(
    const LaserScan &ins,
    int lidarType,
    int version,
    LaserScan &out)
{
    const LaserScan in = ins; //复制
    if (FS_1 == m_strategy)
        filter1(in, out);
    else //if (FS_2 == m_strategy)
        filter2(in, out);
}

bool StrongLightFilter::filter1(
    const LaserScan &in,
    LaserScan &out)
{
    int size = in.points.size(); // 点数
    // 按角度排序
    std::map<float, int> ais;
    for (int i = 0; i < size; ++i)
    {
        const LaserPoint &p = in.points.at(i);
        if (IS_ZERO(p.range))
            continue;
        ais[p.angle] = i;
    }
    // printf("按角度排序从[%d]点到[%d]点\n", size, ais.size());

    size = ais.size(); // 更新点数（过滤无效点后的点数）
    out = in;
    out.points.resize(size); // 更新点数
    std::map<float, int>::iterator it;
    int i = 0;
    for (it = ais.begin(); it != ais.end(); ++it)
    {
        out.points[i++] = in.points.at(it->second);
    }
    // 初始化变量
    std::vector<bool> noises(size, false); // 是否为噪点的标记
    // 将遍历范围扩大到原数组的104%以便处理首尾部分的点
    int sizeEx = int(size * 1.04);
    int startI = -1;  // 标记拖尾起始点下标位置
    LaserPoint lastP; // 上一个点信息

    // 主循环函数
    for (int i = 0; i < sizeEx; ++i)
    {
        const LaserPoint &p = out.points.at(i % size);
        if (i != 0)
        {
            
            Point p1 = Point::polar2Angular(Point(p.angle, p.range));
            Point p2 = Point::polar2Angular(Point(lastP.angle, lastP.range));
            Point p3 = Point(0, 0); //原点
            Point p4 = Point((p1.x + p2.x) / 2.0f, (p1.y + p2.y) / 2.0f); //两点中点
            // 计算两点连线到原点的距离（直角坐标系）
            float d = Point::calcDist(p3, p1, p2);
            //计算两线段组成直线的夹角（直角坐标系）
            float a = Point::calcAngle(p1, p2, p3, p4);

            // printf("点[%d]距离[%.03f]\n", i % size, d);
            // 如果当前距离小于标准，且角度小于标准，则认为是拖尾点
            if (d < maxDist && a < maxAngle)
            {
                // 如果起始点无效则标记
                if (-1 == startI)
                    startI = i;
            }
            // 如果点的距离是在增加的，且当前距离小于2倍标准，且角度小于标准，则认为是拖尾点
            else if (-1 != startI &&
                p.range > lastP.range &&
                d < maxDist * 2 && 
                a < maxAngle)
            {
                // 无处理
            }
            else
            {
                // 判断统计位置是否有效，如果有效需要标记
                if (-1 != startI &&
                    i - startI >= minNoise)
                {
                    for (int j = startI; j <= i; ++j)
                    {
                        noises[j % size] = true;
                        const LaserPoint &pp = out.points.at(j % size);
                        // printf("噪点[%d] a[%.02f] r[%.02f]\n",
                        //     j % size, ydlidar::core::math::to_degrees(pp.angle), pp.range);
                    }
                }

                startI = -1;
            }
        }
        lastP = p;
    }

    // 处理被标记的点
    int noiseCount = 0;
    for (int i = 0; i < size; ++i)
    {
        if (noises[i])
        {
            out.points[i].range = 0.0f;
            noiseCount++;
        }
    }

    // printf("强光过滤噪点数[%d]\n", noiseCount);
    return noiseCount;
}

bool StrongLightFilter::filter2(
    const LaserScan &in,
    LaserScan &out)
{
    int size = in.points.size(); //点数
    // 按角度排序
    std::map<float, int> ais;
    for (int i = 0; i < size; ++i)
    {
        const LaserPoint &p = in.points.at(i);
        if (IS_ZERO(p.range))
            continue;
        ais[p.angle] = i;
    }
    // printf("按角度排序从[%d]点到[%d]点\n", size, ais.size());

    size = ais.size(); // 更新点数（过滤无效点后的点数）
    out = in;
    out.points.resize(size); // 更新点数
    std::map<float, int>::iterator it;
    int i = 0;
    for (it = ais.begin(); it != ais.end(); ++it)
    {
        out.points[i++] = in.points.at(it->second);
    }
    // 初始化变量
    std::vector<bool> noises(size, false); // 是否为噪点的标记

    // 主循环函数
    for (int i = 0; i < size; ++i)
    {
        const LaserPoint &p1 = out.points.at(i);
        const LaserPoint &p2 = out.points.at((i + 1) % size);
        //计算两点连线在x，y轴的截距（直角坐标系）
        Point a1 = Point::polar2Angular(Point(p1.angle, p1.range));
        Point a2 = Point::polar2Angular(Point(p2.angle, p2.range));
        float x = abs((a1.y * a2.x - a2.y * a1.x) / (a2.x - a1.x));
        float y = abs((a1.y * a2.x - a2.y * a1.x) / (a2.y - a1.y));
        float min = std::min(x, y);
        //如果最小截距小于距离阈值，则认为是噪点
        if (min < maxDist)
        {
            noises[i] = true;
            noises[(i + 1) % size] = true;
        }
    }

    // 处理被标记的点
    int noiseCount = 0;
    for (int i = 0; i < size; ++i)
    {
        if (noises[i])
        {
            out.points[i].range = 0.0f;
            noiseCount++;
        }
    }

    // printf("强光过滤噪点数[%d]\n", noiseCount);
    return noiseCount;
}

StrongLightFilter::Point::Point(float x, float y)
    : x(x),
      y(y)
{
}

StrongLightFilter::Point StrongLightFilter::Point::angular2Polar(
    const StrongLightFilter::Point &p)
{
    // 1.极坐标系中的两个坐标 r 和 θ 可以由下面的公式转换为直角坐标系下的坐标值x = r*cos（θ），y = r*sin（θ）。
    // 2.由上述二公式，可得到从直角坐标系中x和y两坐标如何计算出极坐标下的坐标，r = sqrt(x^2 + y^2),θ = arctan(y/x)
    //        float r = qSqrt(x * x + y * y);
    //        float theta = qAtan(y / x);

    // 将弧度值从[-M_PI/2,M_PI/2]转成[0, 2*M_PI]
    float theta = .0;
    if (!IS_ZERO(p.x)) // x不为0时
    {
        theta = atan(p.y / p.x);
        if (p.x > 0.0)
        {
            if (p.y < 0.0)
                theta += (M_PI * 2.0);
        }
        else
        {
            theta += M_PI;
        }
    }
    return Point(theta, sqrt(p.x * p.x + p.y * p.y));
}

StrongLightFilter::Point StrongLightFilter::Point::polar2Angular(
    const StrongLightFilter::Point &p)
{
    return Point(p.y * cos(p.x), p.y * sin(p.x));
}

float StrongLightFilter::Point::calcDist(
    const StrongLightFilter::Point &p,
    const StrongLightFilter::Point &p1,
    const StrongLightFilter::Point &p2)
{
    // 计算点到直线的最近距离
    // 输入点P(x0,y0)和直线AB（x1,y1,x2,y2）,输出点到直线的最近距离
    // # 如果两点相同，则输出一个点的坐标为垂足
    // if x1 == x2 and y1 == y2:
    //     return math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0))
    if (IS_EQUAL(p1.x, p2.x) &&
        IS_EQUAL(p1.y, p2.y))
        return sqrt((p1.x - p.x) * (p1.x - p.x) +
                    (p1.y - p.y) * (p1.y - p.y));

    // # 根据向量外积计算面积
    // s = (x0 - x1) * (y2 - y1) - (y0 - y1) * (x2 - x1)
    float s = (p.x - p1.x) * (p2.y - p1.y) -
              (p.y - p1.y) * (p2.x - p1.x);
    // # 计算直线上两点之间的距离
    // d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    float d = sqrt((p2.x - p1.x) * (p2.x - p1.x) +
                   (p2.y - p1.y) * (p2.y - p1.y));

    // return math.fabs(s / d)
    return fabs(s / d);
}

float StrongLightFilter::Point::calcLen(
    const StrongLightFilter::Point &v)
{
    return sqrt(v.x * v.x + v.y * v.y);
}

float StrongLightFilter::Point::calcDot(
    const StrongLightFilter::Point &v1,
    const StrongLightFilter::Point &v2)
{
    return v1.x * v2.x + v1.y * v2.y;
}

float StrongLightFilter::Point::calcAngle(
    const StrongLightFilter::Point &p1,
    const StrongLightFilter::Point &p2,
    const StrongLightFilter::Point &p3,
    const StrongLightFilter::Point &p4)
{
    Point v1(p2.x - p1.x, p2.y - p1.y); //向量1
    Point v2(p4.x - p3.x, p4.y - p3.y); //向量2
    //计算两向量夹角（锐角）
    float theta = calcDot(v1, v2) / (calcLen(v1) * calcLen(v2));
    float a = acos(theta) * 180.0f / M_PI; 
    if (a < .0f)
        a = -a;
    if (a > 90.0f)
        a = 180.0f - a;
    return a;
}
