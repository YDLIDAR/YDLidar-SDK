#include <map>
#include <math.h>
#include "core/math/angles.h"
#include "StrongLightFilter.h"

#define MAX_DIST 0.1f
#define MIN_NOISE 1
#define MIN_VALUE 1e-8
#define IS_ZERO(d) (abs(d) < MIN_VALUE)
#define IS_EQUAL(d1, d2) IS_ZERO(d1 - d2)


StrongLightFilter::StrongLightFilter()
{
}

StrongLightFilter::~StrongLightFilter()
{
}

void StrongLightFilter::filter(
    const LaserScan &in, 
    int lidarType,
    int version,
    LaserScan &out)
{
    //转
    int size = in.points.size(); //点数
    //按角度排序
    std::map<float, int> ais;
    for (int i=0; i<size; ++i)
    {
        const LaserPoint& p = in.points.at(i);
        //过滤无效点
        if (p.range < in.config.min_range &&
            p.range > in.config.max_range)
            continue;
        if (IS_ZERO(p.range))
            continue;
        ais[p.angle] = i;
    }

    // printf("按角度排序从[%d]点到[%d]点\n", size, ais.size());

    size = ais.size(); //更新点数（过滤无效点后的点数）
    out = in;
    out.points.resize(size); //更新点数
    std::map<float, int>::iterator it;
    int i = 0;
    for (it = ais.begin(); it != ais.end(); ++it)
    {
        out.points[i++] = in.points.at(it->second);
    }
    //初始化变量
    std::vector<bool> noises(size, false); //是否为噪点的标记
    //将遍历范围扩大到原数组的104%以便处理首尾部分的点
    int sizeEx = int(size * 1.04);
    int startI = -1; //标记拖尾起始点下标位置
    LaserPoint lastP; //上一个点信息

    //主循环函数
    for (int i = 0; i < sizeEx; ++i)
    {
        const LaserPoint& p = out.points.at(i % size);
        if (i != 0)
        {
            //计算两点连线到原点的距离（直角坐标系）
            Point p1 = Point::polar2Angular(Point(p.angle, p.range));
            Point p2 = Point::polar2Angular(Point(lastP.angle, lastP.range));
            float d = Point::calcDist(Point(0, 0), p1, p2);

            // printf("点[%d]距离[%.03f]\n", i % size, d);
            //如果当前距离小于标准则认为是拖尾点
            if (d < MAX_DIST)
            {
                //如果起始点无效则标记
                if (-1 == startI)
                    startI = i;
            }
            //如果点的距离是在增加的，则当前距离小于2倍标准也认为是拖尾点
            else if (-1 != startI &&
                p.range > lastP.range &&
                d < MAX_DIST * 2)
            {
                //无处理
            }
            else
            {
                //判断统计位置是否有效，如果有效需要标记
                if (-1 != startI &&
                    i - startI >= MIN_NOISE)
                {
                    for (int j=startI; j<=i; ++j)
                    {
                        noises[j % size] = true;
                        const LaserPoint& pp = out.points.at(j % size);
                        // printf("噪点[%d] a[%.02f] r[%.02f]\n",
                        //     j % size, ydlidar::core::math::to_degrees(pp.angle), pp.range);
                    }
                }

                startI = -1;
            }
        }
        lastP = p;
    }

    //处理被标记的点
    int noiseCount = 0;
    for (int i = 0; i < size; ++i)
    {
        if (noises[i])
        {
            out.points[i].range = 0.0f;
            noiseCount ++;
        }
    }

    // printf("强光过滤噪点数[%d]\n", noiseCount);
}

StrongLightFilter::Point::Point(float x, float y)
    : x(x),
      y(y)
{
}

StrongLightFilter::Point StrongLightFilter::Point::angular2Polar(
        const StrongLightFilter::Point &p)
{
    //1.极坐标系中的两个坐标 r 和 θ 可以由下面的公式转换为直角坐标系下的坐标值x = r*cos（θ），y = r*sin（θ）。
    //2.由上述二公式，可得到从直角坐标系中x和y两坐标如何计算出极坐标下的坐标，r = sqrt(x^2 + y^2),θ = arctan(y/x)
//        float r = qSqrt(x * x + y * y);
//        float theta = qAtan(y / x);

    //将弧度值从[-M_PI/2,M_PI/2]转成[0, 2*M_PI]
    float theta = .0;
    if (!IS_ZERO(p.x)) //x不为0时
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
    //计算点到直线的最近距离
    //输入点P(x0,y0)和直线AB（x1,y1,x2,y2）,输出点到直线的最近距离
    //# 如果两点相同，则输出一个点的坐标为垂足
    //if x1 == x2 and y1 == y2:
    //    return math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0))
    if (IS_EQUAL(p1.x, p2.x) &&
        IS_EQUAL(p1.y, p2.y))
        return sqrt((p1.x - p.x) * (p1.x - p.x) +
            (p1.y - p.y) * (p1.y - p.y));

    //# 根据向量外积计算面积
    //s = (x0 - x1) * (y2 - y1) - (y0 - y1) * (x2 - x1)
    float s = (p.x - p1.x) * (p2.y - p1.y) -
            (p.y - p1.y) * (p2.x - p1.x);
    //# 计算直线上两点之间的距离
    //d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    float d = sqrt((p2.x - p1.x) * (p2.x - p1.x) +
                    (p2.y - p1.y) * (p2.y - p1.y));

    //return math.fabs(s / d)
    return fabs(s / d);
}
