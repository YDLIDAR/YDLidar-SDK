#ifndef STRONGLIGHTFILTER_H
#define STRONGLIGHTFILTER_H
#include "FilterInterface.h"

//强光滤波器
class StrongLightFilter : public FilterInterface
{
public:
    StrongLightFilter();
    virtual ~StrongLightFilter();
    
    virtual void filter(const LaserScan &in,
                int lidarType,
                int version,
                LaserScan &out);

protected:
    struct Point
    {
        float x = .0;
        float y = .0;

        Point(float x = .0, float y = .0);

        static Point angular2Polar(const Point &p); // 直角坐标转极坐标
        static Point polar2Angular(const Point &p); // 极坐标转直角坐标
        // 计算直角坐标系中点到直线的距离
        static float calcDist(
            const Point &p,
            const Point &p1,
            const Point &p2);
    };
};

#endif // STRONGLIGHTFILTER_H
