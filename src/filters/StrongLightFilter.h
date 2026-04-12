#ifndef STRONGLIGHTFILTER_H
#define STRONGLIGHTFILTER_H
#include "FilterInterface.h"


//强光滤波器（拖尾滤波器）
class YDLIDAR_API StrongLightFilter : public FilterInterface
{
public:
    enum FilterStrategy //方法
    {
        FS_1, //方法1，角度距离法
        FS_2, //方法2，截距法
    };
public:
    StrongLightFilter();
    virtual ~StrongLightFilter();
    
    virtual void filter(const LaserScan &in,
                int lidarType,
                int version,
                LaserScan &out);
    void setMaxDist(float dist) {maxDist = dist;}
    void setMaxAngle(float angle) {maxAngle = angle;}
    void setMinNoise(int noise) {minNoise = noise;}
    void setStrategy(int s) {m_strategy = s;}

protected:
    bool filter1(const LaserScan &in,
                LaserScan &out);
    bool filter2(const LaserScan &in,
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
        //计算向量的长度
        static float calcLen(
            const Point &v);
        //计算向量的乘积
        static float calcDot(
            const Point &v1,
            const Point &v2);
        //计算直角坐标系中两线段组成直线的夹角
        static float calcAngle(
            const Point &p1,
            const Point &p2,
            const Point &p3,
            const Point &p4);
    };

    //过滤方法
    int m_strategy = FS_2;
    float maxDist = 0.05; //最大距离阈值，单位米（此值可根据需要自己修改）
    float maxAngle = 12.0; //最大角度阈值，单位°（此值可根据需要自己修改）
    int minNoise = 2; //最小连续噪点数（此值可根据需要自己修改）
};

#endif // STRONGLIGHTFILTER_H
