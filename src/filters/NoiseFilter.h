#ifndef NOISEFILTER_H
#define NOISEFILTER_H
#include "FilterInterface.h"

#define MAX_INCLUDE_ANGLE 12.0f //最大夹角
#define MAX_INCLINE_ANGLE 7.0f //最大倾斜角
#define MIN_NOISEPOINT_COUNT 2 //最小噪点数


struct FilterBlock
{
    int start_index;
    int end_index;
};

class YDLIDAR_API NoiseFilter : public FilterInterface
{
public:
    enum FilterStrategy
    {
        FS_Normal, //滤噪
        FS_Tail, //旧拖尾滤波
        FS_TailStrong, //拖尾滤波
        FS_TailWeek, //拖尾滤波
        FS_TailStrong2, //拖尾滤波
    };
public:
    NoiseFilter();
    ~NoiseFilter() override;
    void filter(const LaserScan &in,
                 int lidarType,
                 int version,
                 LaserScan &out) override;

    std::string version() const;
    void setStrategy(int value) override;

protected:
    void filter_noise(const LaserScan &in,
                      LaserScan &out);
    //过滤拖尾方式1
    void filter_tail(const LaserScan &in,
                     LaserScan &out);
    //过滤拖尾方式2
    void filter_tail2(const LaserScan &in,
                      LaserScan &out);

    double calcInclineAngle(double reading1, double reading2,
                            double angleBetweenReadings) const;
    /**
   * @brief getTargtAngle
   * @param reading1
   * @param angle1
   * @param reading2
   * @param angle2
   * @return
   */
    double calcTargetAngle(double reading1, double angle1,
                           double reading2, double angle2);

    /**
   * @brief calculateTargetOffset
   * @param reading1
   * @param angle1
   * @param reading2
   * @param angle2
   * @return
   */
    double calcTargetOffset(double reading1, double angle1,
                            double reading2, double angle2);

    /**
   * @brief isRangeValid
   * @param reading
   * @return
   */
    bool isRangeValid(const LaserConfig &config, double reading) const;

    /**
   * @brief isIncreasing
   * @param value
   * @return
   */
    bool isIncreasing(double value) const;

    /**
   * Defines how many readings next to an invalid reading get marked as invalid
   * */

protected:
    double minIncline, maxIncline;
    int nonMaskedNeighbours;
    int maskedNeighbours;
    bool m_Monotonous;
    bool maskedFilter;

    float maxIncludeAngle = MAX_INCLUDE_ANGLE;
    float maxInclineAngle = MAX_INCLINE_ANGLE;
};

#endif // NOISEFILTER_H
