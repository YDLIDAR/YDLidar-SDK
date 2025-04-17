#include <math.h>
#include "NoiseFilter.h"
#include "math/angles.h"

NoiseFilter::NoiseFilter()
    : minIncline(0.11),
      maxIncline(3.0),
      nonMaskedNeighbours(4),
      maskedNeighbours(2),
      m_Monotonous(false),
      maskedFilter(true)
{
    m_name = "NoiseFilter";
    setStrategy(FS_Normal);
}

NoiseFilter::~NoiseFilter()
{
}

//计算两点连线和原点的夹角（弧度）
double NoiseFilter::calcInclineAngle(
        double reading1,
        double reading2,
        double angleBetweenReadings) const
{
    return atan2(sin(angleBetweenReadings) * reading2,
                 reading1 - (cos(angleBetweenReadings) * reading2));
}

//计算两点的倾斜角（弧度）
double NoiseFilter::calcTargetAngle(
        double reading1,
        double angle1,
        double reading2,
        double angle2)
{
    double reading1_x = reading1 * cos(angle1);
    double reading1_y = reading1 * sin(angle1);
    double reading2_x = reading2 * cos(angle2);
    double reading2_y = reading2 * sin(angle2);
    double dx = reading2_x - reading1_x;
    double dy = reading2_y - reading1_y;
    return atan2(dy, dx);
}

double NoiseFilter::calcTargetOffset(
        double reading1,
        double angle1,
        double reading2,
        double angle2)
{
    double target_angle = calcTargetAngle(reading1, angle1, reading2, angle2);
    double cos_inv_angle = cos(-target_angle);
    double sin_inv_angle = sin(-target_angle);
    double reading2_x = reading2 * cos(angle2);
    double reading2_y = reading2 * sin(angle2);
    double offset = reading2_x * sin_inv_angle + reading2_y * cos_inv_angle;
    return offset;
}

bool NoiseFilter::isRangeValid(const LaserConfig &config,
                               double reading) const {
    if (reading >= config.min_range && reading <= config.max_range) {
        return true;
    }

    return false;
}

bool NoiseFilter::isIncreasing(double value) const {
    if (value > 0) {
        return true;
    }

    return false;
}

void NoiseFilter::filter(
        const LaserScan &ins,
        int lidarType,
        int version,
        LaserScan &out)
{
    const LaserScan in = ins; //复制
    if (FS_Normal == m_strategy)
    {
        filter_noise(in, out);
    }
    else if (FS_Tail == m_strategy)
    {
        filter_tail(in, out);
    }
    else
    {
        filter_tail2(in, out);
    }
}

void NoiseFilter::filter_noise(
        const LaserScan &in,
        LaserScan &out)
{
    //range is empty
    if (in.points.empty()) {
        out = in;
        return;
    }

    std::vector<bool> maskedPoints;
    double lastRange = in.points[0].range;
    double lastAngle = in.points[0].angle;
    double lastInclineRange = lastRange;
    const int nrPoints = in.points.size();
    maskedPoints.resize(nrPoints, false);
    double lastIncline = 0;
    bool hasFirst = false;

    //copy attributes to filtered scan
    out = in;
    int pointCount  = 0;
    double lastOffset = 0;
    double lastDistance = lastRange;
    double preAngle = lastAngle;
    double lastDiff = 0;
    bool isIncrease = false;
    int minIndex = 0;
    double minIndexDistance = 0;
    double maxDistance = 0;
    bool isNoise = false;
    float filter_offset = 0.05;

    for (int i = 0; i < nrPoints; i++) {
        double current_range = in.points[i].range;//current lidar distance
        double current_angle = in.points[i].angle;//current lidar angle

        if (isRangeValid(in.config, current_range)) {

            double offset;

            if (isRangeValid(in.config, lastRange)) {
                offset = calcTargetOffset(lastRange, lastAngle, current_range,
                                          current_angle);//calculate offset distance
            } else {
                offset = calcTargetOffset(lastDistance, preAngle, current_range,
                                          current_angle);//calculate offset distance
            }

            double Diff = current_range - lastDistance;//distance difference

            if (fabs(Diff) > lastDistance * 0.2 && fabs(offset) < 0.2 &&
                    isRangeValid(in.config, lastDistance)) {
                isNoise = true;
                filter_offset = fabs(offset) + 0.05;
                maxDistance = current_range;
                maskedPoints[i] = true;
            }

            if (isNoise && fabs(offset) > filter_offset) {
                isNoise = false;
            }

            if (isNoise) {
                if (pointCount == 0) {
                    minIndex = i;
                    minIndexDistance = current_range;
                }

                if (current_range > maxDistance) {
                    maxDistance = current_range;
                }

                pointCount++;

                //        if (pointCount >= 2) {
                //          for (int j = minIndex - nonMaskedNeighbours; j <= i + nonMaskedNeighbours;
                //               j++) {
                //            if (j >= 0 && j < nrPoints) {
                //              if (j < minIndex || j > i) {
                //                double offset = calculateTargetOffset(in.points[j].range, in.points[j].angle,
                //                                                      current_range,
                //                                                      current_angle);//calculate offset distance

                //                if (in.points[j].range > maxDistance) {
                //                  maxDistance = in.points[j].range;
                //                }

                //                if (offset < filter_offset && maxDistance > filter_offset &&
                //                    maxDistance > 0.2) {
                //                  maskedPoints[j] = true;
                //                }
                //              } else {
                //                maskedPoints[i] = true;
                //              }
                //            }
                //          }
                //        }

            } else {
                if (pointCount >= 2) {
                    bool isincre = (in.points[minIndex].range - lastDistance) < 0;

                    for (int j = minIndex - nonMaskedNeighbours; j <= i + nonMaskedNeighbours;
                         j++) {
                        if (j >= 0 && j < nrPoints) {
                            double offset = calcTargetOffset(in.points[j].range, in.points[j].angle,
                                                             lastDistance,
                                                             preAngle);//calculate offset distance

                            if (in.points[j].range > maxDistance) {
                                maxDistance = in.points[j].range;
                            }

                            if (offset < filter_offset && maxDistance > filter_offset &&
                                    maxDistance > 0.2) {
                                maskedPoints[j] = true;
                            }
                        }
                    }
                }

                pointCount = 0;
                minIndex = 0;
                maxDistance = 0.0;
            }

            lastOffset = offset;//last offset
            lastDiff = Diff;//last distance difference
            lastDistance = current_range;//last distance
            preAngle = current_angle;
        }

        lastAngle = current_angle;//last angle
        lastRange = current_range;//last range
    }

    //mark all masked points as invalid in scan
    for (unsigned int i = 0; i < in.points.size(); i++) {
        if (maskedPoints[i]) {
            //as we don't have a better error this is an other range error for now
            out.points[i].range = 0.0;
        }
    }
}

void NoiseFilter::filter_tail(
        const LaserScan &in,
        LaserScan &out)
{
//    printf("%s\n", __FUNCTION__);

    //假设激光的原点是O，对于任何两个点P1和P2，则形成角∠OP1P2，
    //如果该角度小于最小阈值角度（min_angle）或大于最大阈值角度（max_angle），
    //我们将该点及其附近符合条件的点移除。

    //range is empty
    if (in.points.empty()) {
        out = in;
        return;
    }

    std::vector<bool> maskedPoints;
    double lastRange = in.points[0].range;
    double lastAngle = in.points[0].angle;
    double lastInclineRange = lastRange;
    const int size = in.points.size();
    maskedPoints.resize(size, false);
    double lastIncline = 0;
    bool hasFirst = false;

    //copy attributes to filtered scan
    out = in;
    double lastDistance = 0;
    int max_skip_step = 5 * maskedNeighbours;

    if (max_skip_step > 7) {
        max_skip_step = 7;
    }

    std::vector<FilterBlock> m_block_vct;
    FilterBlock m_block;
    bool isNextBlock = true;
    int inValidPointCount = 0;

    for (int i = 0; i < size; i++)
    {
        double range = in.points[i].range;//current lidar distance
        double angle = in.points[i].angle;//current lidar angle

        if (isRangeValid(in.config, range)
                /*&& isRangeValid(in.config, lastDistance)*/)
        {
            if (maskedFilter && isRangeValid(in.config, lastDistance))
            {
                const double incline = calcInclineAngle(
                            in.points[i].range,
                            lastDistance,
                            angle - lastAngle);

                if (!hasFirst) {
                    hasFirst = true;
                    lastIncline = incline;
                }

                bool isValid = false;

                //this is a filter for false readings that do occur if one scannes over edgeds of objects
                //如果计算的夹角超出规定的范围
                if (incline < minIncline || incline > maxIncline)
                {
                    //mask neighbour points
                    for (int j = -maskedNeighbours; j < maskedNeighbours; j++)
                    {
                        if ((int(i) + j < 0)
                                || ((int(i) + j) > size)) {
                            continue;
                        }
                        if (i + j - 1 < 0) {
                            continue;
                        }

                        //如果当前点相邻N点中有偏移量较大的点则认为是噪点
                        double offset = calcTargetOffset(in.points[i + j - 1].range,
                                in.points[i + j - 1].angle,
                                in.points[i + j].range,
                                in.points[i + j].angle); //calculate offset distance
                        if (offset < 0.2) {
                            maskedPoints[i + j] = true;
                            isValid = true;
                        }
                    }

                    if (isValid) {
                        if (isNextBlock) {
                            isNextBlock = false;
                            m_block.start_index = i;
                        }

                        m_block.end_index = i;
                        inValidPointCount = 0;
                    } else {
                        inValidPointCount++;
                    }
                }
                else
                {
                    inValidPointCount++;
                }

                //如果上一个夹角和当前夹角差值过大则认为是噪点
                if (fabs(lastIncline - incline) > maxIncline - minIncline) {
                    maskedPoints[i] = true;
                }

                lastInclineRange = range;
                lastIncline = incline;
            }

            lastDistance = range;//last distance
        }

        if (inValidPointCount > max_skip_step) {
            if (!isNextBlock) {
                m_block_vct.push_back(m_block);
                isNextBlock = true;
            }
        }

        lastAngle = angle;//last angle
        lastRange = range;//last range
    }

    /*for (int i = 0; i < m_block_vct.size(); i++) {
    if (m_block_vct[i].end_index - m_block_vct[i].start_index > 20) {
      int first_index = m_block_vct[i].start_index;

      if (i > 0 &&
          (m_block_vct[i - 1].end_index - m_block_vct[i - 1].start_index) > 2 *
          maskedNeighbours) {
        if (m_block_vct[i].start_index -  m_block_vct[i - 1].end_index <= 2 *
            (max_skip_step + 1)) {
          first_index = m_block_vct[i - 1].start_index;
        }
      }

      if (in.points[first_index].range > 0) {
        for (int k = first_index; k >= 0; k--) {
          if (in.points[k].range > in.points[first_index].range) {
            first_index = k;
            break;
          }
        }
      }

      for (int j = first_index - maskedNeighbours;
           j < m_block_vct[i].end_index + maskedNeighbours; j++) {
        if (j >= 0 && j < nrPoints) {
          maskedPoints[j] = true;
        }
      }
    } else {
      if (m_block_vct[i].end_index - m_block_vct[i].start_index > maskedNeighbours) {
        int first_index = m_block_vct[i].start_index - maskedNeighbours;

        for (int j = first_index;
             j < m_block_vct[i].end_index + maskedNeighbours; j++) {
          if (j >= 0 && j < nrPoints) {
            maskedPoints[j] = true;
          }
        }
      }
    }
  }*/

    //mark all masked points as invalid in scan
    for (unsigned int i = 0; i < in.points.size(); i++) {
        if (maskedPoints[i]) {
            //as we don't have a better error this is an other range error for now
            out.points[i].range = 0.0;
        }
    }

    //  for (int i = 0; i < m_block_vct.size(); i++) {
    ////    if (m_block_vct[i].end_index - m_block_vct[i].start_index > 3 *
    ////        maskedNeighbours) {

    //    printf("block[%d]: %d~%d[%f-%f]\n", i, m_block_vct[i].start_index,
    //           m_block_vct[i].end_index,
    //           in.points[m_block_vct[i].start_index].angle * 180 / M_PI,
    //           in.points[m_block_vct[i].end_index].angle * 180 / M_PI);
    ////    }
    //  }
}

void NoiseFilter::filter_tail2(
        const LaserScan &in,
        LaserScan &out)
{
    //    LOG_DEBUG("[{}] 点数[{}]",
    //              m_Name.toStdString().c_str(),
    //              in.points.size());

    out = in;

    if (in.points.empty()) {
        return;
    }

    //1、找出连续（至少3个）点倾斜角朝向原点（极点）的点序列
    //2、判断该点序列首尾点组成的角度范围是否在光斑对应角度范围内
    //3、判断该点序列的强度信息是否满足约定条件（未找到规律，暂未使用）
    //4、去掉该点序列的首尾点（首尾点是正常的）

    std::vector<bool> noises; //是否为噪点的标记
    size_t size = in.points.size(); //一圈点数
    size_t lastIndex = 0; //上一个有效点的索引位置
    LaserPoint lastP; //上一个点信息
    float lastIncline = .0; //上一个倾斜角
    float lastAngle = 90.0; //上一个夹角
    size_t pos = 0; //标记拖尾起始点下标位置
    //    bool hasNoise = false; //是否需要处理噪点的标志
    size_t sizeEx = size + (size * 2 / 100 + 1); //将遍历范围扩大到原数组的102%以便处理首尾部分的点

    noises.resize(size, false);

    //主循环函数
    for (size_t i = 0; i < sizeEx; ++i)
    {
        const LaserPoint& p = in.points.at(i % size);

        if (!isRangeValid(in.config, p.range))
        {
            continue;
        }

        if (i != 0)
        {
            //计算两点连线、两点中间点到原点连线的倾斜角（弧度值）
            float incline2 = calcTargetAngle(
                        lastP.range,
                        lastP.angle,
                        p.range,
                        p.angle);
            float incline3 = calcTargetAngle(
                        (lastP.range + p.range) / 2.0f,
                        (lastP.angle + p.angle) / 2.0f,
                        0.0f,
                        0.0f);
            //转角度值
            incline2 = ydlidar::core::math::to_degrees(incline2);
            incline3 = ydlidar::core::math::to_degrees(incline3);

            float incline = incline2;

            //计算两点连线和两点中间点到原点连线的夹角
            float angle = fabs(incline2 - incline3);
            if (angle > 180.0f)
                angle = 360.0f - angle;
            if (angle > 90.0f)
                angle = 180.0f - angle;

            //            LOG_DEBUG("i:{} l1:{} l2:{} ia:{} range:{} angle:{} intensity:{}",
            //                      i, incline2,
            //                      incline3,
            //                      angle,
            //                      p.range,
            //                      qRadiansToDegrees(p.angle),
            //                      p.intensity);

            //如果倾斜角变化很小则认为是一条直线上的
            if (fabs(incline - lastIncline) < maxInclineAngle)
            {
                //如果上一个夹角不满足要求
                if (fabs(lastAngle) >= maxIncludeAngle)
                {
                    pos = 0;
                }
                //TODO: 需要考虑是否是最后一个点
            }
            else
            {
                if (fabs(lastAngle) < maxIncludeAngle) //判断上一个夹角是否满足要求
                {
                    //判断点的个数是否超过2个，超过2个才可能是拖尾噪点
                    if (0 != pos && i - pos >= MIN_NOISEPOINT_COUNT)
                    {
                        //统计从位置pos到i的有效点数
                        size_t validCount = 0;
                        for (size_t j=pos; j<=i; ++j)
                        {
                            if (isRangeValid(in.config, in.points[i % size].range))
                                validCount += 1;
                        }
                        if (validCount >= MIN_NOISEPOINT_COUNT)
                        {
                            for (size_t j=pos; j<=i; ++j)
                            {
                                noises[j % size] = true;
                                //                                LOG_DEBUG("noise i:{}", j % size);
                            }
                        }
                    }
                    pos = 0;
                }
                if (0 == pos && fabs(angle) < maxIncludeAngle) //判断当前夹角是否满足要求
                {
                    //疑似拖尾点，标记
                    pos = lastIndex;
                }
                else
                {
                    pos = 0;
                }
            }

            lastIncline = incline;
            lastAngle = angle;
        }

        lastIndex = i;
        lastP = p;
    }

    //处理被标记的点
    size_t noiseCount = 0;
    for (size_t i = 0; i < size; ++i)
    {
        if (noises[i])
        {
            out.points[i].range = 0.0f;
            noiseCount ++;
        }
    }
    printf("Noise count %lu\n", noiseCount);
}

void NoiseFilter::setStrategy(int value)
{
    FilterInterface::setStrategy(value);

    if (m_strategy == FS_TailWeek)
    {
        maxIncludeAngle /= 3.0f;
        maxInclineAngle /= 3.0f;
    }
    else if (m_strategy == FS_TailStrong2)
    {
        maxIncludeAngle *= 1.5f;
        maxInclineAngle *= 1.5f;
    }
}

std::string NoiseFilter::version() const
{
    return "1.0.1";
}
