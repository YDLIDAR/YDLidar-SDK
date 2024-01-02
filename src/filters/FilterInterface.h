#ifndef FILTERINTERFACE_H
#define FILTERINTERFACE_H
#include <string>
#include "core/common/ydlidar_protocol.h"


class FilterInterface
{
public:
    FilterInterface() {}
    virtual ~FilterInterface() {}
    
    virtual void filter(const LaserScan &in,
                         int lidarType,
                         int version,
                         LaserScan &out) = 0;
    virtual std::string name() const {
        return m_name;
    }
    virtual std::string version() const {
        return "V1.0";
    }
    virtual void setName(const std::string &name) {
        m_name = name;
    }
    virtual void setStrategy(int value) {
        m_strategy = value;
    }

protected:
    int m_strategy = 0;
    std::string m_name;
};

#endif // FILTERINTERFACE_H
