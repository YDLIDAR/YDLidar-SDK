%module ydlidar
%include "std_string.i"
%include "std_vector.i"
%include "std_map.i"
%include "typemaps.i"
%include "stdint.i"
%include "cpointer.i"

%{
#define SWIG_FILE_WITH_INIT
#define SWIG_PYTHON_EXTRA_NATIVE_CONTAINERS
#include "../src/CYdLidar.h"
#include "../core/base/typedef.h"
#include "../core/common/ydlidar_datatype.h"
#include "../core/common/ydlidar_def.h"
%}

%define YDLIDAR_API
%enddef

namespace std {
%template(PointVector) vector<LaserPoint>;
%template(Str2strMap) map<string, string>;
}



%extend YDLIDAR_API CYdLidar {
public:
  bool setlidaropt(int optname, int value) {
    return $self->setlidaropt(optname, (const void *)(&value), sizeof(int));
  }
  bool setlidaropt(int optname, float value) {
    return $self->setlidaropt(optname, (const void *)(&value), sizeof(float));
  }
  bool setlidaropt(int optname, bool value) {
    return $self->setlidaropt(optname, (const void *)(&value), sizeof(bool));
  }
  bool setlidaropt(int optname, std::string value) {
    return $self->setlidaropt(optname, (const void *)(value.c_str()),value.size());
  }
  bool getlidaropt_toInt(int optname, int *optval) {
    return $self->getlidaropt(optname, (void *)(optval),sizeof(int));
  }
  bool getlidaropt_toBool(int optname, bool *optval) {
    return $self->getlidaropt(optname, (void *)(optval),sizeof(bool));
  }
  bool getlidaropt_toFloat(int optname, float *optval) {
    return $self->getlidaropt(optname, (void *)(optval),sizeof(float));
  }
  bool getlidaropt_toString(int optname, std::string *optval) {
    char value[200];
    bool ret = $self->getlidaropt(optname, (void *)(value),200);
    *optval = value;
    return ret;
  }
}

%apply int *OUTPUT { int *optval};
%apply float *OUTPUT { float *optval};
%apply bool *OUTPUT { bool *optval};
%apply std::string *OUTPUT { std::string *optval};
%ignore setlidaropt(int optname, const void *optval, int optlen);
%ignore getlidaropt(int optname, void *optval, int optlen);


%typemap(out) uint64_t {
  $result = PyLong_FromUnsignedLongLong($1);
}

%typemap(in) uint64_t {
  $1 = PyLong_AsUnsignedLongLong($input);
}

%typemap(out) uint32_t {
  $result = PyLong_FromUnsignedLong($1);
}

%typemap(in) uint32_t {
  $1 = PyLong_AsUnsignedLong($input);
}

%include "../src/CYdLidar.h"
%include "../core/base/typedef.h"
%include "../core/common/ydlidar_datatype.h"
%include "../core/common/ydlidar_def.h"

