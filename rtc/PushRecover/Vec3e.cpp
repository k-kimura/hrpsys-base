// -*- tab-width : 4 ; mode : C++ ; indent-tabs-mode : nil -*-
#include "Vec3e.h"

#if 0
Vec3e::operator hrp::Vector3e() {
    return hrp::Vector3e((double)vec[0],(double)vec[1],(double)vec[2]);
};
#endif

namespace hrp
{
#if defined(__INTEL_COMPILER)||defined(__ICC)
    Vector3e::operator Vec3e() {
        return Vec3e((float)x(),(float)y(),(float)z());
    };
#elif defined(__GNUC__)
    Vector3e::operator Vec3e() {
        return Vec3e((float)x(),(float)y(),(float)z());
    };
#else
#error "Vec3e Error"
#endif
}


