#include "Vec3e.h"

#if 0
Vec3e::operator hrp::Vector3e() {
    return hrp::Vector3e((double)vec[0],(double)vec[1],(double)vec[2]);
};
#endif

namespace hrp
{
    Vector3e::operator Vec3e() {
        return Vec3e((float)x(),(float)y(),(float)z());
    };
}


