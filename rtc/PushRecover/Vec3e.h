// -*- tab-width : 4 ; mode : C++ ; indent-tabs-mode : nil -*-
#ifndef _VEC3E_H_
#define _VEC3E_H_

#include <reactive_walk_generator.h>
#include <hrpUtil/EigenTypes.h>

/* Eigen typeのベクトルhrp::Vector3と intrinsic typeのベクトルVec3=QzVec3=F32Vecとの間の相互変換を提供するための拡張型ベクトルクラス */

namespace hrp {
    class Vector3e;
}

class Vec3e;

namespace hrp
{
    class Vector3e : public Vector3 {
    public:
        Vector3e() : Vector3(0.0f, 0.0f, 0.0f) {
        };
        Vector3e(Vector3 v) : Vector3(v) {
        };
        Vector3e(Vec3e v);
        Vector3e(double x, double y, double z) : Vector3(x,y,z) {
        };
        operator Vec3e();
        template<typename U>
        Vector3e& operator=(const U &v){
            this->x() = v[0];
            this->y() = v[1];
            this->z() = v[2];
            return *this;
        };
    };
}

#if defined(__INTEL_COMPILER)||defined(__ICC)
class Vec3e : public Vec3 {
public:
    Vec3e() : Vec3(0.0f, 0.0f, 0.0f) {
    };
    Vec3e(Vec3 v) : Vec3(v) {
    };
    Vec3e(float x, float y, float z) : Vec3(x,y,z) {
    };
    operator hrp::Vector3e();
    template<typename U>
    Vec3e& operator=(const U &v){
        this->vec    = _mm_set_ps(0.0f, (float)v[2], (float)v[1], (float)v[0]);
        return *this;
    };
};
#elif defined(__GNUC__)
class Vec3e : public Vec3 {
public:
    Vec3e() : Vec3(0.0f, 0.0f, 0.0f) {
    };
    Vec3e(Vec3 v) : Vec3(v) {
    };
    Vec3e(float x, float y, float z) : Vec3(x,y,z) {
    };
    Vec3e(hrp::Vector3e v) : Vec3((float)v[0],(float)v[1],(float)v[2]){
        //std::cout << "calling Vec3e(hrp::Vector3e) constructor" << std::endl;
    };
    operator hrp::Vector3e();
    template<typename U>
    Vec3e& operator=(const U &v){
        *this = Vec3e(v);
        return *this;
    };
};
#else
#error "Vec3e Error"
#endif

#endif /* _VEC3E_H_ */
