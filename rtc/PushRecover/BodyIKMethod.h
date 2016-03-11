#ifndef __BODYIKMETHOD_H__
#define __BODYIKMETHOD_H__
#include <reactive_walk_generator.h>
#include <boost/static_assert.hpp>

/* from control/robot_parameter.h */
/// 左脚先座標の初期値
const static Vec3 InitialLfoot_p(-0.015,  0.08, 0.1045);
/// 右脚先座標の初期値
const static Vec3 InitialRfoot_p(-0.015,  -0.08, 0.1045);

#if 1
/* default joint angle for Zc = 0.578752 */
const static float Zc = 0.578752;
const float default_joint_angle[12] = {-5.751865e-06, -0.027068, -39.6337, 72.5952, -32.9614, 0.027092,
                                       5.686155e-06,  -0.026751, -39.6337, 72.5952, -32.9614, 0.026727
};
#else
/* default joint angle for Zc = 0.620 */
const static float Zc = 0.620;
const float default_joint_angle[12] = {0.0,  -4.968613e-06,  -29.0694,  61.4722,  -32.4028,  4.968613e-06,
                                       0.0,  -4.140511e-06,  -29.0694,  61.4722,  -32.4028,  4.140511e-06};
#endif

const static Vec3 traj_body_init( -0.0586f, 0.0f, Zc - InitialLfoot_p[2]);
//const static Vec3 traj_body_init( -0.0586f, 0.0f, Zc);
//const static Vec3 traj_body_init( 0.0f, 0.0f, 0.578752f );


inline float rad2deg(const float a){
    return a * (180.0f / Q_PI);
}
inline float deg2rad(const float a){
    return a * (Q_PI / 180.0f);
}

/* IIKMethod, BodyIKMethod LinkIKParam are from ikfk_controller.h.cpp */
struct LegIKParam{
        static const int D = 80*1000; // um
        static const int A = 300*1000; // um
        static const int B = 300*1000; // um
        static const int d = 104500; // um
        static const int sensorZ = (int)((39.5E-3-12.5E-3)*1.0E6); // um センサ高さ
};

class IIKMethod : public Aligned<16>
{
public:
        /// @return body_pos
        virtual Vec3 calcik( const Mat3& body_R, const Vec3& ref_pos, const Vec3& footl_pos, const Vec3& footr_pos, const float footl_pitch, const float footr_pitch, float* joint_angle ) = 0;
        /// @return ref_pos
        virtual Vec3 calcref( JointTransform& pR ) = 0;
        virtual ~IIKMethod(){}
};

class BodyIKMethod : public IIKMethod
{
        /// Rootリンク座標の初期値
        const Vec3 InitialBody_p;
public:
        BodyIKMethod( const float body_x, const float body_z )
                : InitialBody_p( body_x, 0, body_z )
        {
                BOOST_STATIC_ASSERT( __alignof__(InitialBody_p) == 16 );
        }
        virtual Vec3 calcik( const Mat3& body_R, const Vec3& ref_pos, const Vec3& footl_pos, const Vec3& footr_pos, const float footl_pitch, const float footr_pitch, float* joint_angle )
        {
                Mat3 footl_R, footr_R;
                Mat3::identity().rotate<1>( cosf( footl_pitch ), sinf( footl_pitch ), footl_R );
                Mat3::identity().rotate<1>( cosf( footr_pitch ), sinf( footr_pitch ), footr_R );
                Vec3 body_pos = InitialBody_p + ref_pos;
                ik_legLR<LegIKParam>(
                                     body_R, body_pos,
                                     footl_R, footl_pos,
                                     footr_R, footr_pos,
                                     joint_angle );
                return body_pos;
        }
        virtual Vec3 calcref( JointTransform& pR )
        {
                return pR.p[JOINT_ROOT];
        }
};

#endif /* __BODYIKMETHOD_H_ */
