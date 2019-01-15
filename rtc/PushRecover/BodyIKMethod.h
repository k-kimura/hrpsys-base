// -*- tab-width : 4 ; mode : C++ ; indent-tabs-mode : nil -*-
#ifndef __BODYIKMETHOD_H__
#define __BODYIKMETHOD_H__
#define eval_dT (0.005)
#define gen_dT (0.001)
#if defined(ROBOT)
#if   ROBOT==0
//#define gen_Zc (0.620)
#define gen_Zc (0.560)
#elif ROBOT==1
//#define gen_Zc (0.800)
#define gen_Zc (0.760)
//#define gen_Zc (0.700)
#elif ROBOT==2
// for L1W
#define gen_Zc (0.760)
#else
#error "Undefined ROBOT TYPE."
#endif
#else
#error "Definition of ROBOT is unavailable."
#endif /* if defined(ROBOT)*/

#include "link_kinematics.h"
#include <reactive_walk_generator.h>
#include <boost/static_assert.hpp>

/* from control/robot_parameter.h */
#if defined(ROBOT)
#if ROBOT==0
#if 1
// TODO : Use IKParam.InitialLfoot_p;
/// 左脚先座標の初期値
//const static Vec3 InitialLfoot_p(-0.015,  0.10, 0.1045);
/// 右脚先座標の初期値
//const static Vec3 InitialRfoot_p(-0.015,  -0.10, 0.1045);
#else
/// 左脚先座標の初期値
//const static Vec3 InitialLfoot_p(-0.015,  0.095, 0.1045);
/// 右脚先座標の初期値
//const static Vec3 InitialRfoot_p(-0.015,  -0.095, 0.1045);
#endif
#elif ROBOT==1
#if 1
// TODO : Use IKParam.InitialLfoot_p;
/// 左脚先座標の初期値
//const static Vec3 InitialLfoot_p(0.0,   0.120, 0.1335);
/// 右脚先座標の初期値
//const static Vec3 InitialRfoot_p(0.0,  -0.120, 0.1335);
#else
/// 左脚先座標の初期値
//const static Vec3 InitialLfoot_p(0.0,   0.100, 0.1335);
/// 右脚先座標の初期値
//const static Vec3 InitialRfoot_p(0.0,  -0.100, 0.1335);
#endif
#elif ROBOT==2
// nothing to do
#else
#error "Undefined ROBOT TYPE."
#endif
#else
#error "Definition of ROBOT is unavailable."
#endif /* if defined(ROBOT)*/


/* from control/robot_parameter.h */
#if defined(ROBOT)
#if ROBOT==0
/* default joint angle for Zc = 0.620 */
//const static Vec3 g_CoG_offset( 0.00460f, 0.0f, 0.000f ); //rootPosから見たCoG位置
//const static Vec3 g_CoG_offset( 0.0460f, 0.0f, 0.1112f ); //rootPosから見たCoG位置
const static Vec3 g_CoG_offset( 0.0f, 0.0f, 0.0f ); //rootPosから見たCoG位置
//const static float Zc = gen_Zc + g_CoG_offset[2];
const static float Zc = gen_Zc;
//const static Vec3 g_CoG_offset( 0.0f, 0.0f, 0.100f ); //rootPosから見たCoG位置
/* ready_jointangle must be initialized to be radian */
// static float g_ready_joint_angle[12] = {-5.751865e-06, -0.027068, -39.6337, 72.5952, -32.9614, 0.027092,
//                                         5.686155e-06,  -0.026751, -39.6337, 72.5952, -32.9614, 0.026727
// };
//const static Vec3 traj_body_init( -0.0586f, 0.0f, Zc - InitialLfoot_p[2]);
const static Vec3 traj_body_init( -0.0445f-g_CoG_offset[0], 0.0f-g_CoG_offset[1], Zc - g_CoG_offset[2]);

#elif ROBOT==1
/* default joint angle for Zc = 0.620 */
//const static float Zc = 0.620;
const static float Zc = gen_Zc;
const static Vec3 g_CoG_offset( 0.0f, 0.0f, 0.0f ); //rootPosから見たCoG位置
//const static Vec3 g_CoG_offset( 0.0f, 0.0f, 0.100f ); //rootPosから見たCoG位置
/* ready_jointangle must be initialized to be radian */
// static float g_ready_joint_angle[12] = {-5.751865e-06, -0.027068, -39.6337, 72.5952, -32.9614, 0.027092,
//                                        5.686155e-06,  -0.026751, -39.6337, 72.5952, -32.9614, 0.026727
// };
const static Vec3 traj_body_init( +0.025f-g_CoG_offset[0], 0.0f-g_CoG_offset[1], Zc - g_CoG_offset[2]);
#elif ROBOT==2
/* default joint angle for Zc = 0.620 */
const static float Zc = gen_Zc;
const static Vec3 g_CoG_offset( 0.0f, 0.0f, 0.0f ); //rootPosから見たCoG位置
/* ready_jointangle must be initialized to be radian */
// static float g_ready_joint_angle[12] = {-5.751865e-06, -0.027068, -39.6337, 72.5952, -12.9614, 0.027092,
//                                        5.686155e-06,  -0.026751, -39.6337, 72.5952, -12.9614, 0.026727
// };
const static Vec3 traj_body_init( +0.025f-g_CoG_offset[0], 0.0f-g_CoG_offset[1], Zc - g_CoG_offset[2]);
#else
#error "Undefined ROBOT TYPE."
#endif
#else
#error "Definition of ROBOT is unavailable."
#endif /* if defined(ROBOT)*/

/* traj_body_init のZは足首高さを基準とした倒立振子モデル用の初期値なのでロボットのrootlink初期値として使うときには + foot heightする */
//const static Vec3 body_p_default_offset(traj_body_init[0], traj_body_init[1], 0.0f);
const static Vec3 body_p_default_offset(0.0f, 0.0f, 0.0f);
//const static Vec3 default_zmp_offset_l(0.0f, 0.0f, 0.0f);
//const static Vec3 default_zmp_offset_r(0.0f, 0.0f, 0.0f);
const static Vec3 default_zmp_offset_l(traj_body_init[0],0.0f,0.0f);
const static Vec3 default_zmp_offset_r(traj_body_init[0],0.0f,0.0f);

#if 0
inline float rad2deg(const float a){
#if defined(__INTEL_COMPILER)||defined(__ICC)
    return a * (180.0f / Q_PI);
#elif defined(__GNUC__)
    return a * (180.0f / bodylink::S_PI);
#endif
};
inline float deg2rad(const float a){
#if defined(__INTEL_COMPILER)||defined(__ICC)
    return a * (Q_PI / 180.0f);
#elif defined(__GNUC__)
    return a * (bodylink::S_PI / 180.0f);
#endif
};
#endif

/* IIKMethod, BodyIKMethod LinkIKParam are from ikfk_controller.h.cpp */
#if defined(ROBOT)
#if ROBOT==0
//typedef TestIKParam LegIKParam;
typedef LzeroIKParam LegIKParam;
#define LEG_IK_TYPE IK_LEG_TYPE_A
#elif ROBOT==1
typedef LoneIKParam LegIKParam;
#define LEG_IK_TYPE IK_LEG_TYPE_B
#elif ROBOT==2
typedef LoneWIKParam LegIKParam;
#define LEG_IK_TYPE IK_LEG_TYPE_B
#else
#error "Undefined ROBOT TYPE."
#endif
#else
#error "Definition of ROBOT is unavailable."
#endif /* if defined(ROBOT)*/

namespace RobotConfiguration{
#if defined(ROBOT)
#if ROBOT==0
    const static std::string robotname = "L0";
#elif ROBOT==1
    const static std::string robotname = "L1";
#elif ROBOT==2
    const static std::string robotname = "L1W";
#else
#error "Undefined ROBOT TYPE."
#endif
#else
#error "Definition of ROBOT is unavailable."
#endif /* if defined(ROBOT)*/
} /* end of namespace */

class IIKMethod : public Aligned<16>
{
public:
    /// @return body_pos
    virtual Vec3 calcik( const Mat3& body_R, const Vec3& ref_pos, const Vec3& footl_pos, const Vec3& footr_pos, const float footl_pitch, const float footr_pitch, float* joint_angle ) = 0;
    virtual Vec3 calcik_r( const Mat3& body_R, const Vec3& ref_pos, const Vec3& footl_pos, const Vec3& footr_pos, const float footl_pitch, const float footr_pitch, const float footl_roll, const float footr_roll, float* joint_angle ) = 0;
    virtual Vec3 calcik_ini( const Mat3& body_R, const Vec3& ref_pos, const Vec3& footl_pos, const Vec3& InitialLfoot_p, const Vec3& footr_pos, const Vec3& InitialRfoot_p, const Mat3& InitialLfoot_R, const Mat3& InitialRfoot_R, const float footl_pitch, const float footr_pitch, const float footl_roll, const float footr_roll, float* joint_angle ) = 0;
    /// @return ref_pos
    virtual Vec3 calcref( JointTransform& pR ) = 0;
    virtual ~IIKMethod(){}
};

class BodyIKMethod : public IIKMethod
{
        /// Rootリンク座標の初期値
        const Vec3 InitialBody_p;
    int loop;
public:
        BodyIKMethod( const float body_x, const float body_z )
            : InitialBody_p( body_x, 0, body_z ), loop(0)
        {
#if defined(__INTEL_COMPILER)||defined(__ICC)
                BOOST_STATIC_ASSERT( __alignof__(InitialBody_p) == 16 );
#elif defined(__GNUC__)
#endif
        }
        virtual Vec3 calcik( const Mat3& body_R, const Vec3& ref_pos, const Vec3& footl_pos, const Vec3& footr_pos, const float footl_pitch, const float footr_pitch, float* joint_angle )
        {
#if defined(__INTEL_COMPILER)||defined(__ICC)
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
#elif defined(__GNUC__)
                _MM_ALIGN16 float c[4],s[4];
                bodylink::sincos_ps( bodylink::F32vec4(footl_pitch,footr_pitch,0.0f,0.0f), (bodylink::v4sf*)s, (bodylink::v4sf*)c );
                const Mat3 footl_R = bodylink::rotateMat3<1>( c[0], s[0] );
                const Mat3 footr_R = bodylink::rotateMat3<1>( c[1], s[1] );
                loop++;
                Vec3 body_pos = InitialBody_p + ref_pos;
                ik_legLR<LegIKParam,LEG_IK_TYPE>(
                                     body_R, body_pos,
                                     footl_R, footl_pos,
                                     footr_R, footr_pos,
                                     joint_angle );
#endif
        }
    virtual Vec3 calcik_r( const Mat3& body_R, const Vec3& ref_pos, const Vec3& footl_pos, const Vec3& footr_pos, const float footl_pitch, const float footr_pitch, const float footl_roll, const float footr_roll, float* joint_angle )
        {
                _MM_ALIGN16 float c[4],s[4];
                bodylink::sincos_ps( bodylink::F32vec4(footl_pitch,footr_pitch,footl_roll,footr_roll), (bodylink::v4sf*)s, (bodylink::v4sf*)c );
                const Mat3 footl_R = bodylink::rotateMat3<1>( c[0], s[0] )*bodylink::rotateMat3<0>( c[2], s[2] );
                const Mat3 footr_R = bodylink::rotateMat3<1>( c[1], s[1] )*bodylink::rotateMat3<0>( c[3], s[3] );

                loop++;
                Vec3 body_pos = InitialBody_p + ref_pos;
                ik_legLR<LegIKParam,LEG_IK_TYPE>(
                                     body_R, body_pos,
                                     footl_R, footl_pos,
                                     footr_R, footr_pos,
                                     joint_angle );
        };
    virtual Vec3 calcik_ini( const Mat3& body_R, const Vec3& ref_pos, const Vec3& footl_pos, const Vec3& InitialLfoot_p, const Vec3& footr_pos, const Vec3& InitialRfoot_p, const Mat3& InitialLfoot_R, const Mat3& InitialRfoot_R, const float footl_pitch, const float footr_pitch, const float footl_roll, const float footr_roll, float* joint_angle )
    {
        _MM_ALIGN16 float c[4],s[4];
        bodylink::sincos_ps( bodylink::F32vec4(footl_pitch,footr_pitch,footl_roll,footr_roll), (bodylink::v4sf*)s, (bodylink::v4sf*)c );
        const Mat3 ee_footl_R = bodylink::rotateMat3<1>( c[0], s[0] )*bodylink::rotateMat3<0>( c[2], s[2] );
        const Mat3 ee_footr_R = bodylink::rotateMat3<1>( c[1], s[1] )*bodylink::rotateMat3<0>( c[3], s[3] );
        const Vec3 footl_pos_mod = ee_footl_R * Vec3(InitialLfoot_p[0],
                                                  0.0f,
                                                  InitialLfoot_p[2]
                                                  )
            + footl_pos + Vec3(0.0f, InitialLfoot_p[1], 0.0f);
        const Vec3 footr_pos_mod = ee_footr_R * Vec3(InitialRfoot_p[0],
                                                  0.0f,
                                                  InitialRfoot_p[2]
                                                  )
            + footr_pos + Vec3(0.0f, InitialRfoot_p[1], 0.0f);
        const Mat3 footl_R = ee_footl_R * InitialLfoot_R;
        const Mat3 footr_R = ee_footr_R * InitialRfoot_R;
        loop++;
        Vec3 body_pos = InitialBody_p + ref_pos;
        ik_legLR<LegIKParam,LEG_IK_TYPE>(
                                         body_R, body_pos,
                                         footl_R, footl_pos_mod,
                                         footr_R, footr_pos_mod,
                                         joint_angle );
    };
    virtual Vec3 calcref( JointTransform& pR )
    {
        return pR.p[JOINT_ROOT];
    }
};

#endif /* __BODYIKMETHOD_H_ */
