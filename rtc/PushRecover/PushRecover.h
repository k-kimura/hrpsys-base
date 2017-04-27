// -*- C++ -*-
/*!
 * @file  PushRecover.h
 * @brief null component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef PUSHRECOVER_H
#define PUSHRECOVER_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>
#include <hrpUtil/EigenTypes.h>

#include "../ImpedanceController/RatsMatrix.h"
#include "../TorqueFilter/IIRFilter.h"

#include "QzMatrix.h"
#include "ReactivePatternGenerator.h"
//#include "OnlinePatternGenerator.h"
#include "BodyIKMethod.h"
#include "interpolator.h"  /* from hrpsys/rtc/SequencePlayer */
#include "FrameRateMatcher.h"
#include "SimpleLogger.h"
#include <algorithm>
#include <boost/shared_ptr.hpp>

/* for gettimeofday */
#include <sys/time.h>
#include <time.h>

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "PushRecoverService_impl.h"

#include "Vec3e.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

/**
   \brief sample RT component which has one data input port and one data output port
*/
class PushRecover
    : public RTC::DataFlowComponentBase
{
public:
    /**
       \brief Constructor
       \param manager pointer to the Manager
    */
    PushRecover(RTC::Manager* manager);
    /**
       \brief Destructor
    */
    virtual ~PushRecover();

    // The initialize action (on CREATED->ALIVE transition)
    // formaer rtc_init_entry()
    virtual RTC::ReturnCode_t onInitialize();

    // The finalize action (on ALIVE->END transition)
    // formaer rtc_exiting_entry()
    virtual RTC::ReturnCode_t onFinalize();

    // The startup action when ExecutionContext startup
    // former rtc_starting_entry()
    // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

    // The shutdown action when ExecutionContext stop
    // former rtc_stopping_entry()
    // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

    // The activated action (Active state entry action)
    // former rtc_active_entry()
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

    // The deactivated action (Active state exit action)
    // former rtc_active_exit()
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

    // The execution action that is invoked periodically
    // former rtc_active_do()
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

    // The aborting action when main logic error occurred.
    // former rtc_aborting_entry()
    // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

    // The error action in ERROR state
    // former rtc_error_do()
    // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

    // The reset action that is invoked resetting
    // This is same but different the former rtc_init_entry()
    // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);

    // The state update action that is invoked after onExecute() action
    // no corresponding operation exists in OpenRTm-aist-0.2.0
    // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

    // The action that is invoked when execution context's rate is changed
    // no corresponding operation exists in OpenRTm-aist-0.2.0
    // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);
    //bool setTendonPairParam(const PushRecoverService::TendonPairParam& i_param);
    //bool getTendonPairParam(PushRecoverService::TendonPairParam& i_param);


    enum PushRecoveryState {PR_IDLE, PR_READY, PR_BUSY, PR_TRANSITION_TO_READY, PR_TRANSITION_TO_IDLE};
    enum PushDetectorState {PD_DISABLE=0, PD_ENABLE=1};

    /* Assert emergencyStopReqFlag = true from service port */
    bool assertEmergencyStop(void);

    /* change current control state to target_state and enable interpolator */
    bool shiftPushRecoveryState(PushRecoveryState target_state);

    /* Stop thread until the control state to be not busy */
    void waitPushRecoveryStateReady(void);

    /* Start push recovery mode */
    bool startPushRecovery(void);

    /* Stop push recovery mode */
    bool stopPushRecovery(void);

    /* Start push recovery logger */
    bool startLogging(void);

    /* Stop push recovery logger */
    bool stopLogging(void);

    /* Enable/Disable Push Detection */
    bool enablePushDetect(void);
    bool disablePushDetect(void);

    /* PushDetectParam Service function */
    bool setPushDetectParam(const OpenHRP::PushRecoverService::PushDetectParam& i_param);
    bool getPushDetectParam(OpenHRP::PushRecoverService::PushDetectParam& o_param);

    template<class T>
    struct TrajectoryElement {
        T p;
        T body_p;
        T footl_p;
        T footr_p;
        T dp;
        T body_dp;
        TrajectoryElement(){};
        template<class U>
        TrajectoryElement<T>& operator=(const TrajectoryElement<U>& lhs){
            this->p       = lhs.p;
            this->body_p  = lhs.body_p;
            this->footl_p = lhs.footl_p;
            this->footr_p = lhs.footr_p;
            this->dp      = lhs.dp;
            this->body_dp = lhs.body_dp;
            return *this;
        };
        void clear(void){
            p       = T(0.0,0.0,0.0);
            body_p  = T(0.0,0.0,0.0);
            footl_p = T(0.0,0.0,0.0);
            footr_p = T(0.0,0.0,0.0);
            dp      = T(0.0,0.0,0.0);
            body_dp = T(0.0,0.0,0.0);
        };
    };

protected:
    // Configuration variable declaration
    // <rtc-template block="config_declare">
    TimedDoubleSeq m_qRef;
    TimedDoubleSeq m_qCurrent;
    std::vector<TimedDoubleSeq> m_force;
    std::vector<TimedDoubleSeq> m_ref_force;
    TimedOrientation3D m_rpy;
    TimedPoint3D m_basePos;
    TimedOrientation3D m_baseRpy;
    TimedPose3D m_basePose;
    TimedPoint3D m_zmp;
    TimedLong m_emergencySignal;

    TimedAcceleration3D m_accRef;
    TimedBooleanSeq m_contactStates;
    TimedDoubleSeq  m_controlSwingSupportTime;
    TimedBoolean    m_walkingStates;
    TimedPoint3D    m_sbpCogOffset;
    // </rtc-template>

    // DataInPort declaration
    // <rtc-template block="inport_declare">
    RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn;
    RTC::InPort<RTC::TimedDoubleSeq> m_qCurrentIn;
    RTC::InPort<TimedPoint3D> m_basePosIn;
    RTC::InPort<TimedOrientation3D> m_baseRpyIn;
    RTC::InPort<TimedPoint3D> m_zmpIn;
    std::vector<RTC::InPort<TimedDoubleSeq> *> m_forceIn;
    std::vector<RTC::InPort<TimedDoubleSeq> *> m_ref_forceIn;
    RTC::InPort<TimedOrientation3D> m_rpyIn;
    RTC::InPort<TimedLong> m_emergencySignalIn;

    RTC::InPort<RTC::TimedAcceleration3D> m_accRefIn;
    RTC::InPort<RTC::TimedBooleanSeq> m_contactStatesIn;
    RTC::InPort<RTC::TimedDoubleSeq> m_controlSwingSupportTimeIn;
    RTC::InPort<RTC::TimedBoolean> m_walkingStatesIn;
    RTC::InPort<RTC::TimedPoint3D> m_sbpCogOffsetIn;
    // </rtc-template>

    // DataOutPort declaration
    // <rtc-template block="outport_declare">
    // </rtc-template>

    // DataOutPort declaration
    // <rtc-template block="outport_declare">
    RTC::OutPort<RTC::TimedDoubleSeq> m_qRefOut;
    RTC::OutPort<RTC::TimedPoint3D> m_zmpOut;
    RTC::OutPort<RTC::TimedPoint3D> m_basePosOut;
    RTC::OutPort<RTC::TimedOrientation3D> m_baseRpyOut;
    RTC::OutPort<RTC::TimedPose3D> m_basePoseOut;

    std::vector<RTC::OutPort<TimedDoubleSeq> *> m_ref_forceOut;
    RTC::OutPort<RTC::TimedAcceleration3D> m_accRefOut;
    RTC::OutPort<RTC::TimedBooleanSeq> m_contactStatesOut;
    RTC::OutPort<RTC::TimedDoubleSeq> m_controlSwingSupportTimeOut;
    RTC::OutPort<RTC::TimedBoolean> m_walkingStatesOut;
    RTC::OutPort<RTC::TimedPoint3D> m_sbpCogOffsetOut;
    // </rtc-template>

    // CORBA Port declaration
    // <rtc-template block="corbaport_declare">

    // </rtc-template>

    // Service declaration
    // <rtc-template block="service_declare">
    RTC::CorbaPort m_PushRecoverServicePort;

    // </rtc-template>

    // Consumer declaration
    // <rtc-template block="consumer_declare">
    PushRecoverService_impl m_service0;

    // </rtc-template>

private:
    double m_dt;
    double m_dt_i;
    coil::Mutex m_mutex;
    std::map<std::string, hrp::Vector3> abs_forces, abs_moments, abs_ref_forces, abs_ref_moments;
    hrp::BodyPtr m_robot;
    unsigned int m_debugLevel;
    bool emergencyStopReqFlag;
    int loop;

    /* PushDetectParam */
    PushDetectorState pushDetector_state;
    OpenHRP::PushRecoverService::PushDetectParam pushDetectParam;

    /* Input data buffer */
    hrp::Vector3 input_zmp, input_basePos;
    hrp::Matrix33 input_baseRot;

    /* Actual Data calced from input data */
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > act_cogvel_filter;
    hrp::Vector3  act_zmp,       prev_act_zmp;
    hrp::Vector3  rel_act_zmp,   prev_rel_act_zmp;
    hrp::Vector3  act_root_pos,  prev_act_root_pos;
    hrp::Vector3  act_world_root_pos;
    hrp::Vector3  act_cog,       prev_act_cog;
    hrp::Vector3  act_cogvel,    prev_act_cogvel;
    hrp::Vector3  act_base_rpy;
    hrp::Matrix33 prev_act_foot_origin_rot;
    double        prev_act_force_z[2]; /* previous force z used in calcZMP */

    /* Estimated state data */
    hrp::Vector3 est_cogvel;

    /* Reference Data buffer */
    double        *ref_q,   *prev_ref_q;
    hrp::Vector3  rel_ref_zmp, prev_rel_ref_zmp; // ref zmp in base frame
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > ref_zmp_modif_filter;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > ref_basePos_modif_filter;
    hrp::Vector3  ref_zmp, ref_basePos, prev_ref_zmp, prev_ref_basePos, ref_zmp_modif, ref_basePos_modif;
    hrp::Matrix33     ref_baseRot;
    hrp::Vector3      prev_imu_sensor_pos, prev_imu_sensor_vel;
    hrp::Vector3      ref_cog;
    hrp::Vector3      ref_force[2];
    TrajectoryElement<hrp::Vector3e>  prev_ref_traj;

    interpolator *transition_interpolator;
    double transition_interpolator_ratio;
    PushRecoveryState current_control_state;

    /* indexed robot model parts names */
    struct EElinkParam {
        std::string   fsensor_name;
        std::string   ee_name;
        std::string   ee_target;
        std::string   ee_base;
        hrp::Vector3  ee_localp;
        hrp::Matrix33 ee_localR;
        bool          act_contact_state;
    };
    enum PR_CONTACT_STATE {BOTH_FOOTS, RFOOT, LFOOT, ON_AIR};
    PR_CONTACT_STATE foots_contact_states, prev_foots_contact_states;
    std::map<std::string, size_t> ee_index_map;
    std::vector<EElinkParam> ee_params;
    hrp::Vector3 world_sensor_ps[2];
    hrp::Vector3 world_force_ps[2];
    hrp::Vector3 world_force_ms[2];

    IIKMethod* m_pIKMethod;
    hrp::Vector3 body_p_at_start, body_p_diff_at_start;
    hrp::Vector3 basePos_modif_at_start;
    FrameRateMatcher rate_matcher;

    ReactivePatternGenerator stpf;

    struct ControlBodyComplianceContext {
        double prev_u;
    };
    ControlBodyComplianceContext bodyComplianceContext[2];

    void updateInputData(const bool shw_msg_flag = false);
    void updateEstimatedInputData(void);
    void updateEstimatedOutputData(void);
    void setTargetDataWithInterpolation(void);
    void setOutputData(const bool shw_msg_flag = false);

    bool calcWorldForceVector(void);
    void calcFootOriginCoords (hrp::Vector3& foot_origin_pos,
                               hrp::Matrix33& foot_origin_rot);
    bool calcZMP(hrp::Vector3& ret_zmp, const double zmp_z);
    bool calcActCoGVel(const hrp::Vector3 act_foot_origin_pos,
                       const hrp::Matrix33 act_foot_origin_rot);
    bool calcActRootPos(const hrp::Vector3 act_foot_origin_pos,
                       const hrp::Matrix33 act_foot_origin_rot);
    bool updateToCurrentRobotPose(void);
    bool checkJointVelocity(void);
    bool checkBodyPosMergin(const double threshold2, const int loop, const bool mask = false);
    bool controlBodyCompliance(bool is_enable);
    void trajectoryReset(void);
    /* ============================================== */
    /* checkEmergencyFlag()                           */
    /* out:                                           */
    /*    :bool                                       */
    /*     EmergencyStopRequestが来ていたらtrueを返す  */
    /*     EmergencyStopReqFlagはfalseになおす。       */
    /* ============================================== */
    bool checkEmergencyFlag(void);


    //boost::shared_ptr<SimpleLogger> slogger;
    SimpleLogger *slogger;
    SimpleLogger::DataLog  dlog;
    bool                   dlog_save_flag;
    struct timeval         stv; /* time of OnInitialized */
};

extern "C"
{
    void PushRecoverInit(RTC::Manager* manager);
};

#define PRINTVEC3(v,f) if( f ){                \
    printf("%s=[", #v );       \
    for(int i=0;i<3;i++){\
        printf("%+3.3lf",(double) v##[i]);         \
        if(i!=2){printf(", ");}                 \
        else{printf("]\n");}                    \
    }                                           \
        }
//#undef PRINTVEC3
//#define PRINTVEC3(v,f) printf("%s", #v )
#endif // PUSHRECOVER_H
