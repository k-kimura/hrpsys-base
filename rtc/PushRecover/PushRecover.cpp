// -*- C++ -*-
/*!
 * @file  PushRecover.cpp
 * @brief Push recovery controller
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "PushRecover.h"
#include <hrpUtil/MatrixSolvers.h>
#include <hrpModel/Sensor.h>

// #include "QzMatrix.h"
// #include "step_forward.h"
// #include "BodyIKMethod.h"
#include <boost/static_assert.hpp>

typedef coil::Guard<coil::Mutex> Guard;

// Module specification
// <rtc-template block="module_spec">
static const char* pushrecover_spec[] =
  {
    "implementation_id", "PushRecover",
    "type_name",         "PushRecover",
    "description",       "Push Recover",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debugLevel", "0",
    ""
  };
// </rtc-template>

PushRecover::PushRecover(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      //Inport
      m_qRefIn("qRef", m_qRef),
      m_qCurrentIn("qCurrent", m_qCurrent),
      m_basePosIn("basePosIn", m_basePos),
      m_baseRpyIn("baseRpyIn", m_baseRpy),
      m_zmpIn("zmpIn", m_zmp),
      m_rpyIn("rpy", m_rpy),
      m_emergencySignalIn("emergencySignal", m_emergencySignal),
      m_accRefIn("accRefIn", m_accRef),
      m_contactStatesIn("contactStatesIn", m_contactStates),
      m_controlSwingSupportTimeIn("controlSwingSupportTimeIn", m_controlSwingSupportTime),
      m_walkingStatesIn("walkingStatesIn", m_walkingStates),
      m_sbpCogOffsetIn("sbpCogOffsetIn", m_sbpCogOffset),
      //Outport
      m_qRefOut("q", m_qRef),
      m_zmpOut("zmpOut", m_zmp),
      m_basePosOut("basePosOut", m_basePos),
      m_baseRpyOut("baseRpyOut", m_baseRpy),
      m_basePoseOut("basePoseOut", m_basePose),
      m_accRefOut("accRefOut", m_accRef),
      m_contactStatesOut("contactStatesOut", m_contactStates),
      m_controlSwingSupportTimeOut("controlSwingSupportTimeOut", m_controlSwingSupportTime),
    m_walkingStatesOut("walkingStatesOut", m_walkingStates),
    m_sbpCogOffsetOut("sbpCogOffsetOut", m_sbpCogOffset),

    m_PushRecoverServicePort("PushRecoverService"),
    rate_matcher(500,1000),
// </rtc-template>
    m_debugLevel(0)
{
    m_service0.pushrecover(this);
    emergencyStopReqFlag = false;

    BOOST_STATIC_ASSERT( __alignof__(BodyIKMethod) == 16 );
    m_pIKMethod = new BodyIKMethod( 0.0f, Zc );
    body_p_at_start = hrp::Vector3(0.0, 0.0, 0.0);
}



PushRecover::~PushRecover()
{
    delete m_pIKMethod;
}



RTC::ReturnCode_t PushRecover::onInitialize()
{
  std::cout << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "0");

  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qRef", m_qRefIn);
  addInPort("qCurrent", m_qCurrentIn);
  addInPort("rpy", m_rpyIn);
  addInPort("basePosIn", m_basePosIn);
  addInPort("baseRpyIn", m_baseRpyIn);
  addInPort("zmpIn", m_zmpIn);
  addInPort("emergencySignal", m_emergencySignalIn);

  addInPort("accRefIn", m_accRefIn);
  addInPort("contactStatesIn", m_contactStatesIn);
  addInPort("controlSwingSupportTimeIn", m_controlSwingSupportTimeIn);
  addInPort("walkingStatesIn", m_walkingStatesIn);
  addInPort("sbpCogOffsetIn", m_sbpCogOffsetIn);

  // Set OutPort buffer
  addOutPort("q", m_qRefOut);
  addOutPort("zmpOut", m_zmpOut);
  addOutPort("basePosOut", m_basePosOut);
  addOutPort("baseRpyOut", m_baseRpyOut);
  addOutPort("basePoseOut", m_basePoseOut);
  addOutPort("accRefOut", m_accRefOut);
  addOutPort("contactStatesOut", m_contactStatesOut);
  addOutPort("controlSwingSupportTimeOut", m_controlSwingSupportTimeOut);
  addOutPort("walkingStatesOut", m_walkingStatesOut);
  addOutPort("sbpCogOffsetOut", m_sbpCogOffsetOut);
  // Set OutPort buffer

  // Set service provider to Ports
  m_PushRecoverServicePort.registerProvider("service0", "PushRecoverService", m_service0);

  // Set service consumers to Ports

  // Set CORBA Service Ports
  addPort(m_PushRecoverServicePort);

  // </rtc-template>
#if 1
  RTC::Properties& prop = getProperties();
  coil::stringTo(m_dt, prop["dt"].c_str());

  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0){
      comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());

  //parameters for internal robot model
  std::cout << "Loading Model=[" << prop["model"].c_str() << std::endl;

  m_robot = hrp::BodyPtr(new hrp::Body());
  if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(),
                               CosNaming::NamingContext::_duplicate(naming.getRootContext())
          )){
      std::cerr << "PR failed to load model[" << prop["model"] << "] in "
                << m_profile.instance_name << std::endl;
      return RTC::RTC_ERROR;
  }
#endif

  /* Start Setting Force Sensor Port */
  // Setting for wrench data ports (real + virtual)
  std::vector<std::string> fsensor_names;
  //   find names for real force sensors
  int npforce = m_robot->numSensors(hrp::Sensor::FORCE);
  for (unsigned int i=0; i<npforce; i++){
      fsensor_names.push_back(m_robot->sensor(hrp::Sensor::FORCE, i)->name);
  }
  //   add ports for all force sensors
  int nforce  = npforce;
  m_force.resize(nforce);
  m_forceIn.resize(nforce);
  m_ref_force.resize(nforce);
  m_ref_forceIn.resize(nforce);
  std::cerr << "[" << m_profile.instance_name << "] force sensor ports" << std::endl;
  for (unsigned int i=0; i<nforce; i++){
      // actual inport
      m_forceIn[i] = new RTC::InPort<TimedDoubleSeq>(fsensor_names[i].c_str(), m_force[i]);
      m_force[i].data.length(6);
      registerInPort(fsensor_names[i].c_str(), *m_forceIn[i]);
      // ref inport
      m_ref_force[i].data.length(6);
      for (unsigned int j=0; j<6; j++) m_ref_force[i].data[j] = 0.0;
      m_ref_forceIn[i] = new RTC::InPort<TimedDoubleSeq>(std::string("ref_"+fsensor_names[i]+"In").c_str(), m_ref_force[i]);
      registerInPort(std::string("ref_"+fsensor_names[i]+"In").c_str(), *m_ref_forceIn[i]);
      std::cerr << "[" << m_profile.instance_name << "]   name = " << fsensor_names[i] << std::endl;
  }
  for (unsigned int i=0; i<m_forceIn.size(); i++){
      abs_forces.insert(std::pair<std::string, hrp::Vector3>(m_forceIn[i]->name(), hrp::Vector3::Zero()));
      abs_moments.insert(std::pair<std::string, hrp::Vector3>(m_forceIn[i]->name(), hrp::Vector3::Zero()));
  }
  /* Finish Setting Force Sensor Port */

  // setting from conf file
  // rleg,TARGET_LINK,BASE_LINK
  coil::vstring end_effectors_str = coil::split(prop["end_effectors"], ",");
  const size_t prop_num = 10;
  if (end_effectors_str.size() > 0) {
      const size_t num = end_effectors_str.size()/prop_num;
      for (size_t i = 0; i < num; i++) {
          std::string ee_name, ee_target, ee_base;
          coil::stringTo(ee_name, end_effectors_str[i*prop_num].c_str());
          coil::stringTo(ee_target, end_effectors_str[i*prop_num+1].c_str());
          coil::stringTo(ee_base, end_effectors_str[i*prop_num+2].c_str());
          /* link endeffector name and index */
          contact_states_index_map.insert(std::pair<std::string, size_t>(ee_name, i));
      }
  }
  /* Finish Setting EndEffector from config file */

  /* clear Emergency Stop Request flag */
  emergencyStopReqFlag = false;

  /* Initialize Acceleration Reference */
  m_accRef.data.ax = m_accRef.data.ay = m_accRef.data.az = 0.0;
  prev_imu_sensor_pos = hrp::Vector3::Zero();
  prev_imu_sensor_vel = hrp::Vector3::Zero();


  /* Initialize Reference Joint Angle Array */
  ref_q = new double[m_robot->numJoints()];
  for(int i=0;i<m_robot->numJoints();i++){
      ref_q[i]=0.0;
  }

  std::cout << "[" << m_profile.instance_name << "] calcik start" << std::endl;
  /* Initialize Default joint angle on PR_READY state */
  {
      _MM_ALIGN16 float target_joint_angle[12];
      _MM_ALIGN16 float pre_joint_angle[12];
      _MM_ALIGN16 Mat3 body_R = Mat3::identity();
      const float foot_l_pitch = 0.0f;
      const float foot_r_pitch = 0.0f;
      _MM_ALIGN16 Vec3 body_p = m_pIKMethod->calcik(body_R,
                                                    Vec3(0.0f,0.0f,0.0f),
                                                    InitialLfoot_p,
                                                    InitialRfoot_p,
                                                    foot_l_pitch,
                                                    foot_r_pitch,
                                                    target_joint_angle );
      for(int i=0;i<12;i++){
          ready_joint_angle[i] = target_joint_angle[i];
      }
  }
  std::cout << "[" << m_profile.instance_name << "] calcik end" << std::endl;

  /* Initialize transition interpolator to interpolate from idle state to push recover ready state */
  {
      const int interpolator_dimension = 1;
      const double average_velocity = 1.0;
      transition_interpolator = new interpolator(interpolator_dimension, m_dt, interpolator::HOFFARBIB, average_velocity);
      transition_interpolator->setName(std::string(m_profile.instance_name)+" transition_interpolator");
      transition_interpolator_ratio = 1.0;
  } /* Finish initializing transition interpolator */

  if(m_robot->numJoints()!=12){
      return RTC::RTC_ERROR;
  }else{
      return RTC::RTC_OK;
  }
}


RTC::ReturnCode_t PushRecover::onFinalize()
{
    delete ref_q;
    delete transition_interpolator;
    return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t PushRecover::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PushRecover::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t PushRecover::onActivated(RTC::UniqueId ec_id)
{
    std::cout << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;

    current_control_state = PR_IDLE;

    return RTC::RTC_OK;
}

RTC::ReturnCode_t PushRecover::onDeactivated(RTC::UniqueId ec_id)
{
    std::cout << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
    Guard guard(m_mutex);
    if(current_control_state != PR_IDLE){
        current_control_state = PR_IDLE;
        const double tmp_ratio = 0.0;
        transition_interpolator->go(&tmp_ratio, m_dt, true); // sync in one controller loop
    }
    return RTC::RTC_OK;
}

void PushRecover::updateInputData(const bool shw_msg_flag){
#define DEBUG_INPUT_PRINT(x)  {if(shw_msg_flag) std::cout << "[pr] " << #x << "=" << x << std::endl;}
#define DEBUG_INPUT_PRINT_I(x,i)  {if(shw_msg_flag) std::cout << "[pr] " << #x << "[" << i << "]=" << x##[i] << std::endl;}
    for (unsigned int i=0; i<m_forceIn.size(); i++){
        if ( m_forceIn[i]->isNew() ) {
            m_forceIn[i]->read();
            DEBUG_INPUT_PRINT(m_force[i].data[0]);
            DEBUG_INPUT_PRINT(m_force[i].data[1]);
            DEBUG_INPUT_PRINT(m_force[i].data[2]);
        }
        if ( m_ref_forceIn[i]->isNew() ) {
            m_ref_forceIn[i]->read();
            DEBUG_INPUT_PRINT(m_ref_force[i].data[0]);
            DEBUG_INPUT_PRINT(m_ref_force[i].data[1]);
            DEBUG_INPUT_PRINT(m_ref_force[i].data[2]);
        }
    }
    if (m_basePosIn.isNew()) {
        m_basePosIn.read();
        input_basePos(0) = m_basePos.data.x;
        input_basePos(1) = m_basePos.data.y;
        input_basePos(2) = m_basePos.data.z;
        DEBUG_INPUT_PRINT(input_basePos(0));
        DEBUG_INPUT_PRINT(input_basePos(1));
        DEBUG_INPUT_PRINT(input_basePos(2));
    }
    if (m_baseRpyIn.isNew()) {
        m_baseRpyIn.read();
        input_baseRot = hrp::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
    }
    if (m_zmpIn.isNew()) {
        m_zmpIn.read();
        input_zmp(0) = m_zmp.data.x;
        input_zmp(1) = m_zmp.data.y;
        input_zmp(2) = m_zmp.data.z;
        DEBUG_INPUT_PRINT(input_zmp(0));
        DEBUG_INPUT_PRINT(input_zmp(1));
        DEBUG_INPUT_PRINT(input_zmp(2));
    }
    if (m_rpyIn.isNew()) {
        m_rpyIn.read();
    }
    if (m_qCurrentIn.isNew()) {
        m_qCurrentIn.read();
    }
    if (m_qRefIn.isNew()) {
        m_qRefIn.read();
    }
    if (m_emergencySignalIn.isNew()){
        m_emergencySignalIn.read();
        std::cout << "[" << m_profile.instance_name << "] emergencySignal is set!" << std::endl;
    }
    if (m_accRefIn.isNew()){
        m_accRefIn.read();
    }
    if (m_contactStatesIn.isNew()){
        m_contactStatesIn.read();
    }
    if (m_controlSwingSupportTimeIn.isNew()){
        m_controlSwingSupportTimeIn.read();
    }
    if (m_walkingStatesIn.isNew()){
        m_walkingStatesIn.read();
    }
    if (m_sbpCogOffsetIn.isNew()){
        m_sbpCogOffsetIn.read();
    }

#undef DEBUG_INPUT_PRINT
#undef DEBUG_INPUT_PRINT_I
}

void PushRecover::setReferenceDataWithInterpolation(void){
    bool is_transition_interpolator_empty = transition_interpolator->isEmpty();
    if (!is_transition_interpolator_empty) {
        /* Interpolation is currently working */
        transition_interpolator->get(&transition_interpolator_ratio, true);
    } else {
        /* Interpolation is not currently working */
        transition_interpolator_ratio = 1.0; /* use controller output */
        if(current_control_state == PR_TRANSITION_TO_READY){
            current_control_state = PR_READY;
            /* Transition to state PR_READY needs to set default values */
            for ( int i = 0; i < m_robot->numJoints(); i++ ) {
                m_robot->joint(i)->q = ready_joint_angle[i];
            }
            m_robot->rootLink()->p = hrp::Vector3(0.0, 0.0, Zc);
        }else if(current_control_state == PR_TRANSITION_TO_IDLE){
            current_control_state = PR_IDLE;
            // for ( int i = 0; i < m_robot->numJoints(); i++ ) {
            //     m_robot->joint(i)->q = m_qRef.data[i];
            // }
        }
    }

    /* transition */
    if (!is_transition_interpolator_empty){
        /* interpolate joint angle by transition_interpolator_ratio */
        /* ratio=1 ->  controlled joint angle                       */
        /* ratio=0 ->  pass through qRef                            */
        for ( int i = 0; i < m_robot->numJoints(); i++ ) {
            /* expecting the joint angle under PR_READY state equals default_joint_angle */
            switch (current_control_state){
            case PR_TRANSITION_TO_IDLE:
                ref_q[i] = (1-transition_interpolator_ratio) * m_qRef.data[i] + transition_interpolator_ratio * ready_joint_angle[i];
                break;
            case PR_TRANSITION_TO_READY:
                ref_q[i] = (1-transition_interpolator_ratio) * m_qRef.data[i] + transition_interpolator_ratio * ready_joint_angle[i];
                break;
            default:
                break;
            }
        }
        /* Set Base Frame Reference Position */
        switch (current_control_state){
        case PR_TRANSITION_TO_IDLE:
            ref_basePos = (1-transition_interpolator_ratio) * input_basePos + transition_interpolator_ratio * m_robot->rootLink()->p;
            break;
        case PR_TRANSITION_TO_READY:
            ref_basePos = (1-transition_interpolator_ratio) * input_basePos + transition_interpolator_ratio * hrp::Vector3(0.0, 0.0, Zc);
            break;
        default:
            break;
        }
        /* Set Base Frame Reference Rotation */
        rats::mid_rot(ref_baseRot, transition_interpolator_ratio, input_baseRot, m_robot->rootLink()->R);
        /* set relative Reference ZMP */
        rel_ref_zmp = (1-transition_interpolator_ratio) * input_zmp + transition_interpolator_ratio * rel_ref_zmp;
    }else{
        for ( int i = 0; i < m_robot->numJoints(); i++ ) {
            ref_q[i] = m_robot->joint(i)->q;
        }
        if(current_control_state==PR_IDLE){
            /* Set Base Frame Reference Position */
            ref_basePos = input_basePos;
            /* Set Base Frame Reference Rotation */
            ref_baseRot = input_baseRot;
            /* set relative Reference ZMP */
            rel_ref_zmp = input_zmp; /* pass through */
        }else if(current_control_state==PR_READY || current_control_state==PR_BUSY){
            /* Set Base Frame Reference Position */
            ref_basePos = m_robot->rootLink()->p;
            /* Set Base Frame Reference Rotation */
            ref_baseRot = m_robot->rootLink()->R;
            /* set relative Reference ZMP */
            rel_ref_zmp = rel_ref_zmp; /* pass through */
        }
    }/* End of Interpolation */
}

void PushRecover::setOutputData(void){
    /*==================================================*/
    /* Set Target Angle Vector and publish from outport */
    /*==================================================*/
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
        m_qRef.data[i] = ref_q[i];
        //m_qRef.data[i] = m_robot->joint(i)->q;
    }

    // basePos
    m_basePos.data.x = ref_basePos(0);
    m_basePos.data.y = ref_basePos(1);
    m_basePos.data.z = ref_basePos(2);
    m_basePos.tm = m_qRef.tm;
    // baseRpy
    hrp::Vector3 baseRpy = hrp::rpyFromRot(ref_baseRot);
    m_baseRpy.data.r = baseRpy(0);
    m_baseRpy.data.p = baseRpy(1);
    m_baseRpy.data.y = baseRpy(2);
    m_baseRpy.tm = m_qRef.tm;
    // basePose
    m_basePose.data.position.x = m_basePos.data.x;
    m_basePose.data.position.y = m_basePos.data.y;
    m_basePose.data.position.z = m_basePos.data.z;
    m_basePose.data.orientation.r = m_baseRpy.data.r;
    m_basePose.data.orientation.p = m_baseRpy.data.p;
    m_basePose.data.orientation.y = m_baseRpy.data.y;
    m_basePose.tm = m_qRef.tm;
    // zmp
    m_zmp.data.x = rel_ref_zmp(0);
    m_zmp.data.y = rel_ref_zmp(1);
    m_zmp.data.z = rel_ref_zmp(2);
    m_zmp.tm = m_qRef.tm;

    /* Write to OutPort */
    m_basePosOut.write();
    m_baseRpyOut.write();
    m_basePoseOut.write();
    m_qRefOut.write();
    m_zmpOut.write();

    // control parameters
    /* TODO */
    m_accRefOut.write();
    m_contactStatesOut.write();
    m_controlSwingSupportTimeOut.write();
    m_walkingStatesOut.write();
    m_sbpCogOffsetOut.write();
}

bool PushRecover::checkEmergencyFlag(void){
    if(emergencyStopReqFlag==true){
        std::cout << "[" << m_profile.instance_name << "] emergency Stop Activated!" << std::endl;
        emergencyStopReqFlag = false;
        return true;
    }
    return false;
}

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t PushRecover::onExecute(RTC::UniqueId ec_id)
{
  bool start_RWG_flag;
  static int loop = 1;
  if (loop%10000==0){
      std::cout << "[" << m_profile.instance_name<< "] onExecute(" << ec_id << ")" << std::endl;
      //std::cout << "[" << m_profile.instance_name<< "] m_dt=(" << m_dt << ")" << std::endl;
      /* Show current state */
      std::string state_string;
      switch(current_control_state){
      case PR_IDLE:
          state_string = "PR_IDLE";
          break;
      case PR_READY:
          state_string = "PR_READY";
          break;
      case PR_BUSY:
          state_string = "PR_BUSY";
          break;
      default:
          state_string = "Under Transition";
          break;
      }
      std::cout << "[" << m_profile.instance_name << "] Current State=\"" << state_string.c_str() << "\"" << std::endl;
      bool is_transition_interpolator_empty = transition_interpolator->isEmpty();
      if(is_transition_interpolator_empty){
          std::cout << "[" << m_profile.instance_name << "] interpolator empty" << std::endl;
      }else{
          std::cout << "[" << m_profile.instance_name << "] interpolator working" << std::endl;
      }
  }

  /* check dataport input */
  {
      bool shw_msg_flag = loop%500==0?true:false;
      //updateInputData(shw_msg_flag);
      updateInputData(false);
  }
  /* End of checking dataport input */

  Guard guard(m_mutex);

  /* Check if Emergency Stop Flag is active */
  start_RWG_flag = checkEmergencyFlag();

  if(current_control_state==PR_READY || current_control_state==PR_BUSY){
      const float dcomr = 0.1f; /* max 0.4 kurai? */
      const float dcomx = dcomr * cosf(((float)(loop%360))*3.14159265358939323826433f/180.0f);
      const float dcomy = dcomr * sinf(((float)(loop%360))*3.14159265358939323826433f/180.0f);
      const float diff_z = input_basePos(2) - Zc; /* TODO 効果の検証 */
      const Vec3 x0[] = {
          Vec3( 0.0f,    0.0f, 0.0f ),
          Vec3( 0.0f,    0.0f, diff_z ),
          Vec3( dcomx,    dcomy, 0.0f )
      };

      if(start_RWG_flag){
          std::cout << "[" << m_profile.instance_name << "] Calling StepForward start" << std::endl;
          /* Save Current base position */
          body_p_at_start = hrp::Vector3(m_robot->rootLink()->p(0),
                                         m_robot->rootLink()->p(1),
                                         m_robot->rootLink()->p(2)
                                         );
          stpf.start(x0);
          rate_matcher.setCurrentFrame(0);
          current_control_state = PR_BUSY;
      }

      if(current_control_state == PR_BUSY){   /* controller main */
          //void getTrajectoryFrame( const int index, Vec3& pref, Vec3& body_p, Vec3 &footl_p, Vec3 &footr_p )
          Vec3 sf_pref;
          Vec3 sf_body_p;
          Vec3 sf_footl_p;
          Vec3 sf_footr_p;
          ITrajectoryGenerator* gen = stpf.getReady();
          if(gen!=0){
              rate_matcher.incrementFrame();
              gen->getTrajectoryFrame(rate_matcher.getConvertedFrame(),
                                     sf_pref,
                                     sf_body_p,
                                     sf_footl_p,
                                     sf_footr_p );
              // 新しい target_joint_angleを計算
              _MM_ALIGN16 float target_joint_angle[12];
              _MM_ALIGN16 float pre_joint_angle[12];
              _MM_ALIGN16 Mat3 body_R = Mat3::identity();
              const float foot_l_pitch = 0.0f;
              const float foot_r_pitch = 0.0f;
              _MM_ALIGN16 Vec3 body_p = m_pIKMethod->calcik(body_R,
                                                            sf_body_p,
                                                            InitialLfoot_p + sf_footl_p,
                                                            InitialRfoot_p + sf_footr_p,
                                                            foot_l_pitch,
                                                            foot_r_pitch,
                                                            target_joint_angle );
              if(rate_matcher.getCurrentFrame()==1){
                  body_p_at_start[2] = body_p[2];
              }

              for(int i=0;i < 12; i++){
                  m_robot->joint(i)->q = target_joint_angle[i];  /* rad to rad */
              }

              /* set body_p to m_robot loot link position */
              m_robot->rootLink()->p = hrp::Vector3(body_p_at_start(0) + sf_body_p[0],
                                                    body_p_at_start(1) + sf_body_p[1],
                                                    body_p_at_start(2));
                                                    //input_basePos(2));
              /* set body_p to m_robot loot link Rotation */
              m_robot->rootLink()->R = input_baseRot; /* TODO */
              /* set Reference ZMP */
              ref_zmp = hrp::Vector3(sf_pref[0],sf_pref[1],sf_pref[2]);
              /* calc Reference ZMP relative to base_frame(Loot link)  */
              rel_ref_zmp = m_robot->rootLink()->R.transpose() * (ref_zmp - m_robot->rootLink()->p);

              /* set current walking status */
              {
                  /* TODO */
                  m_contactStates.data[contact_states_index_map["rleg"]] = (abs(sf_footr_p[2])<0.001)?true:false;
                  m_contactStates.data[contact_states_index_map["lleg"]] = (abs(sf_footl_p[2])<0.001)?true:false;
                  //std::cout << "contact_state=[" << ((m_contactStates.data[contact_states_index_map["lleg"]])?"true ":"false") << "," << ((m_contactStates.data[contact_states_index_map["rleg"]])?"true ":"false") << "]" << std::endl;
                  m_walkingStates.data = true;
                  /* controlSwingSupportTime is not used while double support period, 1.0 is neglected */
                  //m_controlSwingSupportTime.data[contact_states_index_map["rleg"]] = 1.0;
                  //m_controlSwingSupportTime.data[contact_states_index_map["lleg"]] = 1.0;
              }

              const int rwg_len_out = 5000; /* TODO */
              if(rate_matcher.getConvertedFrame()>rwg_len_out){
                  current_control_state = PR_READY;
              }

              const unsigned int cf = rate_matcher.getCurrentFrame();
              if(cf==1 || cf==2 || cf== 2000){
                  //if(loop%500==0){
                  std::cout << "[" << m_profile.instance_name << "] pref=" << sf_pref << "  @" << rate_matcher.getCurrentFrame() << "frame" << std::endl;
                  std::cout << "[" << m_profile.instance_name << "] sf_body_p=" << sf_body_p << std::endl;
                  std::cout << "[" << m_profile.instance_name << "] body_p=" << body_p << std::endl;
                  std::cout << "[" << m_profile.instance_name << "] body_p_@_start=" << body_p_at_start << std::endl;
                  std::cout << "[" << m_profile.instance_name << "] rootLink_p=" << m_robot->rootLink()->p << std::endl;
                  std::cout << "[" << m_profile.instance_name << "] footl_p=" << sf_footl_p << std::endl;
                  std::cout << "[" << m_profile.instance_name << "] footr_p=" << sf_footr_p << std::endl;
                  std::cout << "[" << m_profile.instance_name << "]=qref[";
                  for(int i=0;i< 12;i++){
                      printf("%+3.2lf, ",rad2deg(m_robot->joint(i)->q));
                  }
                  printf("]\n");
                  std::cout << "[" << m_profile.instance_name << "]=q[";
                  for(int i=0;i< 12;i++){
                      printf("%+3.2lf, ",rad2deg(m_qCurrent.data[i]));
                  }
                  printf("]\n");
              }
          }else{
              std::cout << "[" << m_profile.instance_name << "] There is no tragectory" << std::endl;
          }
      }else if(current_control_state == PR_READY){
          /* set current walking status */
          m_contactStates.data[contact_states_index_map["rleg"]] = true;
          m_contactStates.data[contact_states_index_map["lleg"]] = true;
          m_walkingStates.data = false;
          /* controlSwingSupportTime is not used while double support period, 1.0 is neglected */
          m_controlSwingSupportTime.data[contact_states_index_map["rleg"]] = 1.0;
          m_controlSwingSupportTime.data[contact_states_index_map["lleg"]] = 1.0;
          //m_sbpCogOffsetOut.write();
      }
  }else if(current_control_state == PR_IDLE){
      for ( int i = 0; i < m_robot->numJoints(); i++ ) {
          m_robot->joint(i)->q = m_qRef.data[i]; /* pass through qRefIn joint angle */
      }
      m_robot->rootLink()->p = input_basePos;
      m_robot->rootLink()->R = input_baseRot;
  }


  m_robot->calcForwardKinematics();


  // reference acceleration
  hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
  if (sen != NULL) {
      hrp::Vector3 imu_sensor_pos = sen->link->p + sen->link->R * sen->localPos;
      hrp::Vector3 imu_sensor_vel = (imu_sensor_pos - prev_imu_sensor_pos)/m_dt;
      // convert to imu sensor local acceleration
      hrp::Vector3 acc = (sen->link->R * sen->localR).transpose() * (imu_sensor_vel - prev_imu_sensor_vel)/m_dt;
      m_accRef.data.ax = acc(0); m_accRef.data.ay = acc(1); m_accRef.data.az = acc(2);
      prev_imu_sensor_pos = imu_sensor_pos;
      prev_imu_sensor_vel = imu_sensor_vel;
  }

  /*==================================================*/
  /* Interpolate Target angle and qRefIn angle        */
  /*==================================================*/
  setReferenceDataWithInterpolation();

  /*==================================================*/
  /* Set Target Angle Vector and publish from outport */
  /*==================================================*/
  setOutputData();

  loop ++;
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t PushRecover::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PushRecover::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PushRecover::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PushRecover::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PushRecover::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


bool PushRecover::assertEmergencyStop(void){
    if(emergencyStopReqFlag){
        return false;
    }else{
        emergencyStopReqFlag = true;
    }
    return true;
}

bool PushRecover::shiftPushRecoveryState(PushRecoveryState target_state){
    double start_ratio, goal_ratio;
    const double transition_time = 2.0;

    /* assure that interpolator is not currently working  */
    if(!transition_interpolator->isEmpty()){
        std::cerr << "[" << m_profile.instance_name << "] in " << __func__ << ", \"interpolator is currently working\"" << std::endl;
        return false;
    }

    Guard guard(m_mutex);

    /* set current interpolator ratio as start ratio to prevent interpolation jump */
    transition_interpolator->get(&start_ratio, true);

    switch(target_state){
    case PR_IDLE:
    case PR_TRANSITION_TO_IDLE:
        /* transit to idle state */
        //start_ratio = 1.0;
        goal_ratio  = 0.0;
        current_control_state = PR_TRANSITION_TO_IDLE;
        break;
    case PR_READY:
    case PR_TRANSITION_TO_READY:
        /* transit to ready state */
        //start_ratio = 0.0;
        goal_ratio  = 1.0;
        current_control_state = PR_TRANSITION_TO_READY;
        break;
    default:
        start_ratio = goal_ratio = 0.0;
        return false;
        break;
    }

    /* Setup interpolator */
    transition_interpolator->clear();
    transition_interpolator->set(&start_ratio);
    {
        /* Enable Interpolator */
        const bool interpolate_immediate_flag = true;
        transition_interpolator->go(&goal_ratio, transition_time, interpolate_immediate_flag);
    }
    return true;
}

void PushRecover::waitPushRecoveryStateReady(void){
    /* TODO */
    {
        //Guard guard(m_mutex_busy_state);
    }
}

bool PushRecover::startPushRecovery(void){
    bool result;
    std::cout << "[" << m_profile.instance_name << "] " << __func__ << std::endl;

    switch(current_control_state){
    case PR_IDLE:
        result = shiftPushRecoveryState(PR_READY);
        break;
    default:
        result = false;
        break;
    }

    /* Show Message when failed */
    if(!result){
        std::cout << "[" << m_profile.instance_name << "] " << __func__ << ", failed to start push recovery. Please try again later." << std::endl;
    }

    return result;
}

bool PushRecover::stopPushRecovery(void){
    bool result;
    std::cout << "[" << m_profile.instance_name << "] " << __func__ << std::endl;

    switch(current_control_state){
    case PR_IDLE:
    case PR_TRANSITION_TO_IDLE:
        /* nothing to do */
        result = true;
        break;
    case PR_READY:
    case PR_TRANSITION_TO_READY:
        result = shiftPushRecoveryState(PR_IDLE);
        break;
    case PR_BUSY:
        waitPushRecoveryStateReady(); /* sleep during state is busy */
        result = shiftPushRecoveryState(PR_IDLE);
        break;
    default:
        result = false;
        break;
    }

    /* Show Message when failed */
    if(!result){
        std::cout << "[" << m_profile.instance_name << "] " << __func__ << ", failed to stop push recovery." << std::endl;
    }

    return result;
}

extern "C"
{

  void PushRecoverInit(RTC::Manager* manager)
  {
    RTC::Properties profile(pushrecover_spec);
    manager->registerFactory(profile,
                             RTC::Create<PushRecover>,
                             RTC::Delete<PushRecover>);
  }

};



// RTC::ReturnCode_t PushRecover::onExecute(RTC::UniqueId ec_id)
// {
//   static int loop = 1;
//   if (loop%1000==0){
//       std::cout << "[" << m_profile.instance_name<< "] onExecute(" << ec_id << ")" << std::endl;
// #if 0
//       QzVec3 v1(1, 2, 3);
//       QzVec3 v2(4, 5, 6);
//       QzVec3 v3;
//       QzVec3 v4;
//       v3 = v1 + v2;
//       v4 = 3.0*v1 + 2.5*v2;
//       std::cout << "v3=[" << v3 << "]" << std::endl;
//       std::cout << "v4=[" << v4 << "]" << std::endl;
// #endif
//   }
// #if 0
//   if (loop%1000==0){
//       for(int i=0; i<m_robot->numJoints(); i++){
//           std::cout << "[" << m_profile.instance_name<< "] m_robot->joint(" << i << ")->q(" << m_robot->joint(i)->q << "), cur_ang=" << m_qCurrent.data[i] << std::endl;
//       }
//       for (unsigned int i=0; i<m_forceIn.size(); i++){
//         if ( m_force[i].data.length()==6 ) {
//             std::string sensor_name = m_forceIn[i]->name();
//             hrp::Vector3 data_p(m_force[i].data[0], m_force[i].data[1], m_force[i].data[2]);
//             hrp::Vector3 data_r(m_force[i].data[3], m_force[i].data[4], m_force[i].data[5]);
//             std::cout << "[" << m_profile.instance_name << "] force and moment [" << sensor_name << "]" << std::endl;
//             std::cout << "[" << m_profile.instance_name << "]   sensor force  = " << data_p.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N]" << std::endl;
//             std::cout << "[" << m_profile.instance_name << "]   sensor moment = " << data_r.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
//         }
//       }
//   }
// #endif
// #if 1
//   {
//       const float dcomx = 0.4f * cosf(((float)(loop%360))*3.14159265358939323826433f/180.0f);
//       const float dcomy = 0.4f * sinf(((float)(loop%360))*3.14159265358939323826433f/180.0f);
//       const Vec3 x0[] = {
//           Vec3( 0.0f, -0.095f, 0.0f ),
//           Vec3( 0.0f,    0.0f, 0.0f ),
//           Vec3( dcomx,    dcomy, 0.0f )
//       };

//       if(loop%8000==1){ /* 8000Frame = 16sec? Cycle */
//           std::cout << "[" << m_profile.instance_name << "] Calling StepForward start" << std::endl;
//           stpf.start(x0);
//           std::cout << "[" << m_profile.instance_name << "] Calling StepForward func" << std::endl;
//           frame_ref = 0;
//       }else{
//           //virtual void getTrajectoryFrame( const int index, Vec3& pref, Vec3& body_p, Vec3 &footl_p, Vec3 &footr_p )
//           Vec3 sf_pref;
//           Vec3 sf_body_p;
//           Vec3 sf_footl_p;
//           Vec3 sf_footr_p;
//           ITrajectoryGenerator* gen = stpf.getReady();
//           if(gen!=0){
//               gen->getTrajectoryFrame(frame_ref++,
//                                      sf_pref,
//                                      sf_body_p,
//                                      sf_footl_p,
//                                      sf_footr_p );
//               if(loop%500==0){
//                   std::cout << "[" << m_profile.instance_name << "] pref=" << sf_pref << "  @" << frame_ref << "frame" << std::endl;
//                   std::cout << "[" << m_profile.instance_name << "] body_p=" << sf_body_p << std::endl;
//                   std::cout << "[" << m_profile.instance_name << "] footl_p=" << sf_footl_p << std::endl;
//                   std::cout << "[" << m_profile.instance_name << "] footr_p=" << sf_footr_p << std::endl;
//               }
//           }else{
//               if(loop%500==0){
//                   std::cout << "[" << m_profile.instance_name << "] Failed to call stfp.getReady()" << std::endl;
//               }
//           }
//       }
//   }
// #endif
//   loop ++;
//   // check dataport input
//   for (unsigned int i=0; i<m_forceIn.size(); i++){
//       if ( m_forceIn[i]->isNew() ) {
//           m_forceIn[i]->read();
//       }
//       if ( m_ref_forceIn[i]->isNew() ) {
//             m_ref_forceIn[i]->read();
//       }
//   }
//   if (m_rpyIn.isNew()) {
//       m_rpyIn.read();
//   }
//   if (m_qCurrentIn.isNew()) {
//       m_qCurrentIn.read();
//   }
//   if (m_qRefIn.isNew()) {
//       m_qRefIn.read();
//       for ( int i = 0; i < m_robot->numJoints(); i++ ){
//           m_robot->joint(i)->q = m_qRef.data[i];
//       }
//   }
//   // m_robot->joint(m_robot->link("RLEG_JOINT4")->jointId)->q += offset_r;
//   // m_robot->joint(m_robot->link("LLEG_JOINT4")->jointId)->q += offset_l;
//   // m_robot->joint(m_robot->link("HEAD_JOINT2")->jointId)->q = m_robot->joint(m_robot->link("RLEG_JOINT4")->jointId)->q - offset_r;
//   // m_robot->joint(m_robot->link("HEAD_JOINT3")->jointId)->q = m_robot->joint(m_robot->link("LLEG_JOINT4")->jointId)->q - offset_l;

// #if 1
//   for ( int i = 0; i < m_robot->numJoints(); i++ ){
//     m_qRef.data[i] = m_robot->joint(i)->q;
//   }
// #endif
//   m_qRefOut.write();
//   return RTC::RTC_OK;
// }

