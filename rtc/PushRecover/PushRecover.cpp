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
      ee_params(2), /* Default number of End Effector is 2 */
// </rtc-template>
    m_debugLevel(0)
{
    m_service0.pushrecover(this);
    emergencyStopReqFlag = false;

    BOOST_STATIC_ASSERT( __alignof__(BodyIKMethod) == 16 );
    m_pIKMethod          = new BodyIKMethod( 0.0f, Zc );
    //slogger = boost::shared_ptr<SimpleLogger>(new SimpleLogger());
    slogger = new SimpleLogger();
}



PushRecover::~PushRecover()
{
    std::cout << "[pr] Destructor" << std::endl;
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
  m_dt_i = 1.0/m_dt;

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
  m_ref_forceOut.resize(nforce);
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

      // ref outport
      //m_ref_forceOut[i] = new OutPort<TimedDoubleSeq>(std::string(sensor_names[i]).c_str(), m_force[i]);
      m_ref_forceOut[i] = new OutPort<TimedDoubleSeq>(std::string("ref_" + fsensor_names[i]+"Out").c_str(), m_ref_force[i]);
      registerOutPort(std::string("ref_"+fsensor_names[i]+"Out").c_str(), *m_ref_forceOut[i]);

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

#if 0
  std::cout << "[pr] " << MAKE_CHAR_COLOR_RED << std::endl;
  for(int k=0; k<end_effectors_str.size(); k++){
      std::cout << "[pr] eeprop[" << k << "]" << end_effectors_str[k] << std::endl;
  }
  std::cout << MAKE_CHAR_COLOR_DEFAULT << std::endl;
#endif

  if (end_effectors_str.size() > 0) {
      const size_t num = end_effectors_str.size()/prop_num;
      /* Resize ee_params vector */
      if(ee_params.size() < num){
          ee_params.resize(num);
      }

      for (size_t i = 0; i < num; i++) {
          std::string ee_name, ee_target, ee_base;
          coil::stringTo(ee_name,   end_effectors_str[i*prop_num].c_str());
          coil::stringTo(ee_target, end_effectors_str[i*prop_num+1].c_str());
          coil::stringTo(ee_base,   end_effectors_str[i*prop_num+2].c_str());
          hrp::Vector3 ee_localp;
          for (size_t j = 0; j < 3; j++) {
              coil::stringTo(ee_localp(j), end_effectors_str[i*prop_num+3+j].c_str());
          }
          printf("[pr] oninitialize ee_localp:\n");
          std::cout << "[pr] " << ee_name << ": " << ee_target << ": " << std::endl;
          PRINTVEC3(ee_localp,true);
          double tmp_q[4];
          for (int j = 0; j < 4; j++ ) {
              coil::stringTo(tmp_q[j], end_effectors_str[i*prop_num+6+j].c_str());
          }
          hrp::Matrix33 ee_localR = Eigen::AngleAxis<double>(tmp_q[3], hrp::Vector3(tmp_q[0], tmp_q[1], tmp_q[2])).toRotationMatrix(); // rotation in VRM
          /* link endeffector name and index */
          ee_index_map.insert(std::pair<std::string, size_t>(ee_name, i));
          /* Save End Effector name indexed */
          {
              vector<std::string>::iterator it;
              std::string serching_fsensor_name;
              if(ee_name=="lleg"){
                  serching_fsensor_name = "lfsensor";
              }else if(ee_name=="rleg"){
                  serching_fsensor_name = "rfsensor";
              }else if(ee_name=="larm"){
                  serching_fsensor_name = "lasensor";
              }else if(ee_name=="rarm"){
                  serching_fsensor_name = "rasensor";
              }else{
                  serching_fsensor_name = "none";
              }
              std::cout << "[pr] " << MAKE_CHAR_COLOR_RED << "serching_fsensor_name=" << serching_fsensor_name << MAKE_CHAR_COLOR_DEFAULT << std::endl;
              /* serch for if expected sensor is exists */
              if(serching_fsensor_name!="none"){
                  it = find(fsensor_names.begin(),fsensor_names.end(), serching_fsensor_name);
                  if(it==fsensor_names.end()){
                      std::cout << "[" << m_profile.instance_name << "] Error " << serching_fsensor_name << " not found" << std::endl;
                      return RTC::RTC_ERROR;
                  }else{
                      ee_params[ee_index_map[ee_name]].fsensor_name = *it;
                  }
              }else{
                  ee_params[ee_index_map[ee_name]].fsensor_name = "none";
              }
              ee_params[ee_index_map[ee_name]].ee_name   = ee_name;
              ee_params[ee_index_map[ee_name]].ee_target = ee_target;
              ee_params[ee_index_map[ee_name]].ee_base   = ee_base;
              ee_params[ee_index_map[ee_name]].ee_localp = ee_localp;
              ee_params[ee_index_map[ee_name]].ee_localR = ee_localR;
              ee_params[ee_index_map[ee_name]].act_contact_state = false;
          }
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
  prev_ref_q = new double[m_robot->numJoints()];
  for(int i=0;i<m_robot->numJoints();i++){
      prev_ref_q[i] = ref_q[i] = 0.0;
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
                                                    body_p_default_offset,
#if 0
                                                    InitialLfoot_p - default_zmp_offset_l,
                                                    InitialRfoot_p - default_zmp_offset_r,
#else
                                                    InitialLfoot_p,
                                                    InitialRfoot_p,
#endif
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

  /* Initialize CoG velocity IIR filter */
  act_cogvel_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(4.0, m_dt, hrp::Vector3::Zero())); // [Hz]
  ref_zmp_modif_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(4.0, m_dt, hrp::Vector3::Zero())); // [Hz]
  ref_basePos_modif_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(4.0, m_dt, hrp::Vector3::Zero())); // [Hz]

  /* Initialize PR trajectory generator related values */
  trajectoryReset();

  dlog_save_flag = false;
  gettimeofday(&stv,NULL); /* save the time of initialized */

  prev_act_zmp      = hrp::Vector3::Zero();
  prev_rel_act_zmp  = hrp::Vector3::Zero();
  prev_act_root_pos = hrp::Vector3::Zero();
  prev_act_cog      = hrp::Vector3::Zero();
  prev_act_cogvel   = hrp::Vector3::Zero();
  act_base_rpy      = hrp::Vector3::Zero();
  ref_zmp           = hrp::Vector3::Zero();
  prev_ref_zmp      = hrp::Vector3::Zero();
  ref_basePos       = hrp::Vector3::Zero();
  ref_zmp_modif     = hrp::Vector3::Zero();
  ref_basePos_modif = hrp::Vector3::Zero();

  prev_act_foot_origin_rot = hrp::Matrix33::Identity();
  input_baseRot            = hrp::Matrix33::Identity();
  ref_baseRot              = hrp::Matrix33::Identity();

  /* Initialize PushDetector */
  pushDetector_state = PD_DISABLE;
  pushDetectParam.diff_margin_threshold_x = 70.0*70.0;
  pushDetectParam.diff_margin_threshold_y = (40.0+80.0)*(40.0+80.0);
  pushDetectParam.body_compliance_k1      = 0.03;
  pushDetectParam.body_compliance_k2      = -0.03;
  pushDetectParam.body_compliance_k3      = 0.1;

  loop = 1;

  if(m_robot->numJoints()!=12){
      std::cout << "[" << m_profile.instance_name << "] number of joint is expected 12." << std::endl;
      return RTC::RTC_ERROR;
  }else{
      return RTC::RTC_OK;
  }
}


RTC::ReturnCode_t PushRecover::onFinalize()
{
    std::cout << "[pr] onFinalize" << std::endl;
    delete ref_q;
    delete prev_ref_q;
    delete transition_interpolator;
    delete m_pIKMethod;
    delete slogger;
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

    if(shw_msg_flag){
        std::cout << "[pr] "  << __func__ << std::endl;
    }

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
        DEBUG_INPUT_PRINT(m_baseRpy.data.r);
        DEBUG_INPUT_PRINT(m_baseRpy.data.p);
        DEBUG_INPUT_PRINT(m_baseRpy.data.y);
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
        DEBUG_INPUT_PRINT(m_rpy.data.r);
        DEBUG_INPUT_PRINT(m_rpy.data.p);
        DEBUG_INPUT_PRINT(m_rpy.data.y);
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

    {
        if(shw_msg_flag){
            std::cout << "[pr] accRef=";
            printf("%lf, %lf, %lf\n", m_accRef.data.ax, m_accRef.data.ay, m_accRef.data.az);
            std::cout << "[pr] controlSwingSupportTime=";
            printf("%lf, %lf\n",m_controlSwingSupportTime.data[0],m_controlSwingSupportTime.data[1]);
            if(m_walkingStates.data){
                std::cout << "[pr] walkingstates=true" << std::endl;
            }else{
                std::cout << "[pr] walkingstates=false" << std::endl;
            }
            std::cout << "[pr] sbpCogOffsetIn=";
            printf("%lf, %lf, %lf\n",m_sbpCogOffset.data.x,m_sbpCogOffset.data.y,m_sbpCogOffset.data.z);
        }
    }

    if(shw_msg_flag){
        std::cout <<std::endl;
    }
#undef DEBUG_INPUT_PRINT
#undef DEBUG_INPUT_PRINT_I
};

void PushRecover::updateEstimatedInputData(void){
    //com_dph = hrp::Vector3(Zc * m_rpy.data.p, Zc * m_rpy.data.r, 0.0) + com_dph_base;
    est_cogvel = hrp::Vector3(Zc * m_rpy.data.p, Zc * m_rpy.data.r, 0.0);
};

void PushRecover::updateEstimatedOutputData(void){
};

void PushRecover::setTargetDataWithInterpolation(void){
    bool is_transition_interpolator_empty = transition_interpolator->isEmpty();
    if (!is_transition_interpolator_empty) {
        /* Interpolation is currently working */
        transition_interpolator->get(&transition_interpolator_ratio, true);
        //std::cout << "[pr] " << MAKE_CHAR_COLOR_RED << "transition_ratio=" << transition_interpolator_ratio << MAKE_CHAR_COLOR_DEFAULT << std::endl;
    } else {
        /* Interpolation is not currently working */
        if(current_control_state == PR_TRANSITION_TO_READY){
            current_control_state = PR_READY;
            transition_interpolator_ratio = 1.0; /* use controller output */

            /* Initialize PR trajectory generator related values */
            trajectoryReset();
            stpf.reset();

            /* Set Initial Base Frame Reference Position */
            m_robot->rootLink()->p = hrp::Vector3(traj_body_init[0], traj_body_init[1], traj_body_init[2]+InitialLfoot_p[2]);
            /* Set Initial Base Frame Reference Rotation */
            //m_robot->rootLink()->R = 

            /* Transition to state PR_READY needs to set default values */
            for ( int i = 0; i < m_robot->numJoints(); i++ ) {
                m_robot->joint(i)->q = ready_joint_angle[i];
            }
        }else if(current_control_state == PR_TRANSITION_TO_IDLE){
            transition_interpolator_ratio = 0.0; /* use input as output */
            current_control_state = PR_IDLE;

            /* Transition to state PR_READY needs to set default values */
            for ( int i = 0; i < m_robot->numJoints(); i++ ) {
                m_robot->joint(i)->q = m_qRef.data[i];
            }
            /* Initialize Default joint angle on PR_READY state */
            {
                _MM_ALIGN16 float target_joint_angle[12];
                _MM_ALIGN16 float pre_joint_angle[12];
                _MM_ALIGN16 Mat3 body_R = Mat3::identity();
                const float foot_l_pitch = 0.0f;
                const float foot_r_pitch = 0.0f;
                _MM_ALIGN16 Vec3 body_p = m_pIKMethod->calcik(body_R,
                                                              body_p_default_offset,
#if 0
                                                              InitialLfoot_p - default_zmp_offset_l,
                                                              InitialRfoot_p - default_zmp_offset_r,
#else
                                                              InitialLfoot_p,
                                                              InitialRfoot_p,
#endif
                                                              foot_l_pitch,
                                                              foot_r_pitch,
                                                              target_joint_angle );
                for(int i=0;i<12;i++){
                    ready_joint_angle[i] = target_joint_angle[i];
                }
            }
        }else{
            transition_interpolator_ratio = 1.0; /* use controller output */
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
            ref_basePos = (1-transition_interpolator_ratio) * input_basePos + transition_interpolator_ratio * act_world_root_pos;
            break;
        case PR_TRANSITION_TO_READY:
            ref_basePos = (1-transition_interpolator_ratio) * input_basePos + transition_interpolator_ratio * hrp::Vector3(traj_body_init[0], traj_body_init[1], traj_body_init[2]+InitialLfoot_p[2]);
            break;
        default:
            break;
        }

        /* Set Base Frame Reference Rotation */
        rats::mid_rot(ref_baseRot, transition_interpolator_ratio, input_baseRot, m_robot->rootLink()->R);

        /* set relative Reference ZMP */
        switch (current_control_state){
        case PR_TRANSITION_TO_IDLE:
            //rel_ref_zmp = (1-transition_interpolator_ratio) * input_zmp + transition_interpolator_ratio * hrp::Vector3(0.0f, 0.0f, -Zc);
            rel_ref_zmp = (1-transition_interpolator_ratio) * input_zmp + transition_interpolator_ratio * hrp::Vector3(-default_zmp_offset_l[0], 0.0f, -Zc);
            break;
        case PR_TRANSITION_TO_READY:
            //rel_ref_zmp = (1-transition_interpolator_ratio) * input_zmp + transition_interpolator_ratio * hrp::Vector3(0.0f, 0.0f, -Zc);
            rel_ref_zmp = (1-transition_interpolator_ratio) * input_zmp + transition_interpolator_ratio * hrp::Vector3(-default_zmp_offset_l[0], 0.0f, -Zc);
            break;
        default:
            break;
        }
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

    // reference acceleration
    hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
    if (sen != NULL) {
        hrp::Vector3 imu_sensor_pos = sen->link->p + sen->link->R * sen->localPos;
        hrp::Vector3 imu_sensor_vel = (imu_sensor_pos - prev_imu_sensor_pos)/m_dt;
        // convert to imu sensor local acceleration
        hrp::Vector3 acc = (sen->link->R * sen->localR).transpose() * (imu_sensor_vel - prev_imu_sensor_vel)/m_dt;
        m_accRef.data.ax = acc(0); m_accRef.data.ay = acc(1); m_accRef.data.az = acc(2);
        m_accRef.tm = m_qRef.tm;
        prev_imu_sensor_pos = imu_sensor_pos;
        prev_imu_sensor_vel = imu_sensor_vel;
    }

    /* set reference force */
    {
        const std::string legs_ee_name[2]={"lleg","rleg"};
        for (size_t i = 0; i < 2; i++) {
            const int eei = ee_index_map[legs_ee_name[i]];
            m_ref_force[eei].tm = m_qRef.tm;
            m_ref_force[eei].data[0] = ref_force[eei](0);
            m_ref_force[eei].data[1] = 0.0;
            m_ref_force[eei].data[2] = 0.0;
#if 0
            m_ref_force[eei].data[3] = 0.0;
            m_ref_force[eei].data[4] = 0.0;
            m_ref_force[eei].data[5] = 0.0;
#endif
        }
    }
}; /* end of setTargetDataWithInterpolation */

void PushRecover::setOutputData(const bool shw_msg_flag){
#define DEBUG_PRINT(x)  {if(shw_msg_flag) std::cout << "[pr] " << #x << "=" << x << std::endl;}

    if(shw_msg_flag){
        std::cout << "[pr] "  << __func__ << std::endl;
    }

    /*==================================================*/
    /* Set Target Angle Vector and publish from outport */
    /*==================================================*/
    if(shw_msg_flag) printf("[pr] q=[");
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
        m_qRef.data[i] = ref_q[i];
        prev_ref_q[i]  = ref_q[i];
        if(shw_msg_flag) printf("%+3.1lf",rad2deg(ref_q[i]));
        if(shw_msg_flag && (i==m_robot->numJoints()-1)){ printf("]\n");
        }else{
            if(shw_msg_flag) printf(", ");
        }
    }

    // basePos
    m_basePos.data.x = ref_basePos(0);
    m_basePos.data.y = ref_basePos(1);
    m_basePos.data.z = ref_basePos(2);
    m_basePos.tm = m_qRef.tm;
    PRINTVEC3(ref_basePos, shw_msg_flag);
    // baseRpy
    hrp::Vector3 baseRpy = hrp::rpyFromRot(ref_baseRot);
    m_baseRpy.data.r = baseRpy(0);
    m_baseRpy.data.p = baseRpy(1);
    m_baseRpy.data.y = baseRpy(2);
    m_baseRpy.tm = m_qRef.tm;
    PRINTVEC3(baseRpy, shw_msg_flag);

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
    prev_rel_ref_zmp = rel_ref_zmp;
    PRINTVEC3(rel_ref_zmp, shw_msg_flag);

    /* Write to OutPort */
    m_basePosOut.write();
    m_baseRpyOut.write();
    m_basePoseOut.write();
    m_qRefOut.write();
    m_zmpOut.write();

    // control parameters
    /* TODO */
    for (unsigned int i=0; i<m_ref_forceOut.size(); i++){
        m_ref_forceOut[i]->write();
    }
    m_accRefOut.write();
    m_contactStatesOut.write();
    m_controlSwingSupportTimeOut.write();
    m_walkingStatesOut.write();
    m_sbpCogOffsetOut.write();
    {
        if(shw_msg_flag){
            if(m_walkingStates.data){
                std::cout << "[pr] walkingstates=true" << std::endl;
            }else{
                std::cout << "[pr] walkingstates=false" << std::endl;
            }
            std::cout << "[pr] controlSwingSupportTime=";
            printf("%lf, %lf\n",m_controlSwingSupportTime.data[0],m_controlSwingSupportTime.data[1]);
            std::cout << "[pr] accRef=";
            printf("%lf, %lf, %lf\n", m_accRef.data.ax, m_accRef.data.ay, m_accRef.data.az);
            std::cout << "[pr] ref_force[0](right)=";
            printf("%lf, %lf, %lf\n",
                   m_ref_force[0].data[0],
                   m_ref_force[0].data[1],
                   m_ref_force[0].data[2]);
            std::cout << "[pr] ref_force[1](left) =";
            printf("%lf, %lf, %lf\n",
                   m_ref_force[1].data[0],
                   m_ref_force[1].data[1],
                   m_ref_force[1].data[2]);
            printf("[pr] sbpCogoft=[");
            printf("%+3.4lf, %+3.4lf, %+3.4lf]\n",m_sbpCogOffset.data.x,m_sbpCogOffset.data.y,m_sbpCogOffset.data.z);
            printf("\n");
        }
    }

    if(shw_msg_flag) printf("\n");

#undef DEBUG_PRINT
};

bool PushRecover::checkEmergencyFlag(void){
    if(emergencyStopReqFlag==true){
        std::cout << "[" << m_profile.instance_name << "] emergency Stop Activated!" << std::endl;
        emergencyStopReqFlag = false;
        return true;
    }
    return false;
};

bool PushRecover::calcWorldForceVector(void){
    const std::string legs_ee_name[2]={"lleg","rleg"};
    const double contact_decision_threshold_foot = 8.0; // [N]

    for (size_t i = 0; i < 2; i++) {
        const int eei = ee_index_map[legs_ee_name[i]];
        hrp::ForceSensor* sensor = m_robot->sensor<hrp::ForceSensor>(ee_params[eei].fsensor_name);
        hrp::Vector3 fsp = sensor->link->p + sensor->link->R * sensor->localPos;
        hrp::Matrix33 tmpR;
        rats::rotm3times(tmpR, sensor->link->R, sensor->localR);
        hrp::Vector3 nf = tmpR * hrp::Vector3(m_force[eei].data[0], m_force[eei].data[1], m_force[eei].data[2]);
        hrp::Vector3 nm = tmpR * hrp::Vector3(m_force[eei].data[3], m_force[eei].data[4], m_force[eei].data[5]);

        /* save force on each foot by sensor coords */
        world_sensor_ps[eei] = fsp;
        world_force_ps[eei]  = nf;
        world_force_ms[eei]  = nm;
#if 0
        if(loop%1000==0){
            std::cout << MAKE_CHAR_COLOR_RED << "[pr] calcWorldForceVector" << MAKE_CHAR_COLOR_DEFAULT << std::endl;
            printf("[pr] eei=[%d]=%s=%s\n",eei,legs_ee_name[i].c_str(), ee_params[eei].fsensor_name);
            const hrp::Vector3 fpmm = hrp::Vector3(fsp(0)*1000.0,fsp(1)*1000.0,fsp(2)*1000.0);
            PRINTVEC3(fpmm,true);
        }
#endif

        // calc ee-local COP
        hrp::Link* target = m_robot->link(ee_params[eei].ee_target);
        const hrp::Matrix33 eeR = target->R * ee_params[eei].ee_localR;
        const hrp::Vector3 ee_fsp = eeR.transpose() * (fsp - (target->p + target->R * ee_params[eei].ee_localp)); // ee-local force sensor pos EndEffector座標に変換
        nf = eeR.transpose() * nf;
        nm = eeR.transpose() * nm;
        // ee-local total moment and total force at ee position
        /* センサ値をワールド座標からEndEffector座標に変換*/

        prev_act_force_z[eei] = 0.85 * prev_act_force_z[eei] + 0.15 * nf(2); // filter, cut off 5[Hz]

        /* Determin contact state on each foot */
        if (prev_act_force_z[eei] < contact_decision_threshold_foot) {
            ee_params[eei].act_contact_state = false; /* in the air */
        }else{
            ee_params[eei].act_contact_state = true; /* on ground */
        }
    }
}; /* end of PushRecover::calcWorldForceVector */

/* Copied from Stabilizer.  */
void PushRecover::calcFootOriginCoords (hrp::Vector3& foot_origin_pos, hrp::Matrix33& foot_origin_rot){
  rats::coordinates leg_c[2], tmpc;
  hrp::Vector3 ez = hrp::Vector3::UnitZ();
  hrp::Vector3 ex = hrp::Vector3::UnitX();
  const std::string legs_ee_name[2]={"lleg","rleg"};
  for (size_t i = 0; i < 2; i++) {
      /* EndEffector pos (足平中心位置) 具体的には (ROBOT).confを参照 */
      const int           eei = ee_index_map[legs_ee_name[i]];
      hrp::Link*       target = m_robot->link(ee_params[eei].ee_target);
      const hrp::Vector3  eep = target->p + target->R * ee_params[eei].ee_localp;
#if 0
      if(loop%1500==0){
          PRINTVEC3(target->p, true);
          PRINTVEC3(ee_params[eei].ee_localp,true);
          PRINTVEC3(eep, true);
          std::cout << "[pr] tartget->R=" << std::endl << target->R << std::endl;
      };
#endif

      leg_c[eei].pos = eep;
      hrp::Vector3 xv1(target->R * ex);
      xv1(2)=0.0;
      xv1.normalize();
      hrp::Vector3 yv1(ez.cross(xv1));
      leg_c[eei].rot(0,0) = xv1(0); leg_c[eei].rot(1,0) = xv1(1); leg_c[eei].rot(2,0) = xv1(2);
      leg_c[eei].rot(0,1) = yv1(0); leg_c[eei].rot(1,1) = yv1(1); leg_c[eei].rot(2,1) = yv1(2);
      leg_c[eei].rot(0,2) = ez(0); leg_c[eei].rot(1,2) = ez(1); leg_c[eei].rot(2,2) = ez(2);
  }
  if (ee_params[ee_index_map["rleg"]].act_contact_state &&
      ee_params[ee_index_map["lleg"]].act_contact_state) {
      /* Both foots are on ground */
      rats::mid_coords(tmpc, 0.5, leg_c[0], leg_c[1]);
      foot_origin_pos = tmpc.pos;
      foot_origin_rot = tmpc.rot;
      foots_contact_states = BOTH_FOOTS;
  } else if (ee_params[ee_index_map["rleg"]].act_contact_state) {
      /* Right foot is on ground */
      foot_origin_pos = leg_c[ee_index_map["rleg"]].pos;
      foot_origin_rot = leg_c[ee_index_map["rleg"]].rot;
      foots_contact_states = RFOOT;
  } else if (ee_params[ee_index_map["lleg"]].act_contact_state){
      foot_origin_pos = leg_c[ee_index_map["lleg"]].pos;
      foot_origin_rot = leg_c[ee_index_map["lleg"]].rot;
      foots_contact_states = LFOOT;
  } else {
      /* Both foots are on the air */
      rats::mid_coords(tmpc, 0.5, leg_c[0], leg_c[1]);
      foot_origin_pos = tmpc.pos;
      foot_origin_rot = tmpc.rot;
      foots_contact_states = ON_AIR;
  }
};

/* Copied from Stabilizer */
/* ret_zmp is in world coords based on contacting foots */
bool PushRecover::calcZMP(hrp::Vector3& ret_zmp, const double zmp_z)
{
    const double contact_decision_threshold = 8.0; // [N]
    double tmpzmpx = 0.0;
    double tmpzmpy = 0.0;
    double tmpfz = 0.0, tmpfz2 = 0.0;

    const std::string legs_ee_name[2]={"lleg","rleg"};

    for (size_t i = 0; i < 2; i++) {
        const int eei = ee_index_map[legs_ee_name[i]];
        const hrp::Vector3 fsp = world_sensor_ps[eei];
        const hrp::Vector3 nf  = world_force_ps[eei];
        const hrp::Vector3 nm  = world_force_ms[eei];
        if(loop%1000==0){
            const hrp::Vector3 fpmm = hrp::Vector3(fsp(0)*1000.0,fsp(1)*1000.0,fsp(2)*1000.0);
            PRINTVEC3(fpmm,true);
            PRINTVEC3(nf,true);
            PRINTVEC3(nm,true);
        }

        tmpzmpx += nf(2) * fsp(0) - (fsp(2) - zmp_z) * nf(0) - nm(1);
        tmpzmpy += nf(2) * fsp(1) - (fsp(2) - zmp_z) * nf(1) + nm(0);
        tmpfz += nf(2); /* calc sum of force z */

        tmpfz2 += prev_act_force_z[eei]; /* calc sum of filtered force z in EE coordinates */
    }

    if (tmpfz2 < contact_decision_threshold) {
        ret_zmp = act_zmp;
        return false; // in the air
    } else {
        ret_zmp = hrp::Vector3(tmpzmpx / tmpfz, tmpzmpy / tmpfz, zmp_z);
        return true; // on ground
    }
};

bool PushRecover::calcActCoGVel(const hrp::Vector3 act_foot_origin_pos, const hrp::Matrix33 act_foot_origin_rot){
    // convert absolute (in st) -> root-link relative
    rel_act_zmp = m_robot->rootLink()->R.transpose() * (act_zmp - m_robot->rootLink()->p);

    // Actual foot_origin frame => ルートリンク基準座標から足先基準座標に変換
    act_zmp = act_foot_origin_rot.transpose() * (act_zmp - act_foot_origin_pos);
    act_cog = act_foot_origin_rot.transpose() * (act_cog - act_foot_origin_pos);

    //act_cogvel = foot_origin_rot.transpose() * act_cogvel;
    if (foots_contact_states != prev_foots_contact_states) {
        act_cogvel = (act_foot_origin_rot.transpose() * prev_act_foot_origin_rot) * act_cogvel;
    } else {
        act_cogvel = (act_cog - prev_act_cog)/m_dt;
    }

    act_cogvel = act_cogvel_filter->passFilter(act_cogvel);
};

bool PushRecover::calcActRootPos(const hrp::Vector3 act_foot_origin_pos, const hrp::Matrix33 act_foot_origin_rot){
    act_root_pos = act_foot_origin_rot.transpose() * (-act_foot_origin_pos);
    return true;
}

bool PushRecover::updateToCurrentRobotPose(void){
    // update by current joint angles
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
        m_robot->joint(i)->q = m_qCurrent.data[i];
    }
    // tempolary set root link origin.
    m_robot->rootLink()->p = hrp::Vector3::Zero();

    /* calc current robot root Posture */
    hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
    hrp::Matrix33 senR = sen->link->R * sen->localR;
    hrp::Matrix33 act_Rs(hrp::rotFromRpy(m_rpy.data.r, m_rpy.data.p, m_rpy.data.y));
    m_robot->rootLink()->R = act_Rs * (senR.transpose() * m_robot->rootLink()->R);
    m_robot->calcForwardKinematics(); /* FK on actual joint angle */
    act_base_rpy = hrp::rpyFromRot(m_robot->rootLink()->R);

    // world_force_ps, world_force_ms
    calcWorldForceVector();

    // foot base pos
    hrp::Vector3 act_foot_origin_pos;
    hrp::Matrix33 act_foot_origin_rot;
    calcFootOriginCoords(act_foot_origin_pos, act_foot_origin_rot);

    {
        dlog.act_foot_origin_pos = CONV_HRPVEC3(act_foot_origin_pos);
    }

    // cog
    act_cog = m_robot->calcCM();
    // zmp
    bool on_ground = calcZMP(act_zmp, act_foot_origin_pos(2));
    // cogvel
    calcActCoGVel(act_foot_origin_pos, act_foot_origin_rot);
    // act_root_pos
    calcActRootPos(act_foot_origin_pos, act_foot_origin_rot);

    /* save current values for next cycle */
    prev_act_foot_origin_rot  = act_foot_origin_rot;
    prev_act_cog              = act_cog;
    prev_act_cogvel           = act_cogvel;
    prev_foots_contact_states = foots_contact_states;
    prev_act_root_pos         = act_root_pos;

    return on_ground;
};


bool PushRecover::checkJointVelocity(void){
    bool ret = true;
#if 1
    for(int i=0; i<m_robot->numJoints(); i++){
        const double maxq  = m_robot->joint(i)->ulimit;
        const double minq  = m_robot->joint(i)->llimit;
        const double maxdq = m_robot->joint(i)->uvlimit;
        const double mindq = m_robot->joint(i)->lvlimit;
        if(m_robot->joint(i)->q > maxq){
            m_robot->joint(i)->q = maxq;
            ret = false;
        }
        if(m_robot->joint(i)->q < minq){
            m_robot->joint(i)->q = minq;
            ret = false;
        }
        double dq = (m_robot->joint(i)->q - prev_ref_q[i])/m_dt;
        if(dq > maxdq){
            dq = maxdq;
            m_robot->joint(i)->q = m_qCurrent.data[i] + dq*m_dt;
            ret = false;
        }
        if(dq < mindq){
            dq = mindq;
            m_robot->joint(i)->q = prev_ref_q[i] + dq*m_dt;
            ret = false;
        }
        m_robot->joint(i)->dq = dq;
    }
#endif
    return ret;
};

bool PushRecover::checkBodyPosMergin(const double threshold2, const int loop, const bool mask){
    double diff2;
    const double y_margin_gain = 3.0; /* 円領域ではなく、四角形領域で考える */
    /* mm単位での実root_posとref_root_pos誤差の自乗和で判定 */
#if 0
    diff2  = (act_root_pos(0) - prev_ref_basePos(0))*(act_root_pos(0) - prev_ref_basePos(0))*(1000.0*1000.0);
    diff2 += (act_root_pos(1) - prev_ref_basePos(1))*(act_root_pos(1) - prev_ref_basePos(1))*(1000.0*1000.0);
#elif 0
    diff2  = (act_root_pos(0) - (prev_ref_basePos(0) + prev_rel_ref_zmp(0)))*(act_root_pos(0) - (prev_rel_ref_zmp(0))) * (1000.0*1000.0);
    diff2 += (act_root_pos(1) - (prev_ref_basePos(1) + prev_rel_ref_zmp(1)))*(act_root_pos(1) - (prev_rel_ref_zmp(1))) * (1000.0*1000.0);
#elif 0   /* USE Actual ZMP */
    const double diff2x  = (rel_act_zmp(0) - prev_rel_ref_zmp(0))*(rel_act_zmp(0) - prev_rel_ref_zmp(0)) * (1000.0*1000.0);
    const double diff2y  = (rel_act_zmp(1) - prev_rel_ref_zmp(1))*(rel_act_zmp(1) - prev_rel_ref_zmp(1)) * (1000.0*1000.0);
#elif 1  /* USE Modified body_p */
    const double diff2x  = ref_basePos_modif(0) * ref_basePos_modif(0) * (1000.0*1000.0);
    const double diff2y  = ref_basePos_modif(1) * ref_basePos_modif(1) * (1000.0*1000.0);
#else /* 動いていないとき、act_root_posはdefault_zmp_offsetだけ動いているはずで、rel_ref_zmpは0,0を示すはずだからact_root_posを使うのではなく,act_zmpを使うのが正しい? */
    diff2  = (act_root_pos(0) - (prev_ref_basePos(0) + prev_rel_ref_zmp(0)))*(act_root_pos(0) - (prev_ref_basePos(0) + prev_rel_ref_zmp(0))) * (1000.0*1000.0);
    diff2 += (act_root_pos(1) - (prev_ref_basePos(1) + prev_rel_ref_zmp(1)))*(act_root_pos(1) - (prev_ref_basePos(1) + prev_rel_ref_zmp(1))) * (1000.0*1000.0);
#endif

    if(loop%1000==0){
    //if(0){
#if 0
        const float diff_x = (act_root_pos(0) - (prev_ref_basePos(0) + prev_rel_ref_zmp(0)))*1000.0;
        const float diff_y = (act_root_pos(1) - (prev_ref_basePos(1) + prev_rel_ref_zmp(1)))*1000.0;
#elif 0
        const float diff_x = (rel_act_zmp(0) - (prev_rel_ref_zmp(0)))*1000.0;
        const float diff_y = (rel_act_zmp(1) - (prev_rel_ref_zmp(1)))*1000.0;
        const float diff_z = (rel_act_zmp(2) + Zc) * 1000.0;
#elif 1
        const float diff_x = ref_basePos_modif(0) * 1000.0;
        const float diff_y = ref_basePos_modif(1) * 1000.0;
        const float diff_z = (rel_act_zmp(2) + Zc) * 1000.0;
#else
        const float diff_x = (act_root_pos(0) - (prev_rel_ref_zmp(0)))*1000.0;
        const float diff_y = (act_root_pos(1) - (prev_rel_ref_zmp(1)))*1000.0;
        const float diff_z = (act_root_pos(2) - Zc) * 1000.0;
#endif
        const double diff  = sqrt(diff2);
        std::cout << "[pr] diff=" << diff << "[mm]" << std::endl;
        std::cout << "[pr] diffv=[" << diff_x << ", " << diff_y << ", " << diff_z << "]" << std::endl;
        std::cout << "[pr] foots_contact_state=";
        switch (foots_contact_states) {
        case BOTH_FOOTS:
            std::cout << "BOTH_FOOT" << std::endl;
            break;
        case RFOOT:
            std::cout << "RFOOT" << std::endl;
            break;
        case LFOOT:
            std::cout << "LFOOT" << std::endl;
            break;
        case ON_AIR:
            std::cout << "ON AIR" << std::endl;
            break;
        }
        PRINTVEC3(act_root_pos, true);
        PRINTVEC3(prev_ref_basePos, true);
        PRINTVEC3(rel_act_zmp, true);
        PRINTVEC3(prev_rel_ref_zmp, true);
        PRINTVEC3(act_cogvel, true);
    }

#if 0 /* Circular Area */
    if(diff2>threshold2 && loop%250==0){
        std::cout << "[pr] " << MAKE_CHAR_COLOR_RED << "=====Invoking PushRecover=====" << MAKE_CHAR_COLOR_DEFAULT << std::endl;
    }
    return ((diff2>threshold2)?true:false) & mask;
#else /* Rectanglar Area */
    //if(diff2x>threshold2 && loop%250==0){
    if(diff2x>pushDetectParam.diff_margin_threshold_x && loop%250==0){
        std::cout << "[pr] " << MAKE_CHAR_COLOR_RED << "=====Invoking PushRecover by x=====" << MAKE_CHAR_COLOR_DEFAULT << std::endl;
    }
    /* y方向は足幅の分だけ余裕を持たせる */
    //if(diff2y>(threshold2 + 80.0*80.0) && loop%250==0){
    if(diff2y>pushDetectParam.diff_margin_threshold_y && loop%250==0){
        std::cout << "[pr] " << MAKE_CHAR_COLOR_RED << "=====Invoking PushRecover by y=====" << MAKE_CHAR_COLOR_DEFAULT << std::endl;
    }
    bool diff_flag;
#if 0
    if(diff2x>threshold2 || diff2y>threshold2*y_margin_gain){
        diff_flag = true;
    }else{
        diff_flag = false;
    }
#else
    if(diff2x>pushDetectParam.diff_margin_threshold_x || diff2y>pushDetectParam.diff_margin_threshold_y){
        diff_flag = true;
    }else{
        diff_flag = false;
    }
#endif

    return diff_flag & mask;
#endif
};

bool PushRecover::controlBodyCompliance(bool is_enable){
    double u;

    const double k[3] = {pushDetectParam.body_compliance_k1, pushDetectParam.body_compliance_k2, pushDetectParam.body_compliance_k3};
    const double maxdd = 0.5*m_dt; /* 0.5m/sec^2 */
    const double maxmodif = 0.1;
#if 1
    if(loop%1000==0) std::cout << "[pr] " << MAKE_CHAR_COLOR_RED << "controlBodyCompliance()" << MAKE_CHAR_COLOR_DEFAULT << std::endl;
    for(int i = 0; i<2; i++){
        u = k[0] * (rel_act_zmp(i) - prev_rel_ref_zmp(i)) + k[1] * (ref_basePos_modif(i) - 0.0f);
        if(loop%1000==0) std::cout << "[pr] zmp_diff=[" << rel_act_zmp(i) << " - " << prev_rel_ref_zmp(i) << "]" << std::endl;
        double prev_dx = bodyComplianceContext[i].prev_u;
        //act_cogvel;も使う？
        if(u > prev_dx + maxdd){
            u = prev_dx + maxdd;
        }else if(u < prev_dx - maxdd){
            u = prev_dx - maxdd;
        }
        double tmp_x = ref_basePos_modif(i) + u;
        if(tmp_x > maxmodif){
            tmp_x = maxmodif;
        }else if(tmp_x < -maxmodif){
            tmp_x = -maxmodif;
        }
        if(is_enable){
            ref_basePos_modif(i) = tmp_x;
            bodyComplianceContext[i].prev_u = u;
        }
    }
#endif

    if(loop%1000==0) std::cout << "[pr] u=[" << bodyComplianceContext[0].prev_u << ", " << bodyComplianceContext[1].prev_u << "]" << std::endl;
    if(loop%1000==0) std::cout << "[pr] modif=[" << ref_basePos_modif[0] << ", " << ref_basePos_modif[1] << "]" << std::endl;

    /* smoothing by filter */
    //ref_zmp_modif     = ref_zmp_modif_filter->passFilter(ref_zmp_modif);
    ref_zmp_modif     = hrp::Vector3(0.0f, 0.0f, 0.0f);
    ref_basePos_modif = ref_basePos_modif_filter->passFilter(ref_basePos_modif);

    return true;
}; /* controlBodyCompliance */

void PushRecover::trajectoryReset(void){
    act_world_root_pos   = hrp::Vector3(traj_body_init[0],
                                        traj_body_init[1],
                                        traj_body_init[2] + InitialLfoot_p[2]
                                        );
    body_p_at_start      = act_world_root_pos;
    body_p_diff_at_start = hrp::Vector3(0.0, 0.0, 0.0);
    prev_ref_basePos     = hrp::Vector3(traj_body_init[0],
                                        traj_body_init[1],
                                        traj_body_init[2] + InitialLfoot_p[2]
                                        );

    rel_ref_zmp       = hrp::Vector3(-default_zmp_offset_l[0], 0.0f, -prev_ref_basePos[2]);
    prev_rel_ref_zmp  = rel_ref_zmp;

    for(int i=0;i<2;i++){
        bodyComplianceContext[i].prev_u = 0.0;
    }
    ref_basePos_modif = hrp::Vector3::Zero();
    ref_basePos_modif_filter->reset(ref_basePos_modif);
    ref_zmp_modif     = hrp::Vector3::Zero();
    bodyPos_modif_at_start = hrp::Vector::Zero();

    prev_ref_traj.clear();
}

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t PushRecover::onExecute(RTC::UniqueId ec_id)
{
  bool start_RWG_flag;
  struct timeval tv;
  gettimeofday(&tv,NULL);

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

  /* zero clear dlog */
  memset(&dlog,0,sizeof(SimpleLogger::DataLog));

  /* check dataport input */
  {
      bool shw_msg_flag = loop%500==0?true:false;
      //updateInputData(shw_msg_flag);
      //updateInputData(true);
      updateInputData(false);
  }
  /* End of checking dataport input */

  Guard guard(m_mutex);

  /* Check if Emergency Stop Flag is active */
  start_RWG_flag = checkEmergencyFlag();

  /* calculate actual zmp, cog, cogvel, poture from joint_angle, force, rpy */
  bool on_ground = updateToCurrentRobotPose();

  /* calculate estimated cogvel from imu rate */
  updateEstimatedInputData();


  {
      dlog.act_zmp      = CONV_HRPVEC3(act_zmp);
      dlog.rel_act_zmp  = CONV_HRPVEC3(rel_act_zmp);
      dlog.act_cog      = CONV_HRPVEC3(act_cog);
      dlog.act_root_pos = CONV_HRPVEC3(act_root_pos);
      dlog.act_contact_state[0] = (float)((uint32_t)ee_params[ee_index_map["lleg"]].act_contact_state);
      dlog.act_contact_state[1] = (float)((uint32_t)ee_params[ee_index_map["rleg"]].act_contact_state);
  }

  // TODO set modified ref_basePos_modif and ref_zmp_modif
  {
      const bool do_body_compliance = ((current_control_state==PR_READY)||(current_control_state==PR_BUSY))?true:false;
      controlBodyCompliance(do_body_compliance);
  }

  if(current_control_state==PR_READY || current_control_state==PR_BUSY){
      const double threshold  = 70;
      const double threshold2 = (threshold*threshold);

      /* check the state */
      const bool  checkBodyPosflag = checkBodyPosMergin(threshold2, loop, on_ground & (pushDetector_state==PD_ENABLE));

#if 0
      const float diff_x = act_root_pos(0) - ref_basePos(0);
      const float diff_y = act_root_pos(1) - ref_basePos(1);
#else
      const float diff_x = act_root_pos(0) - prev_ref_basePos(0);
      const float diff_y = act_root_pos(1) - prev_ref_basePos(1);
#endif
      //const float diff_z = act_root_pos(2) - (Zc - InitialLfoot_p[2]); /* TODO 効果の検証 */
      const float diff_z = act_root_pos(2) - Zc; /* TODO 効果の検証 */
      const Vec3 x0[] = {
#if 0
          Vec3( ref_zmp_modif(0), ref_zmp_modif(1), 0.0f ),
#elif 1
          Vec3(0.0f, 0.0, 0.0f),
#else
          Vec3( traj_body_init[0], traj_body_init[1], 0.0f),
#endif
#if 1
          Vec3( ref_basePos_modif(0), ref_basePos_modif(1), diff_z ),
#elif 0
          Vec3( 0.0f,    0.0f, diff_z ),
#else
          Vec3( traj_body_init[0],  traj_body_init[1], diff_z),
#endif
#if 0
          Vec3( (float)act_cogvel(0), (float)act_cogvel(1), 0.0f)
#elif 1
          /* これでいいのだろうか */ /* 挙動的にはこっちが正しそう */
          Vec3( (float)act_cogvel(0), -(float)act_cogvel(1), 0.0f)
#else
          Vec3( 0.0f, 0.0f, 0.0f )
#endif
      };

#if 0
      PRINTVEC3(x0[0], (loop%1000==0));
      PRINTVEC3(x0[1], (loop%1000==0));
      PRINTVEC3(x0[2], (loop%1000==0));
#endif

      /* TODO set rootLink position default value */
      m_robot->rootLink()->p = hrp::Vector3(traj_body_init[0], traj_body_init[1], traj_body_init[2]+InitialLfoot_p[2]) + ref_basePos_modif;
      /* save current reference rootlink position for checkBodyPosMergin */
      prev_ref_basePos = m_robot->rootLink()->p;

      //m_robot->rootLink()->p = act_root_pos;にはならない．ここは理想的なposを入れる
      /* set body_p to m_robot loot link Rotation */
      //m_robot->rootLink()->R = input_baseRot; /* TODO */
      m_robot->rootLink()->R = hrp::Matrix33::Identity(); /* TODO */

      if((start_RWG_flag || checkBodyPosflag) && !m_walkingStates.data && on_ground && current_control_state != PR_BUSY){
      //if((start_RWG_flag || checkBodyPosflag) && !m_walkingStates.data && current_control_state != PR_BUSY){
          std::cout << "[" << m_profile.instance_name << "] " << MAKE_CHAR_COLOR_RED << "Calling StepForward start" << MAKE_CHAR_COLOR_DEFAULT << std::endl;
          /* Save Current base position */

          body_p_at_start = act_world_root_pos;
          body_p_diff_at_start = hrp::Vector3( diff_x, diff_y, diff_z );
          basePos_modif_at_start = ref_basePos_modif;

          stpf.start(x0);
          rate_matcher.setCurrentFrame(0);
          current_control_state = PR_BUSY;
      }

      /* Get reference trajectory */
      TrajectoryElement<Vec3e> ref_traj;
      {
          Vec3 sf_pref;
          Vec3 sf_body_p;
          Vec3 sf_footl_p;
          Vec3 sf_footr_p;
          ITrajectoryGenerator* gen = stpf.getReady();
          if(current_control_state == PR_BUSY && gen!=0){
              rate_matcher.incrementFrame();
              gen->getTrajectoryFrame(rate_matcher.getConvertedFrame(),
                                      sf_pref,
                                      sf_body_p,
                                      sf_footl_p,
                                      sf_footr_p );
              ref_traj.p       = sf_pref;
              ref_traj.body_p  = sf_body_p - basePos_modif_at_start;
              ref_traj.footl_p = sf_footl_p;
              ref_traj.footr_p = sf_footr_p;
          }else{
              ref_traj = prev_ref_traj;  /* keep current trajectory state */
          }
#if 0
          if(loop%1000==0){
              const unsigned int cf  = rate_matcher.getCurrentFrame();
              const unsigned int cfc = rate_matcher.getConvertedFrame();
              std::cout << "[pr] cf=[" << cf << "], cfc=[" << cfc << "]";
              if(current_control_state == PR_BUSY){
                  std::cout << " : PR_BUSY" << std::endl;
              }else{
                  std::cout << " : PR_READY" << std::endl;
              }
          }
#endif
      }

      /* calc the trajectory velocity dp and body_dp */
      {
          ref_traj.dp      = (ref_traj.p      - ((Vec3e)prev_ref_traj.p))      * m_dt_i;
          ref_traj.body_dp = (ref_traj.body_p - ((Vec3e)prev_ref_traj.body_p)) * m_dt_i;
      }
      prev_ref_traj = ref_traj;



      {
          /* todo : check the continuity of act_world_root_pos */
          /* calc body_p on world coords */
          act_world_root_pos = hrp::Vector3(ref_traj.body_p[0] + body_p_at_start(0),
                                            ref_traj.body_p[1] + body_p_at_start(1),
                                            ref_traj.body_p[2] + body_p_at_start(2));
          /* calc body_p on base frame coords */
          m_robot->rootLink()->p = hrp::Vector3(traj_body_init[0] + ref_traj.body_p[0],
                                                traj_body_init[1] + ref_traj.body_p[1],
                                                traj_body_init[2] + InitialLfoot_p[2] + ref_traj.body_p[2]);
          m_robot->rootLink()->p += ref_basePos_modif;
#if 0
          if(loop%1000==0)printf("[pr] todo pos\n");
          PRINTVEC3(act_world_root_pos,(loop%500==0));
          PRINTVEC3(m_robot->rootLink()->p,(loop%500==0));
#endif
          /* set body_p to m_robot loot link Rotation */
          m_robot->rootLink()->R = input_baseRot; /* TODO */

          /* calc Reference ZMP relative to base_frame(Loot link)  */
          const hrp::Vector3 default_zmp_offset(default_zmp_offset_l[0],default_zmp_offset_l[1],default_zmp_offset_l[2]);
          ref_zmp     = hrp::Vector3(ref_traj.p[0],ref_traj.p[1],ref_traj.p[2]) + ref_zmp_modif + default_zmp_offset;
          //rel_ref_zmp = (m_robot->rootLink()->R.transpose() * (ref_zmp - m_robot->rootLink()->p));
          rel_ref_zmp = (m_robot->rootLink()->R.transpose() * (ref_zmp - m_robot->rootLink()->p)) - default_zmp_offset;
          /* TODO rel_ref_zmpはSTではどういう座標として使っているのか */
          //rel_ref_zmp = (m_robot->rootLink()->R.transpose() * (ref_zmp - m_robot->rootLink()->p)) + default_zmp_offset;
      }


      {
          // 新しい target_joint_angleを計算
          _MM_ALIGN16 float target_joint_angle[12];
          _MM_ALIGN16 float pre_joint_angle[12];
          _MM_ALIGN16 Mat3 body_R = Mat3::identity();
          const float foot_l_pitch = 0.0f;
          const float foot_r_pitch = 0.0f;
          const Vec3 basePos_modif = Vec3(ref_basePos_modif(0),ref_basePos_modif(1),ref_basePos_modif(2));

          if(current_control_state == PR_BUSY){   /* controller main */
              _MM_ALIGN16 Vec3 body_p = m_pIKMethod->calcik(body_R,
                                                            body_p_default_offset + ref_traj.body_p + basePos_modif,
                                                            //body_p_default_offset + ref_traj.body_p + basePos_modif,
#if 0
                                                            InitialLfoot_p + ref_traj.footl_p - default_zmp_offset_l,
                                                            InitialRfoot_p + ref_traj.footr_p - default_zmp_offset_r,
#else
                                                            InitialLfoot_p + ref_traj.footl_p,
                                                            InitialRfoot_p + ref_traj.footr_p,
#endif
                                                            foot_l_pitch,
                                                            foot_r_pitch,
                                                            target_joint_angle );
          }else if(current_control_state == PR_READY){
              _MM_ALIGN16 Vec3 body_p = m_pIKMethod->calcik(body_R,
                                                            body_p_default_offset + ref_traj.body_p + basePos_modif,
#if 0
                                                            InitialLfoot_p + ref_traj.footl_p - default_zmp_offset_l,
                                                            InitialRfoot_p + ref_traj.footr_p - default_zmp_offset_r,
#else
                                                            InitialLfoot_p + ref_traj.footl_p,
                                                            InitialRfoot_p + ref_traj.footr_p,
#endif
                                                            foot_l_pitch,
                                                            foot_r_pitch,
                                                            target_joint_angle );
          }

          m_robot->calcForwardKinematics(); /* FK on target joint angle */

          /* set target_joint_angle */
          for(int i=0;i < 12; i++){
              m_robot->joint(i)->q = target_joint_angle[i];  /* rad to rad */
              ready_joint_angle[i] = target_joint_angle[i];
          }
      }

      { /* set reference force */
          const hrp::Vector3 footl_p_ee =
              hrp::Vector3(ref_traj.footl_p[0]+InitialLfoot_p[0],
                           ref_traj.footl_p[1]+InitialLfoot_p[1],
                           ref_traj.footl_p[2]);
          const hrp::Vector3 footr_p_ee =
              hrp::Vector3(ref_traj.footr_p[0]+InitialRfoot_p[0],
                           ref_traj.footr_p[1]+InitialRfoot_p[1],
                           ref_traj.footr_p[2]);
          double alpha = (ref_zmp - footr_p_ee).norm() / (footr_p_ee - footl_p_ee).norm();
          if (alpha>1.0) alpha = 1.0;
          if (alpha<0.0) alpha = 0.0;
          const double mg = m_robot->totalMass() * 9.80;
          ref_force[0](0) = alpha * mg;     /*ref_force right*/
          ref_force[1](0) = (1-alpha) * mg; /*ref_force left*/
#if 0
          const double mg2 = m_robot->totalMass() * (9.80 * 0.5);
          ref_force[ee_index_map["rleg"]](0) = mg2; /*ref_force right*/
          ref_force[ee_index_map["lleg"]](0) = mg2; /*ref_force left*/
          //m_sbpCogOffsetOut.write();
#endif
      }

      /* set current walking status */
      {
          /* TODO */
          if(current_control_state == PR_BUSY){
              m_contactStates.data[ee_index_map["rleg"]] = (abs(ref_traj.footr_p[2])<0.001)?true:false;
              m_contactStates.data[ee_index_map["lleg"]] = (abs(ref_traj.footl_p[2])<0.001)?true:false;
              m_walkingStates.data = true;
              /* controlSwingSupportTime is not used while double support period, 1.0 is neglected */
              //m_controlSwingSupportTime.data[ee_index_map["rleg"]] = 1.0;
              //m_controlSwingSupportTime.data[ee_index_map["lleg"]] = 1.0;
          }else if(current_control_state == PR_READY){
              /* set current walking status */
              m_contactStates.data[ee_index_map["rleg"]] = true;
              m_contactStates.data[ee_index_map["lleg"]] = true;
              m_walkingStates.data = false;
              /* controlSwingSupportTime is not used while double support period, 1.0 is neglected */
              m_controlSwingSupportTime.data[ee_index_map["rleg"]] = 1.0;
              m_controlSwingSupportTime.data[ee_index_map["lleg"]] = 1.0;
          }
      }

#if 1
      const int rwg_len_out = 4500; /* TODO */
      if(rate_matcher.getConvertedFrame()>rwg_len_out){
          // if(current_control_state==PR_BUSY){
          //     /* TODO reset basepos modif */
          //     for(int i=0;i<2;i++){
          //         bodyComplianceContext[i].prev_u = 0.0;
          //     }
          //     ref_basePos_modif = hrp::Vector3::Zero();
          //     ref_basePos_modif_filter->reset(ref_basePos_modif);
          //     ref_zmp_modif     = hrp::Vector3::Zero();
          // }
          stpf.reset();
          current_control_state = PR_READY;
      }
#endif

      /* Save log */
      {
          dlog.sf_pref          = CONV_VEC3(ref_traj.p);
          dlog.sf_body_p        = CONV_VEC3(ref_traj.body_p);
          dlog.sf_footl_p       = CONV_VEC3(ref_traj.footl_p);
          dlog.sf_footr_p       = CONV_VEC3(ref_traj.footr_p);
          dlog.ref_traj_dp      = CONV_VEC3(ref_traj.dp);
          dlog.ref_traj_body_dp = CONV_VEC3(ref_traj.body_dp);
      }

      const unsigned int cf = rate_matcher.getCurrentFrame();
      if(cf==1){
#if 1
          std::cout << "[" << m_profile.instance_name << "]";
          PRINTVEC3(ref_traj.p, true);
          std::cout << "[" << m_profile.instance_name << "]";
          PRINTVEC3(ref_traj.body_p, true);
          std::cout << "[" << m_profile.instance_name << "]";
          PRINTVEC3(ref_traj.footl_p, true);
          std::cout << "[" << m_profile.instance_name << "]";
          PRINTVEC3(ref_traj.footr_p, true);
#endif
          std::cout << "[" << m_profile.instance_name << "] body_p_@_start=" << body_p_at_start << std::endl;
          std::cout << "[" << m_profile.instance_name << "] rootLink_p=" << m_robot->rootLink()->p << std::endl;
      }

      /* Finally set rootlink pos to world */
      //m_robot->rootLink()->p = act_world_root_pos;
      m_robot->rootLink()->p = act_world_root_pos + ref_basePos_modif;

#if 0
      if(loop%500==0)printf("[pr] todo finally\n");
      PRINTVEC3(act_world_root_pos,(loop%500==0));
      PRINTVEC3(m_robot->rootLink()->p,(loop%500==0));
#endif
  }else if(current_control_state == PR_IDLE){
      for ( int i = 0; i < m_robot->numJoints(); i++ ) {
          m_robot->joint(i)->q = m_qRef.data[i]; /* pass through qRefIn joint angle */
      }
      m_robot->rootLink()->p = input_basePos;
      m_robot->rootLink()->R = input_baseRot;

      ref_force[0](0) = m_ref_force[0].data[0]; /*ref_force right*/
      ref_force[1](0) = m_ref_force[1].data[0]; /*ref_force left*/
  }else { /* Transition state */
      for ( int i = 0; i < m_robot->numJoints(); i++ ) {
          //m_robot->joint(i)->q = m_qCurrent.data[i];
          m_robot->joint(i)->q = prev_ref_q[i];
      }
      m_robot->rootLink()->p = input_basePos;
      m_robot->rootLink()->R = input_baseRot;
  }

  m_robot->calcForwardKinematics(); /* FK on target joint angle */
  ref_cog = m_robot->calcCM();

#if 0
  if(loop%12000==250) std::cout << CLEAR_CONSOLE << std::endl;
  if(loop%300==1){
      //std::cout << CLEAR_CONSOLE << MOVE_CURSOL << std::endl;
      std::cout << MOVE_CURSOL << std::endl;
      std::cout << "[pr] " << MAKE_CHAR_COLOR_GREEN << "SHOW STATE"<< MAKE_CHAR_COLOR_DEFAULT << std::endl;
      std::cout << "[" << m_profile.instance_name << "] rootLink_p=[";
      printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", m_robot->rootLink()->p(0), m_robot->rootLink()->p(1), m_robot->rootLink()->p(2));
      std::cout << "[" << m_profile.instance_name << "] act_zmp=[";
      printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", act_zmp(0), act_zmp(1), act_zmp(2));
      std::cout << "[" << m_profile.instance_name << "] rel_act_zmp=[";
      printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", rel_act_zmp(0), rel_act_zmp(1), rel_act_zmp(2));
      std::cout << "[" << m_profile.instance_name << "] rel_ref_zmp=[";
      printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", rel_ref_zmp(0), rel_ref_zmp(1), rel_ref_zmp(2));
      std::cout << "[" << m_profile.instance_name << "] ref_fource[0]=[";
      printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", ref_force[0](0), ref_force[0](1), ref_force[0](2));
      std::cout << "[" << m_profile.instance_name << "] ref_fource[1]=[";
      printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", ref_force[1](0), ref_force[1](1), ref_force[1](2));
      std::cout << "[" << m_profile.instance_name << "] act_rtp=[";
      printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", act_root_pos(0), act_root_pos(1), act_root_pos(2));
      std::cout << "[" << m_profile.instance_name << "] act_cog=[";
      printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", act_cog(0), act_cog(1), act_cog(2));
     std::cout << "[" << m_profile.instance_name << "] ref_cog=[";
      printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", ref_cog(0), ref_cog(1), ref_cog(2));
      std::cout << "[" << m_profile.instance_name << "] act_cogvel=[";
      printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", act_cogvel(0), act_cogvel(1), act_cogvel(2));
      std::cout << "[" << m_profile.instance_name << "] est_cogvel=[";
      printf("%+3.5lf, %+3.5lf, %+3.5lf]\n", est_cogvel(0), est_cogvel(1), est_cogvel(2));

      printf("[pr] contact_st=[");
      if(m_contactStates.data[ee_index_map["lleg"]]){
          printf("true , ");
      }else{
          printf("false, ");
      }
      if(m_contactStates.data[ee_index_map["rleg"]]){
          printf("true ]\n");
      }else{
          printf("false]\n");
      }
      printf("[pr] waling_st=[");
      if(m_walkingStates.data){
          printf("true ]\n");
      }else{
          printf("false]\n");
      }
      printf("[pr] swing_tim=[");
      printf("%+1.1lf, %+1.1lf]\n",
             m_controlSwingSupportTime.data[ee_index_map["rleg"]],
             m_controlSwingSupportTime.data[ee_index_map["lleg"]]);
      printf("[pr] sbpCogoft=[");
      printf("%+3.4lf, %+3.4lf, %+3.4lf]\n",m_sbpCogOffset.data.x,m_sbpCogOffset.data.y,m_sbpCogOffset.data.z);
      printf("\n");
      printf("[pr] target_q=[");
      for(int i=0;i< 12;i++){
          printf("%+3.2lf, ",rad2deg(m_robot->joint(i)->q));
      }
      printf("]\n");
      printf("[pr] prev_rfq=[");
      for(int i=0;i< 12;i++){
          printf("%+3.2lf, ",rad2deg(prev_ref_q[i]));
      }
      printf("]\n");
      printf("[pr] curren_q=[");
      for(int i=0;i< 12;i++){
          printf("%+3.2lf, ",rad2deg(m_qCurrent.data[i]));
      }
      printf("]\n");
      printf("[pr] prev_target_dq=[");
      for(int i=0;i< 12;i++){
          printf("%+3.2lf, ",rad2deg(m_robot->joint(i)->dq));
      }
      printf("]\n");
  }
#endif
#if 0
  if((!checkJointVelocity()) && loop%20==0){
      std::cout << "[pr] " << MAKE_CHAR_COLOR_RED << "ERROR Joint Velocity "<< MAKE_CHAR_COLOR_DEFAULT << std::endl;
      printf("[pr] target_q=[");
      for(int i=0;i< 12;i++){
          printf("%+3.2lf, ",rad2deg(m_robot->joint(i)->q));
      }
      printf("]\n");
      printf("[pr] prev_rfq=[");
      for(int i=0;i< 12;i++){
          printf("%+3.2lf, ",rad2deg(prev_ref_q[i]));
      }
      printf("[pr] curren_q=[");
      for(int i=0;i< 12;i++){
          printf("%+3.2lf, ",rad2deg(m_qCurrent.data[i]));
      }
      printf("]\n");
      printf("[pr] target_dq=[");
      for(int i=0;i< 12;i++){
          printf("%+3.2lf, ",rad2deg(m_robot->joint(i)->dq));
      }
      printf("]\n");
  }
#endif

  /*==================================================*/
  /* Interpolate Target angle and qRefIn angle        */
  /*==================================================*/
  setTargetDataWithInterpolation();

  /* Save log */
  if(true){
  //if(false){
      dlog.frame      = (float)rate_matcher.getCurrentFrame();
      dlog.loop       = (float)loop;
      dlog.sectime   = ((float)(tv.tv_sec-stv.tv_sec)) + ((float)(tv.tv_usec - stv.tv_usec)/1000000.0f);
      for(int i=0;i<12;i++){
          dlog.act_q[i]  = (float)rad2deg(m_qCurrent.data[i]);
          dlog.ref_q[i]  = (float)rad2deg(m_robot->joint(i)->q);
          dlog.ref_dq[i] = (float)rad2deg(m_robot->joint(i)->dq);
      };
      dlog.ref_zmp       = CONV_HRPVEC3(ref_zmp);
      dlog.rel_ref_zmp   = CONV_HRPVEC3(rel_ref_zmp);
      //dlog.ref_base_pos  = CONV_HRPVEC3(m_robot->rootLink()->p);
      dlog.ref_base_pos  = CONV_HRPVEC3(ref_basePos);
      dlog.ref_cog       = CONV_HRPVEC3(ref_cog);
      for(int i=0;i<6;i++){
          dlog.act_force_l[i] = (float)m_force[ee_index_map["lleg"]].data[i];
          dlog.act_force_r[i] = (float)m_force[ee_index_map["rleg"]].data[i];
          dlog.ref_force_l[i] = (float)m_ref_force[ee_index_map["lleg"]].data[i];
          dlog.ref_force_r[i] = (float)m_ref_force[ee_index_map["rleg"]].data[i];
      }
      dlog.contact_state[0] = (float)((int)m_contactStates.data[ee_index_map["lleg"]]);
      dlog.contact_state[1] = (float)((int)m_contactStates.data[ee_index_map["rleg"]]);
      dlog.walking_state    = (float)((int)m_walkingStates.data);
      dlog.controlSwingSupportTime[0] = (float)m_controlSwingSupportTime.data[ee_index_map["lleg"]];
      dlog.controlSwingSupportTime[0] = (float)m_controlSwingSupportTime.data[ee_index_map["rleg"]];
      dlog.sbpCogOffset[0] = m_sbpCogOffset.data.x;
      dlog.sbpCogOffset[1] = m_sbpCogOffset.data.y;
      dlog.sbpCogOffset[2] = m_sbpCogOffset.data.z;

#if 1
      dlog.act_cogvel         = CONV_HRPVEC3(act_cogvel);
      dlog.ref_zmp_modif      = CONV_HRPVEC3(ref_zmp_modif);
      dlog.ref_basePos_modif  = CONV_HRPVEC3(ref_basePos_modif);
      dlog.act_world_root_pos = CONV_HRPVEC3(act_world_root_pos);
#endif

      slogger->dump(&dlog);
  }

  /*==================================================*/
  /* Set Target Angle Vector and publish from outport */
  /*==================================================*/
  //const bool shw_debug_msg_outputdata = loop%500==0?true:false;;
  const bool shw_debug_msg_outputdata = ((loop%1000==0) || ((loop%20==0) && (current_control_state == PR_TRANSITION_TO_READY || current_control_state == PR_TRANSITION_TO_IDLE)));
  //setOutputData(shw_debug_msg_outputdata);
  setOutputData(false);

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
        result = shiftPushRecoveryState(PR_IDLE);
    case PR_TRANSITION_TO_READY:
        std::cout << "[" << m_profile.instance_name << "] " << __func__ << " Currently moving to start push recovery. Please try again later." << std::endl;
        result = false;
        break;
    case PR_BUSY:
        waitPushRecoveryStateReady(); /* sleep during state is busy */
        result = shiftPushRecoveryState(PR_IDLE);
        break;
    default:
        result = false;
        break;
    }

    disablePushDetect();

    /* Show Message when failed */
    if(!result){
        std::cout << "[" << m_profile.instance_name << "] " << __func__ << ", failed to stop push recovery." << std::endl;
    }

    return result;
}

bool PushRecover::startLogging(void){
    bool result = true;
    std::cout << "[" << m_profile.instance_name << "] " << __func__ << std::endl;

    //Guard guard(m_mutex);
    if(!slogger->isRunning()){
        if(slogger->startLogging(true)){
            dlog_save_flag = true;
            std::cout << "[" << m_profile.instance_name << "] " << __func__ << ": start logger" << std::endl;
            result = true;
        }else{
            std::cout << "[" << m_profile.instance_name << "] " << __func__ << ": could not start logger" << std::endl;
            dlog_save_flag = false;
            result         = false;
        }
    }else{
        dlog_save_flag = false;
        std::cout << "[" << m_profile.instance_name << "] " << __func__ << ": already running logger" << std::endl;
        result = false;
    }
    return result;
}

bool PushRecover::stopLogging(void){
    bool result;
    std::cout << "[" << m_profile.instance_name << "] " << __func__ << std::endl;

    Guard guard(m_mutex);
    if(slogger->isRunning()){
        slogger->stopLogging();
        dlog_save_flag = false;
        std::cout << "[" << m_profile.instance_name << "] " << __func__ << ": stopped logger" << std::endl;
        result = true;
    }else{
        std::cout << "[" << m_profile.instance_name << "] " << __func__ << ": There is no running logger" << std::endl;
        result = false;
    }

    return result;
}

bool PushRecover::enablePushDetect(void){
    std::cerr << "[" << m_profile.instance_name << "] " << MAKE_CHAR_COLOR_RED << "Enabling Push Detection" << MAKE_CHAR_COLOR_DEFAULT << std::endl;

    if(current_control_state == PR_READY){
        pushDetector_state = PD_ENABLE;
    }else{
        pushDetector_state = PD_DISABLE;
        return false;
    }

    return true;
}

bool PushRecover::disablePushDetect(void){
    std::cerr << "[" << m_profile.instance_name << "] " << MAKE_CHAR_COLOR_RED << "Disabling Push Detection" << MAKE_CHAR_COLOR_DEFAULT << std::endl;

    pushDetector_state = PD_DISABLE;

    return true;
}

bool PushRecover::setPushDetectParam(const OpenHRP::PushRecoverService::PushDetectParam& i_param)
{
    std::cerr << "[" << m_profile.instance_name << "] setPushDetectParam" << std::endl;
    pushDetectParam = i_param;

    std::cout << "[pr] diff_margin_threshold_x=[" << pushDetectParam.diff_margin_threshold_x << "]" << std::endl;
    std::cout << "[pr] diff_margin_threshold_y=[" << pushDetectParam.diff_margin_threshold_y << "]" << std::endl;
    std::cout << "[pr] body_compliance_k=[" << pushDetectParam.body_compliance_k1 << ", " << pushDetectParam.body_compliance_k2 << ", " << pushDetectParam.body_compliance_k3 << "]" << std::endl;

    return true;
}

bool PushRecover::getPushDetectParam(OpenHRP::PushRecoverService::PushDetectParam& o_param)
{
    std::cerr << "[" << m_profile.instance_name << "] getPushDetectParam" << std::endl;
    o_param.diff_margin_threshold_x = pushDetectParam.diff_margin_threshold_x;
    o_param.diff_margin_threshold_y = pushDetectParam.diff_margin_threshold_y;
    o_param.body_compliance_k1      = pushDetectParam.body_compliance_k1;
    o_param.body_compliance_k2      = pushDetectParam.body_compliance_k2;
    o_param.body_compliance_k3      = pushDetectParam.body_compliance_k3;

    return true;
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
