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
    m_qRefIn("qRef", m_qRef),
    m_qCurrentIn("qCurrent", m_qCurrent),
    m_rpyIn("rpy", m_rpy),
    m_qRefOut("q", m_qRef),
    m_PushRecoverServicePort("PushRecoverService"),
    // </rtc-template>
    m_debugLevel(0)
{
    //m_service0.absorber(this);
}

PushRecover::~PushRecover()
{
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

  // Set OutPort buffer
  addOutPort("q", m_qRefOut);

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
      m_forceIn[i] = new InPort<TimedDoubleSeq>(fsensor_names[i].c_str(), m_force[i]);
      m_force[i].data.length(6);
      registerInPort(fsensor_names[i].c_str(), *m_forceIn[i]);
      // ref inport
      m_ref_force[i].data.length(6);
      for (unsigned int j=0; j<6; j++) m_ref_force[i].data[j] = 0.0;
      m_ref_forceIn[i] = new InPort<TimedDoubleSeq>(std::string("ref_"+fsensor_names[i]+"In").c_str(), m_ref_force[i]);
      registerInPort(std::string("ref_"+fsensor_names[i]+"In").c_str(), *m_ref_forceIn[i]);
      std::cerr << "[" << m_profile.instance_name << "]   name = " << fsensor_names[i] << std::endl;
  }
  for (unsigned int i=0; i<m_forceIn.size(); i++){
      abs_forces.insert(std::pair<std::string, hrp::Vector3>(m_forceIn[i]->name(), hrp::Vector3::Zero()));
      abs_moments.insert(std::pair<std::string, hrp::Vector3>(m_forceIn[i]->name(), hrp::Vector3::Zero()));
  }
  /* Finish Setting Force Sensor Port */

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t PushRecover::onFinalize()
{
  return RTC::RTC_OK;
}
*/

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
    return RTC::RTC_OK;
}

RTC::ReturnCode_t PushRecover::onDeactivated(RTC::UniqueId ec_id)
{
    std::cout << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
    return RTC::RTC_OK;
}

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t PushRecover::onExecute(RTC::UniqueId ec_id)
{
  static int loop = 1;
  if (loop%1000==0) std::cout << "[" << m_profile.instance_name<< "] onExecute(" << ec_id << ")" << std::endl;
#if 0
  if (loop%1000==0){
      for(int i=0; i<m_robot->numJoints(); i++){
          std::cout << "[" << m_profile.instance_name<< "] m_robot->joint(" << i << ")->q(" << m_robot->joint(i)->q << "), cur_ang=" << m_qCurrent.data[i] << std::endl;
      }
      for (unsigned int i=0; i<m_forceIn.size(); i++){
        if ( m_force[i].data.length()==6 ) {
            std::string sensor_name = m_forceIn[i]->name();
            hrp::Vector3 data_p(m_force[i].data[0], m_force[i].data[1], m_force[i].data[2]);
            hrp::Vector3 data_r(m_force[i].data[3], m_force[i].data[4], m_force[i].data[5]);
            std::cout << "[" << m_profile.instance_name << "] force and moment [" << sensor_name << "]" << std::endl;
            std::cout << "[" << m_profile.instance_name << "]   sensor force  = " << data_p.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N]" << std::endl;
            std::cout << "[" << m_profile.instance_name << "]   sensor moment = " << data_r.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
        }
      }
  }
#endif
  loop ++;
  // check dataport input
  for (unsigned int i=0; i<m_forceIn.size(); i++){
      if ( m_forceIn[i]->isNew() ) {
          m_forceIn[i]->read();
      }
      if ( m_ref_forceIn[i]->isNew() ) {
            m_ref_forceIn[i]->read();
      }
  }
  if (m_rpyIn.isNew()) {
      m_rpyIn.read();
  }
  if (m_qCurrentIn.isNew()) {
      m_qCurrentIn.read();
  }
  if (m_qRefIn.isNew()) {
      m_qRefIn.read();
      for ( int i = 0; i < m_robot->numJoints(); i++ ){
          m_robot->joint(i)->q = m_qRef.data[i];
      }
  }
  // m_robot->joint(m_robot->link("RLEG_JOINT4")->jointId)->q += offset_r;
  // m_robot->joint(m_robot->link("LLEG_JOINT4")->jointId)->q += offset_l;
  // m_robot->joint(m_robot->link("HEAD_JOINT2")->jointId)->q = m_robot->joint(m_robot->link("RLEG_JOINT4")->jointId)->q - offset_r;
  // m_robot->joint(m_robot->link("HEAD_JOINT3")->jointId)->q = m_robot->joint(m_robot->link("LLEG_JOINT4")->jointId)->q - offset_l;

#if 1
  for ( int i = 0; i < m_robot->numJoints(); i++ ){
    m_qRef.data[i] = m_robot->joint(i)->q;
  }
#endif
  m_qRefOut.write();
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


