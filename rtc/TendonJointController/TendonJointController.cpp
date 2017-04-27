// -*- C++ -*-
/*!
 * @file  TendonJointController.cpp
 * @brief tendon joint controlling component
 * $Date$
 *
 * $Id$
 */

#include "TendonJointController.h"
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpUtil/MatrixSolvers.h>
#include <hrpModel/Sensor.h>

// Module specification
// <rtc-template block="module_spec">
static const char* tendonjointcontroller_spec[] =
  {
    "implementation_id", "TendonJointController",
    "type_name",         "TendonJointController",
    "description",       "Tendon Joint Controller",
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

TendonJointController::TendonJointController(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qRefIn("qRef", m_qRef),
    m_qRefOut("q", m_qRef),
    m_TendonJointControllerServicePort("TendonJointControllerService"),
    // </rtc-template>
    m_debugLevel(0)
{
  m_service0.absorber(this);
}

TendonJointController::~TendonJointController()
{
}



RTC::ReturnCode_t TendonJointController::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qRef", m_qRefIn);

  // Set OutPort buffer
  addOutPort("q", m_qRefOut);

  // Set OutPort buffer
  
  // Set service provider to Ports
  m_TendonJointControllerServicePort.registerProvider("service0", "TendonJointControllerService", m_service0);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_TendonJointControllerServicePort);
  
  // </rtc-template>

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

  // parameters for internal robot model
  m_robot = hrp::BodyPtr(new hrp::Body());
  if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(),
                               CosNaming::NamingContext::_duplicate(naming.getRootContext())
          )){
      std::cerr << "TJC failed to load model[" << prop["model"] << "] in "
                << m_profile.instance_name << std::endl;
      return RTC::RTC_ERROR;
  }

  // for ystleg
  TendonPairParam tp1, tp2;
  tp1.joint_names.push_back("RLEG_JOINT4");
  tp1.joint_names.push_back("RLEG_JOINT4_2");
  tp1.joint_names.push_back("RLEG_JOINT3");
  pair_params.push_back(tp1);
  tp2.joint_names.push_back("LLEG_JOINT4");
  tp2.joint_names.push_back("LLEG_JOINT4_2");
  tp2.joint_names.push_back("LLEG_JOINT3");
  pair_params.push_back(tp2);

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t TendonJointController::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TendonJointController::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TendonJointController::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t TendonJointController::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t TendonJointController::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t TendonJointController::onExecute(RTC::UniqueId ec_id)
{
  static int loop = 0;
  //if (loop%200==0) std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
#if 1
  if (loop%1000==0){
    //for(int i=0; 1; i++){
      std::cout << m_profile.instance_name<< ": m_robot->joint(" << 0 << ")->q(" << m_robot->joint(0)->q << ")" << std::endl;
      std::cout << m_profile.instance_name<< ": m_robot->joint(" << 1 << ")->q(" << m_robot->joint(1)->q << ")" << std::endl;
      std::cout << m_profile.instance_name<< ": m_robot->joint(" << 2 << ")->q(" << m_robot->joint(2)->q << ")" << std::endl;
      std::cout << m_profile.instance_name<< ": m_robot->joint(" << 3 << ")->q(" << m_robot->joint(3)->q << ")" << std::endl;
      std::cout << m_profile.instance_name<< ": m_robot->joint(" << 4 << ")->q(" << m_robot->joint(4)->q << ")" << std::endl;
      std::cout << m_profile.instance_name<< ": m_robot->joint(" << 5 << ")->q(" << m_robot->joint(5)->q << ")" << std::endl;
      std::cout << m_profile.instance_name<< ": m_robot->joint(" << 6 << ")->q(" << m_robot->joint(6)->q << ")" << std::endl;
      std::cout << m_profile.instance_name<< ": m_robot->joint(" << 7 << ")->q(" << m_robot->joint(7)->q << ")" << std::endl;
      std::cout << m_profile.instance_name<< ": m_robot->joint(" << 8 << ")->q(" << m_robot->joint(8)->q << ")" << std::endl;
      std::cout << m_profile.instance_name<< ": m_robot->joint(" << 9 << ")->q(" << m_robot->joint(9)->q << ")" << std::endl;
      std::cout << m_profile.instance_name<< ": m_robot->joint(" << 10 << ")->q(" << m_robot->joint(10)->q << ")" << std::endl;
      std::cout << m_profile.instance_name<< ": m_robot->joint(" << 11 << ")->q(" << m_robot->joint(11)->q << ")" << std::endl;
      std::cout << m_profile.instance_name<< ": m_robot->joint(" << 12 << ")->q(" << m_robot->joint(12)->q << ")" << std::endl;
      std::cout << m_profile.instance_name<< ": m_robot->joint(" << 13 << ")->q(" << m_robot->joint(13)->q << ")" << std::endl;
      //std::cout << m_profile.instance_name<< ": m_qRef.data[(" << i << "](" << m_qRef.data[i] << ")" << std::endl;
      //}
  }
#endif
  loop ++;
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
#if 0
  for ( int i = 0; i < pair_params.size(); i++ ){
    m_robot->joint(m_robot->link(pair_params[i].joint_names[1])->jointId)->q = m_robot->joint(m_robot->link(pair_params[i].joint_names[0])->jointId)->q - pair_params[i].offset;
    m_robot->joint(m_robot->link(pair_params[i].joint_names[0])->jointId)->q += pair_params[i].offset;
  }

  for ( int i = 0; i < m_robot->numJoints(); i++ ){
    m_qRef.data[i] = m_robot->joint(i)->q;
  }
#endif
  m_qRefOut.write();
  return RTC::RTC_OK;
}

bool TendonJointController::setTendonPairParam(const TendonJointControllerService::TendonPairParam& i_param)
{
  std::cerr << m_profile.instance_name<< ": setTendonPairParam" << std::endl;
  for ( int i = 0; i < pair_params.size(); i++ ){
    if ( pair_params[i].joint_names[0] == std::string(i_param.joint_names[0]) ) {
      pair_params[i].offset = i_param.offset;
      std::cerr << "   pair(" << pair_params[i].joint_names[0] << " " << pair_params[i].joint_names[1] << "), offset = " << pair_params[i].offset << "[rad]" << std::endl;
    }
  }
  return true;
}

bool TendonJointController::getTendonPairParam(TendonJointControllerService::TendonPairParam& i_param)
{
  std::cerr << m_profile.instance_name<< ": getTendonPairParam" << std::endl;
  // for ( int i = 0; i < pair_params.size(); i++ ){
  //   if ( pair_params[i].joint_names[0] == std::string(i_param.joint_names[0]) ) {
  //     i_param.offset = pair_params[i].offset;
  //   }
  // }
  return true;
}

/*
RTC::ReturnCode_t TendonJointController::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TendonJointController::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TendonJointController::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TendonJointController::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t TendonJointController::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

extern "C"
{

  void TendonJointControllerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(tendonjointcontroller_spec);
    manager->registerFactory(profile,
                             RTC::Create<TendonJointController>,
                             RTC::Delete<TendonJointController>);
  }

};


