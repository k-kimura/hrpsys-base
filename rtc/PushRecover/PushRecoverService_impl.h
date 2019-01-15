// -*- tab-width : 4 ; mode : C++ ; indent-tabs-mode : nil -*-
#ifndef PUSHRECOVERSVC_IMPL_H
#define PUSHRECOVERSVC_IMPL_H

#include "hrpsys/idl/PushRecoverService.hh"

using namespace OpenHRP;

class PushRecover;

class PushRecoverService_impl
    : public virtual POA_OpenHRP::PushRecoverService,
      public virtual PortableServer::RefCountServantBase
{
public:
    PushRecoverService_impl();
    virtual ~PushRecoverService_impl();
    CORBA::Boolean dummy(const double a);
    CORBA::Boolean emergencyStop();
    CORBA::Boolean startPushRecovery();
    CORBA::Boolean stopPushRecovery();
    CORBA::Boolean startLogging();
    CORBA::Boolean stopLogging();
    CORBA::Boolean enablePushDetect();
    CORBA::Boolean disablePushDetect();
    CORBA::Boolean setPushDetectParam(const OpenHRP::PushRecoverService::PushDetectParam& i_param);
    CORBA::Boolean getPushDetectParam(OpenHRP::PushRecoverService::PushDetectParam_out o_param);
    CORBA::Boolean setOnlineWalkParam(const OpenHRP::PushRecoverService::OnlineWalkParam& i_param);
    CORBA::Boolean getOnlineWalkParam(OpenHRP::PushRecoverService::OnlineWalkParam_out o_param);
    CORBA::Boolean setWheelMode(const CORBA::Long mode);
    CORBA::Boolean setWheelControllerParam(const OpenHRP::PushRecoverService::WheelControllerParamSet& i_param);
    CORBA::Boolean getWheelControllerParam(OpenHRP::PushRecoverService::WheelControllerParamSet_out o_param);
    //
    void pushrecover(PushRecover *i_pushrecover);
private:
    PushRecover *m_pushrecover;
};

#endif
