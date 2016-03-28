// -*-C++-*-
#ifndef PUSHRECOVERSVC_IMPL_H
#define PUSHRECOVERSVC_IMPL_H

#include "PushRecoverService.hh"

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
    //
    void pushrecover(PushRecover *i_pushrecover);
private:
    PushRecover *m_pushrecover;
};

#endif
