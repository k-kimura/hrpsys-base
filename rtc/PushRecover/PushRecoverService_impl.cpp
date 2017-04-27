// -*- tab-width : 4 ; mode : C++ ; indent-tabs-mode : nil -*-
#include <iostream>
#include "PushRecoverService_impl.h"
#include "PushRecover.h"

PushRecoverService_impl::PushRecoverService_impl()
{
}

PushRecoverService_impl::~PushRecoverService_impl()
{
}

CORBA::Boolean PushRecoverService_impl::dummy(const double a)
{
	return 1;
};

CORBA::Boolean PushRecoverService_impl::emergencyStop()
{
	//return m_autobalancer->emergencyStop();
	m_pushrecover->assertEmergencyStop();
	return 1;
};

CORBA::Boolean PushRecoverService_impl::startPushRecovery()
{
	return m_pushrecover->startPushRecovery();
};

CORBA::Boolean PushRecoverService_impl::stopPushRecovery()
{
	return m_pushrecover->stopPushRecovery();
};

CORBA::Boolean PushRecoverService_impl::startLogging()
{
	return m_pushrecover->startLogging();
};

CORBA::Boolean PushRecoverService_impl::stopLogging()
{
	return m_pushrecover->stopLogging();
};

CORBA::Boolean PushRecoverService_impl::enablePushDetect()
{
	return m_pushrecover->enablePushDetect();
};

CORBA::Boolean PushRecoverService_impl::disablePushDetect()
{
	return m_pushrecover->disablePushDetect();
};

void PushRecoverService_impl::pushrecover(PushRecover *i_pushrecover){
  m_pushrecover = i_pushrecover;
};

CORBA::Boolean PushRecoverService_impl::setPushDetectParam(const OpenHRP::PushRecoverService::PushDetectParam& i_param)
{
  return m_pushrecover->setPushDetectParam(i_param);
};

CORBA::Boolean PushRecoverService_impl::getPushDetectParam(OpenHRP::PushRecoverService::PushDetectParam& o_param)
{
  return m_pushrecover->getPushDetectParam(o_param);
};
