// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
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

void PushRecoverService_impl::pushrecover(PushRecover *i_pushrecover){
	m_pushrecover = i_pushrecover;
};
