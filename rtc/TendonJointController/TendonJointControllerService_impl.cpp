// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include "TendonJointControllerService_impl.h"
#include "TendonJointController.h"

TendonJointControllerService_impl::TendonJointControllerService_impl() : m_absorber(NULL)
{
}

TendonJointControllerService_impl::~TendonJointControllerService_impl()
{
}

CORBA::Boolean TendonJointControllerService_impl::setTendonPairParam(const OpenHRP::TendonJointControllerService::TendonPairParam& i_param)
{
  return m_absorber->setTendonPairParam(i_param);
}

CORBA::Boolean TendonJointControllerService_impl::getTendonPairParam(OpenHRP::TendonJointControllerService::TendonPairParam_out i_param)
{
	i_param = new OpenHRP::TendonJointControllerService::TendonPairParam();
	i_param->joint_names.length(2);
	return m_absorber->getTendonPairParam(*i_param);
}

void TendonJointControllerService_impl::absorber(TendonJointController *i_absorber)
{
	m_absorber = i_absorber;
}

