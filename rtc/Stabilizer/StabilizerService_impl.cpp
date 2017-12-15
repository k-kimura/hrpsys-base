// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include "StabilizerService_impl.h"
#include "Stabilizer.h"

StabilizerService_impl::StabilizerService_impl() : m_stabilizer(NULL)
{
}

StabilizerService_impl::~StabilizerService_impl()
{
}

void StabilizerService_impl::startStabilizer(void)
{
	m_stabilizer->startStabilizer();
}

void StabilizerService_impl::stopStabilizer(void)
{
	m_stabilizer->stopStabilizer();
}

void StabilizerService_impl::getParameter(OpenHRP::StabilizerService::stParam_out i_param)
{
  i_param = new OpenHRP::StabilizerService::stParam();
  return m_stabilizer->getParameter(*i_param);
};

void StabilizerService_impl::setParameter(const OpenHRP::StabilizerService::stParam& i_stp)
{
	m_stabilizer->setParameter(i_stp);
}

bool StabilizerService_impl::dummy()
{
	std::cout << "StabilizerService: " << std::endl;
}

void StabilizerService_impl::stabilizer(Stabilizer *i_stabilizer)
{
  m_stabilizer = i_stabilizer;
} 

void StabilizerService_impl::startTQStabilizer(void)
{
	m_stabilizer->startTQStabilizer();
}

void StabilizerService_impl::stopTQStabilizer(void)
{
	m_stabilizer->stopTQStabilizer();
}

void StabilizerService_impl::getTQSTParameter(OpenHRP::StabilizerService::tstParam_out i_param)
{
  //i_param = new OpenHRP::StabilizerService::tstParam();
  //return m_stabilizer->getTQSTParameter(*i_param);
};

void StabilizerService_impl::setTQSTParameter(const OpenHRP::StabilizerService::tstParam& i_stp)
{
	//m_stabilizer->setTQSTParameter(i_stp);
}

bool StabilizerService_impl::startLogging(void)
{
	return m_stabilizer->startLogging();
}
