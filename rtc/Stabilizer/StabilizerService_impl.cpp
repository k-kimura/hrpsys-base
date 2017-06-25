// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include <sys/time.h>
#include "StabilizerService_impl.h"
#include "Stabilizer.h"

char *time_string()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  struct tm *tm_ = localtime(&tv.tv_sec);
  static char time[20];
  sprintf(time, "%02d:%02d:%02d.%06d", tm_->tm_hour, tm_->tm_min, tm_->tm_sec, (int)tv.tv_usec);
  return time;
}

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

void StabilizerService_impl::getSegwayParameter(OpenHRP::StabilizerService::sgParam_out i_param)
{
  i_param = new OpenHRP::StabilizerService::sgParam();
  return m_stabilizer->getSegwayParameter(*i_param);
};

void StabilizerService_impl::setSegwayParameter(const OpenHRP::StabilizerService::sgParam& i_param)
{
  std::cerr << "localtime before setSegwayParameter function: [" << time_string() << "]" << std::endl;
  return m_stabilizer->setSegwayParameter(i_param);
};

bool StabilizerService_impl::dummy()
{
	std::cout << "StabilizerService: " << std::endl;
}

void StabilizerService_impl::stabilizer(Stabilizer *i_stabilizer)
{
  m_stabilizer = i_stabilizer;
} 
