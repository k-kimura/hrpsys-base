// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include <sys/time.h>
#include "StabilizerService_impl.h"
#include "Stabilizer.h"

double pre_time_second = 1000000000000000000000000000000000.0;

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

char *time_string()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *tm_ = localtime(&tv.tv_sec);
    static char time[20];
    sprintf(time, "%02d:%02d:%02d.%06d", tm_->tm_hour, tm_->tm_min, tm_->tm_sec, (int)tv.tv_usec);
    return time;
}

double now_time_second()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *tm_ = localtime(&tv.tv_sec);
	double time_sec;
	time_sec = tm_->tm_hour * 60.0 * 60.0 + tm_->tm_min * 60.0 + tm_->tm_sec + tv.tv_usec / (1000.0 * 1000.0);
    return time_sec;
}

void StabilizerService_impl::setSegwayParameter(const OpenHRP::StabilizerService::sgParam& i_param)
{
  // std::cerr << "time_string = ["
  // 		   << time_string()
  // 		   << "]" << std::endl;
  if ( now_time_second() - pre_time_second > 1.5 ) {
	  std::cerr << "setSegwayParameter costs much time !!!" << std::endl;
	  std::cerr << "second_diff = " << now_time_second() - pre_time_second << std::endl;
	  std::cerr << "pre_time_second = " << pre_time_second << std::endl;
	  std::cerr << "now_time_second = " << now_time_second() << std::endl;
  }
  pre_time_second = now_time_second();

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
