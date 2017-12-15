// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#ifndef __KALMANFILTER_SERVICE_H__
#define __KALMANFILTER_SERVICE_H__

#include "hrpsys/idl/StabilizerService.hh"

class Stabilizer;

class StabilizerService_impl
	: public virtual POA_OpenHRP::StabilizerService,
	  public virtual PortableServer::RefCountServantBase
{
public:
	/**
	   \brief constructor
	*/
	StabilizerService_impl();

	/**
	   \brief destructor
	*/
	virtual ~StabilizerService_impl();

	void startStabilizer(void);
	void stopStabilizer(void);
	void getParameter(OpenHRP::StabilizerService::stParam_out i_param);
	void setParameter(const OpenHRP::StabilizerService::stParam& i_param);
	void stabilizer(Stabilizer *i_stabilizer);

	void startTQStabilizer(void);
	void stopTQStabilizer(void);
	void getTQSTParameter(OpenHRP::StabilizerService::tstParam_out i_param);
	void setTQSTParameter(const OpenHRP::StabilizerService::tstParam& i_param);
	bool startLogging(void);

	bool dummy();
private:
	Stabilizer *m_stabilizer;
};

#endif
