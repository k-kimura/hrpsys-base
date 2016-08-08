// -*-C++-*-
#ifndef ABSOLUTEFORCESENSORSERVICESVC_IMPL_H
#define ABSOLUTEFORCESENSORSERVICESVC_IMPL_H

#include "hrpsys/idl/TendonJointControllerService.hh"

using namespace OpenHRP;

class TendonJointController;

class TendonJointControllerService_impl 
  : public virtual POA_OpenHRP::TendonJointControllerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  TendonJointControllerService_impl();
  virtual ~TendonJointControllerService_impl();
  //
  CORBA::Boolean setTendonPairParam(const OpenHRP::TendonJointControllerService::TendonPairParam& i_param);
  CORBA::Boolean getTendonPairParam(OpenHRP::TendonJointControllerService::TendonPairParam_out i_param);
  //
  void absorber(TendonJointController *i_absorber);
private:
  TendonJointController *m_absorber;
};				 

#endif
