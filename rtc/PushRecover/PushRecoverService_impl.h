// -*-C++-*-
#ifndef PUSHRECOVERSVC_IMPL_H
#define PUSHRECOVERSVC_IMPL_H

//#include "PushRecover.h"
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
  //
};

#endif
