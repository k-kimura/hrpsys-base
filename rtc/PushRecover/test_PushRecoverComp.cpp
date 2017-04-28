// -*- tab-width : 4 ; mode : C++ ; indent-tabs-mode : nil -*-
#include <stdexcept>
#include <rtm/RTC.h>
#include <rtm/Manager.h>
#include <rtm/ModuleManager.h>
#include<rtm/CorbaNaming.h>
#include<rtm/CorbaConsumer.h>
#include <iostream>
#include <string>
#include "PushRecover.h"

using namespace RTC;

void MyModuleInit(RTC::Manager* manager)
{
    std::cout << __func__ << ": Started" << std::endl;
    PushRecoverInit(manager);
    RTC::RtcBase* comp;

    // Create a component
    comp = manager->createComponent("PushRecover");

    return;
}

int test1(CORBA::ORB_var *corbaORB);

int main (int argc, char** argv)
{
    RTC::Manager* manager;
    PushRecover* comp_pr;
    manager = RTC::Manager::init(argc, argv);
    
    manager->init(argc, argv);
    
    manager->setModuleInitProc(MyModuleInit);
    
    manager->activateManager();
    
    manager->runManager(true);
    
    // Create a component
    std::vector<RTObject_impl* > comps = manager->getComponents();
    for(std::vector<RTObject_impl* >::iterator it = comps.begin(); it != comps.end(); ++it){
        std::cout << "type_name" << (*it)->get_component_profile()->type_name << std::endl;
        char* type_name = (char*)(*it)->get_component_profile()->type_name;
        if(strcmp(type_name,"PushRecover")==0){
            std::cout << "Found component[" << type_name << "]" << std::endl;
            comp_pr = (PushRecover*)manager->getComponent(((*it)->get_component_profile()->instance_name));
        }
    }

    CORBA::ORB_var orb;
    orb = CORBA::ORB_init(argc, argv);
    //test1(&orb);

    std::cout << "Sleep(5)" << std::endl;
    sleep(5);

    manager->shutdown();
    manager->terminate();

    return 0;
};

int test1(CORBA::ORB_var *corbaORB){
    //ネームサービスオブジェクト生成
    CorbaNaming corbaNaming(*corbaORB, "localhost:15005");

    CorbaConsumer<RTC::RTObject> pr0;
    CORBA::Object_ptr object_pr;
    std::cout << corbaNaming.getNameServer() << std::endl;
    try{
        object_pr = corbaNaming.resolve("PushRecover0.rtc");
    }catch(CosNaming::NamingContext::NotFound e){
        std::cerr << __func__ << "() notfound" << std::endl;
        return -1;
    }catch(...){
        std::cerr << "notfound aho" << std::endl;
        return -1;
    }
    pr0.setObject(object_pr);

    {
        RTC::ReturnCode_t ret;
        ExecutionContextList_var ExecutionContextList = pr0->get_owned_contexts();
        ret = ExecutionContextList[0]->activate_component(RTObject::_duplicate(pr0._ptr()));
        if(ret==RTC::RTC_OK){
            std::cout << __func__ << ": activated PR." << std::endl;
        }else{
            std::cerr << __func__ << ": failded to activated PR." << std::endl;
        }
    }
    return 0;
};
