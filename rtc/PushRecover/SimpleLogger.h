// -*- tab-width : 4 ; mode : C++ ; indent-tabs-mode : nil -*-
#ifndef __SIMPLE_LOGGER_H__
#define __SIMPLE_LOGGER_H__
#include <stdlib.h>
#include <iostream>
#include <time.h>
#include <boost/circular_buffer.hpp> //for data logger
#include <pthread.h>

#include <coil/Guard.h>

#define MOVE_CURSOLN(x)         "\x1b["<< #x <<";0H"
#define CLEAR_CONSOLE           "\x1b[2J"
#define MOVE_CURSOL             "\x1b[2;0H"
#define MAKE_CHAR_DEFAULT       "\x1b[0m"
#define MAKE_CHAR_BOLD          "\x1b[1m"
#define MAKE_CHAR_COLOR_BLACK   "\x1b[30m"
#define MAKE_CHAR_COLOR_RED     "\x1b[31m"
#define MAKE_CHAR_COLOR_GREEN   "\x1b[32m"
#define MAKE_CHAR_COLOR_YELLOW  "\x1b[33m"
#define MAKE_CHAR_COLOR_BLUE    "\x1b[34m"
#define MAKE_CHAR_COLOR_MAGENTA "\x1b[35m"
#define MAKE_CHAR_COLOR_CYAN    "\x1b[36m"
#define MAKE_CHAR_COLOR_WHITE   "\x1b[37m"
#define MAKE_CHAR_COLOR_DEFAULT "\x1b[39m"
#define MAKE_BACK_COLOR_BLACK   "\x1b[40m"
#define MAKE_BACK_COLOR_RED     "\x1b[41m"
#define MAKE_BACK_COLOR_GREEN   "\x1b[42m"
#define MAKE_BACK_COLOR_YELLOW  "\x1b[43m"
#define MAKE_BACK_COLOR_BLUE    "\x1b[44m"
#define MAKE_BACK_COLOR_MAGENTA "\x1b[45m"
#define MAKE_BACK_COLOR_CYAN    "\x1b[46m"
#define MAKE_BACK_COLOR_WHITE   "\x1b[47m"
#define MAKE_BACK_COLOR_DEFAULT "\x1b[49m"

#define PRED MAKE_CHAR_COLOR_RED
#define PGRE MAKE_CHAR_COLOR_GREEN
#define PBLU MAKE_CHAR_COLOR_BLUE
#define PDEF MAKE_CHAR_COLOR_DEFAULT

#define PACKING __attribute__((__packed__))

namespace dlog {
    /* TODO check alignment */
    struct V3 {
        float x;
        float y;
        float z;
        V3(){
            x = y = z = 0.0f;
        };
        V3(float _x, float _y, float _z) : x(_x),y(_y),z(_z){
        };
        V3(const Vec3& v) : x(v[0]), y(v[1]), z(v[2]){
        };
    }PACKING;
    //typedef hrp::Vector3 V3;
    struct DataLog {
        float   sectime;
        float   frame;
        float   loop;
        float   act_q[12];
        float   ref_q[12];
        float   ref_dq[12];
        V3       act_zmp;
        V3       rel_act_zmp;
        V3       ref_zmp;
        V3       rel_ref_zmp;
        V3       act_root_pos;
        V3       ref_base_pos;
        V3       act_cog;
        V3       ref_cog;
        V3       act_foot_origin_pos;
        V3       ref_foot_origin_pos_l;
        V3       ref_foot_origin_pos_r;
        V3       sf_pref;
        V3       sf_body_p;
        V3       sf_footl_p;
        V3       sf_footr_p;
        float   act_force_l[6];
        float   act_force_r[6];
        float   ref_force_l[6];
        float   ref_force_r[6];
        float   act_contact_state[2];
        float   contact_state[2];
        float   walking_state;
        float   controlSwingSupportTime[2];
        float   sbpCogOffset[3];
        V3      act_cogvel;
        V3      ref_zmp_modif;
        V3      ref_basePos_modif;
        V3      act_world_root_pos;
        V3      ref_traj_dp;
        V3      ref_traj_body_dp;
        V3      rpy;
        V3      filtered_rot;
        V3      lpf_rot;
        V3      rot_offset;
        V3      ref_force[2];
        float   tau_ref[12];
    }PACKING;
    struct DataLog_20171023 {
        float   sectime;
        float   frame;
        float   loop;
        float   act_q[12];
        float   ref_q[12];
        float   ref_dq[12];
        V3       act_zmp;
        V3       rel_act_zmp;
        V3       ref_zmp;
        V3       rel_ref_zmp;
        V3       act_root_pos;
        V3       ref_base_pos;
        V3       act_cog;
        V3       ref_cog;
        V3       act_foot_origin_pos;
        V3       ref_foot_origin_pos_l;
        V3       ref_foot_origin_pos_r;
        V3       sf_pref;
        V3       sf_body_p;
        V3       sf_footl_p;
        V3       sf_footr_p;
        float   act_force_l[6];
        float   act_force_r[6];
        float   ref_force_l[6];
        float   ref_force_r[6];
        float   act_contact_state[2];
        float   contact_state[2];
        float   walking_state;
        float   controlSwingSupportTime[2];
        float   sbpCogOffset[3];
        V3      act_cogvel;
        V3      ref_zmp_modif;
        V3      ref_basePos_modif;
        V3      act_world_root_pos;
        V3      ref_traj_dp;
        V3      ref_traj_body_dp;
        V3      rpy;
        V3      filtered_rot;
        V3      lpf_rot;
        V3      rot_offset;
        V3      ref_force[2];
        float   tau_ref[12];
        V3      abs_zmp;
        V3      abs_rel_act_zmp;
        V3      abs_contact_state;
        V3      abs_rate;
        V3      abs_foot_origin_pos;
        V3      abs_foot_origin_rpy;
        V3      abs_body_p;
        V3      abs_body_v;
        V3      abs_cog_p;
        V3      abs_cog_v;
        V3      accIn;
        V3      x_offset;
        V3      x_offset_orig;
    }PACKING;
}

template<class Dlog, bool use_float>
class data_logger_online {
private:
  boost::circular_buffer<Dlog> buf;
  bool dumping_flag;
  pthread_t dlog_thread;
  static void* dlog_thread_fun(void* arg);
  float progress;
public:
  data_logger_online(const int size) : buf(size), dumping_flag(false), progress(0.0){};
  ~data_logger_online(){
    if(dumping_flag){
      if ( pthread_join ( dlog_thread, NULL ) ) {
	/* スレッドのjoinに失敗 */
	std::cerr << "[data_logger_online] failed to join pthread." << std::endl;
      }
    }
  };
  bool startDumpFile(void){
    bool ret;
    if(dumping_flag){
      /* Currently running dlog thread. */
      std::cerr << "[data_logger_online] already running dumping." << std::endl;
      ret = false;
    }else if(pthread_create(&dlog_thread, NULL, dlog_thread_fun, this)){
      std::cerr << "[data_logger_online] failed to create pthread." << std::endl;
      ret = false;
    }else{
      dumping_flag=true;
      ret = true;
    }
    return ret;
  };
  void push(Dlog &v){
    if(!dumping_flag){
      /* push data if not dummping data into file. */
      buf.push_back(v);
    }
  }; /*push()*/
  float get_progress(void){
    if(dumping_flag){
      return progress;
    }else{
      return -1.0;
    }
  }; /*get_progress()*/
};

template<class Dlog, bool use_float>
void* data_logger_online<Dlog,use_float>::dlog_thread_fun(void* arg){
  data_logger_online<Dlog,use_float>* self = (data_logger_online<Dlog,use_float>*)arg;

  /* To push this thread in Low-Priority */
  sched_param param;
  param.sched_priority = 0;
  if( sched_setscheduler( 0, SCHED_OTHER, &param ) == -1 ){
    printf("Error sched_setscheduler()\n");
  }

  /* Open DataLog File  */
  FILE *fp;
  //char* homedir = getenv("HOME");
  char* homedir = "/home/leus";
  char filename[256], cur_time_buf[14];
  //time_t now = time(NULL);
  //struct tm *pnow = localtime(&now);

  //printf("Openinig datalog.dat\n");
  //Get current time
  //sprintf(cur_time_buf, "%04d%02d%02d%02d%02d%02d", pnow->tm_year + 1900, pnow->tm_mon + 1, pnow->tm_mday, pnow->tm_hour, pnow->tm_min, pnow->tm_sec);
  sprintf(filename,"%s/%s/%s",homedir,"log","datalog.dat");
  //printf("%s\n",filename);
  if((fp = fopen(filename,"w"))==NULL){
    fprintf(stderr,"Error Cannot Open %s\n",filename);
    self->dumping_flag=false;
    return 0;
    //exit(EXIT_FAILURE);
  }
  fprintf(fp,"#global_clock(1)");
  fprintf(fp,"\n");

#if 1
  Dlog *v = self->buf.linearize();
  int sizeof_element;
  if(use_float){
    sizeof_element = sizeof(float);
  }else{
    sizeof_element = sizeof(signed long);
  }
  const int datalen = sizeof(Dlog)/sizeof(float);
  float counter = 0;
  const float bufsize = self->buf.size();
    
  sleep(1);
  for(int i=0;i<self->buf.size();i++,v++){
    self->progress = 100.0*counter/bufsize;
    int j;
    if(use_float){
      float *p_vd;
      for(j=0, p_vd= ((float*)v);j<datalen;j++,p_vd++){
	fprintf(fp,"%f",*p_vd);
	if(j==datalen-1){
	  fprintf(fp,"\n");
	}else{
	  fprintf(fp,", ");
	}
      }
    }else{
      signed long *p_vd;
      for(j=0, p_vd= ((signed long*)v);j<datalen;j++,p_vd++){
	if(j==0){
	  /* j==0 and j==1 is 64bit gclk type */
	  signed long long int* p_gclk = (signed long long int*)p_vd;
	  fprintf(fp,"%lld",*p_gclk);
	  if(j==datalen-1){
	    fprintf(fp,"\n");
	  }else{
	    fprintf(fp,", ");
	  }
	}else if(j>1){
	  /* j==1 is gclk upper half. */
	  fprintf(fp,"%ld",*p_vd);
	  if(j==datalen-1){
	    fprintf(fp,"\n");
	  }else{
	    fprintf(fp,", ");
	  }
	}
      }
    }
    counter++;
  }
#endif
  fclose(fp);
  std::cout << "[DataLogger] Done." << std::endl;
  self->dumping_flag=false;

  return 0;
};


#if 1
class SimpleLogger {
public:
    /* TODO check alignment */
    struct V3 {
        float x;
        float y;
        float z;
        V3(){
            x = y = z = 0.0f;
        };
        V3(float _x, float _y, float _z) : x(_x),y(_y),z(_z){
        };
    }PACKING;
    //typedef hrp::Vector3 V3;
    struct DataLog {
        float   frame;
        float   loop;
        float   sectime;
        float   act_q[12];
        float   ref_q[12];
        float   ref_dq[12];
        V3       act_zmp;
        V3       rel_act_zmp;
        V3       ref_zmp;
        V3       rel_ref_zmp;
        V3       act_root_pos;
        V3       ref_base_pos;
        V3       act_cog;
        V3       ref_cog;
        V3       act_foot_origin_pos;
        V3       ref_foot_origin_pos_l;
        V3       ref_foot_origin_pos_r;
        V3       sf_pref;
        V3       sf_body_p;
        V3       sf_footl_p;
        V3       sf_footr_p;
        float   act_force_l[6];
        float   act_force_r[6];
        float   ref_force_l[6];
        float   ref_force_r[6];
        float   act_contact_state[2];
        float   contact_state[2];
        float   walking_state;
        float   controlSwingSupportTime[2];
        float   sbpCogOffset[3];
        V3      act_cogvel;
        V3      ref_zmp_modif;
        V3      ref_basePos_modif;
        V3      act_world_root_pos;
        V3      ref_traj_dp;
        V3      ref_traj_body_dp;
        V3      rpy;
        V3      filtered_rot;
        V3      lpf_rot;
        V3      rot_offset;
    }PACKING;
    //private:
    //boost::circular_buffer<DataLog> buf;
    bool logger_en;
    FILE *fp;
    char filename[256];
    typedef coil::Guard<coil::Mutex> Guard;
    coil::Mutex m_mutex;
public:
    //SimpleLogger() : buf(5000+200), logger_en(false) ,fp(NULL){
    SimpleLogger() : logger_en(false) ,fp(NULL){
        std::cout << MAKE_CHAR_COLOR_RED << "SimpleLogger()" << MAKE_CHAR_COLOR_DEFAULT << std::endl;
    };
    ~SimpleLogger(){
        std::cout << MAKE_CHAR_COLOR_RED << "~SimpleLogger()" << MAKE_CHAR_COLOR_DEFAULT << std::endl;
        stopLogging();
    };
    bool startLogging(bool time_append){
        Guard guard(m_mutex);
        if(!logger_en){
            logger_en = true;
            /* Open DataLog file */
            char* homedir = "/home/leus";
            sprintf(filename, "%s/%s/%s.dat",homedir,"log","datalog");

            if((fp=fopen(filename,"w"))==NULL){
                logger_en = false;
                std::cerr << "Error Cannot Open " << filename << std::endl;
            }else{
                fprintf(fp,"#PushRecover.cpp dataLog\n");
            }
        }
        return logger_en;
    };
    bool stopLogging(void){
        Guard guard(m_mutex);
        if(logger_en){
            //std::cout << MAKE_CHAR_COLOR_GREEN << "Closing Log file" << MAKE_CHAR_COLOR_DEFAULT << filename << std::endl;
            //fclose(fp);
            logger_en = false;
            fclose(fp);
            fp = NULL;
        }
        return true;
    };
    bool isRunning(void){
        Guard guard(m_mutex);
        return logger_en;
    };
    bool push(const DataLog &din){
        Guard guard(m_mutex);
        //buf.push_back(din);
        return logger_en;
    };
    /* DataLogがfloatだけで構成されていてpackされていることを仮定 */
    bool dump(const DataLog *din){
        Guard guard(m_mutex);
        float* pd = (float*)din;
        if(logger_en && fp!=NULL){
            for(int j=0; j<(sizeof(DataLog)/sizeof(float)); j++,pd++){
                fprintf(fp,"%+05.5f",*pd);
                if(j != (sizeof(DataLog)/sizeof(float))-1){
                    fprintf(fp,", ");
                }else{
                    fprintf(fp,"\n");
                }
            }
        }
    };
};
#else
class SimpleLogger {
public:
  /* TODO check alignment */
  struct V3 {
    float x;
    float y;
    float z;
    V3(){
      x = y = z = 0.0f;
    };
    V3(float _x, float _y, float _z) : x(_x),y(_y),z(_z){
    };
  }PACKING;
  //typedef hrp::Vector3 V3;
  struct DataLog {
    float   frame;
    float   loop;
    float   sectime;
    float   act_q[12];
    float   ref_q[12];
    float   ref_dq[12];
    V3       act_zmp;
    V3       rel_act_zmp;
    V3       ref_zmp;
    V3       rel_ref_zmp;
    V3       act_root_pos;
    V3       ref_base_pos;
    V3       act_cog;
    V3       ref_cog;
    V3       act_foot_origin_pos;
    V3       ref_foot_origin_pos_l;
    V3       ref_foot_origin_pos_r;
    V3       sf_pref;
    V3       sf_body_p;
    V3       sf_footl_p;
    V3       sf_footr_p;
    float   act_force_l[6];
    float   act_force_r[6];
    float   ref_force_l[6];
    float   ref_force_r[6];
    float   act_contact_state[2];
    float   contact_state[2];
    float   walking_state;
    float   controlSwingSupportTime[2];
    float   sbpCogOffset[3];
    V3      act_cogvel;
    V3      ref_zmp_modif;
    V3      ref_basePos_modif;
    V3      act_world_root_pos;
    V3      ref_traj_dp;
    V3      ref_traj_body_dp;
  }PACKING;
private:
  boost::circular_buffer<DataLog> buf;
  FILE *fp;
  bool logger_en;
  char filename[256];
public:
 SimpleLogger() : buf(5000+200), logger_en(false) {
      std::cout << MAKE_CHAR_COLOR_RED << "SimpleLogger()" << MAKE_CHAR_COLOR_DEFAULT << std::endl;
      /* Open DataLog file */
#if 0
      const bool time_append = true;
      char* homedir = "/home/leus";
      char timevar[] = "";
      time_t now = time(NULL);
      struct tm *pnow = localtime(&now);
      sprintf(timevar,"%04d%02d%02d%02d%02d%02d",pnow->tm_year+1900, pnow->tm_mon+1, pnow->tm_mday, pnow->tm_hour, pnow->tm_min, pnow->tm_sec);

      if(time_append){
          sprintf(filename, "%s/%s/%s_%s.dat",homedir,"log","datalog",timevar);
      }else{
          sprintf(filename, "%s/%s/%s.dat",homedir,"log","datalog");
      }
      std::cout << "Opening " << filename << std::endl;
#else
      char* homedir = "/home/leus";
      sprintf(filename, "%s/%s/%s.dat",homedir,"log","datalog");
#endif

      if((fp=fopen(filename,"w"))==NULL){
          std::cerr << "Error Cannot Open " << filename << std::endl;
      }else{
          fprintf(fp,"#PushRecover.cpp dataLog\n");
      }
  };
  ~SimpleLogger(){
      stopLogging();
  };
  bool startLogging(bool time_append){
      if(!logger_en){
          logger_en = true;
      }
      return logger_en;
  };
  bool stopLogging(void){
      if(logger_en){
          //std::cout << MAKE_CHAR_COLOR_GREEN << "Closing Log file" << MAKE_CHAR_COLOR_DEFAULT << filename << std::endl;
          //fclose(fp);
          logger_en = false;
      }
      return true;
  };
  bool isRunning(void){
      return logger_en;
  };
  bool push(const DataLog &din){
    buf.push_back(din);
    return logger_en;
  };
  /* DataLogがfloatだけで構成されていてpackされていることを仮定 */
  bool dump(const DataLog *din){
    float* pd = (float*)din;
    if(logger_en){
        for(int j=0; j<(sizeof(DataLog)/sizeof(float)); j++,pd++){
            fprintf(fp,"%+05.5f",*pd);
            if(j != (sizeof(DataLog)/sizeof(float))-1){
                fprintf(fp,", ");
            }else{
                fprintf(fp,"\n");
            }
        }
    }
  };
private:
  void dumpDataLog(const DataLog &din){
    /* TODO */
  };
};
#endif

//#define CONV_HRPVEC3(v) (SimpleLogger::V3((float)v(0),(float)v(1),(float)v(2)))
//#define CONV_VEC3(v) (SimpleLogger::V3((float)v[0],(float)v[1],(float)v[2]))
#define CONV_HRPVEC3(v) (dlog::V3((float)v(0),(float)v(1),(float)v(2)))
#define CONV_VEC3(v) (dlog::V3((float)v[0],(float)v[1],(float)v[2]))

#undef PACKING

#endif /*__SIMPLE_LOGGER_H__*/
