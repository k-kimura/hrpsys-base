#ifndef __SIMPLE_LOGGER_H__
#define __SIMPLE_LOGGER_H__
#include <boost/circular_buffer.hpp>

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

#define PACKING __attribute__((__packed__))

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
  }PACKING;
private:
  boost::circular_buffer<DataLog> buf;
  FILE *fp;
  bool logger_en;
public:
  SimpleLogger() : buf(5000+200), logger_en(true) {
    /* Open DataLog file */
    //char* homedir = getenv("HOME");
    char* homedir = "/home/leus";
    char filename[256];
    sprintf(filename, "%s/%s/%s",homedir,"log","datalog.dat");
    std::cout << "Opening " << filename << std::endl;

    if((fp=fopen(filename,"w"))==NULL){
      std::cerr << "Error Cannot Open " << filename << std::endl;
      logger_en = false;
    }else{
      fprintf(fp,"#PushRecover.cpp dataLog\n");
    }
  };
  ~SimpleLogger(){
    DataLog *d = buf.linearize();

    if(logger_en){
      for(int i=0; i<buf.size(); i++,d++){
        float* pd = (float*)d;
        for(int j=0; j<(sizeof(DataLog)/sizeof(float)); j++,pd++){
          fprintf(fp,"%+05.5f",*pd);
          if(j != (sizeof(DataLog)/sizeof(float))-1){
            fprintf(fp,", ");
          }else{
            fprintf(fp,"\n");
          }
        }
        if(i%100==0){
          printf(".");
        }
      }
      std::cout << MAKE_CHAR_COLOR_GREEN << "Save DataLog Succeeded" << MAKE_CHAR_COLOR_DEFAULT << std::endl;
    }else{
      std::cout << MAKE_CHAR_COLOR_RED << "Cannot Save DataLog" << MAKE_CHAR_COLOR_DEFAULT << std::endl;
    }

    fclose(fp);
  };
  bool push(const DataLog &din){
    buf.push_back(din);
    return logger_en;
  };
  /* DataLogがfloatだけで構成されていてpackされていることを仮定 */
  bool dump(const DataLog *din){
    float* pd = (float*)din;
    for(int j=0; j<(sizeof(DataLog)/sizeof(float)); j++,pd++){
      fprintf(fp,"%+05.5f",*pd);
      if(j != (sizeof(DataLog)/sizeof(float))-1){
        fprintf(fp,", ");
      }else{
        fprintf(fp,"\n");
      }
    }
  };
private:
  void dumpDataLog(const DataLog &din){
    /* TODO */
  };
};

#define CONV_HRPVEC3(v) (SimpleLogger::V3((float)v(0),(float)v(1),(float)v(2)))
#define CONV_VEC3(v) (SimpleLogger::V3((float)v[0],(float)v[1],(float)v[2]))

#undef PACKING

#endif /*__SIMPLE_LOGGER_H__*/
