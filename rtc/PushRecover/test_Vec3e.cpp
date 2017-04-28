// -*- tab-width : 4 ; mode : C++ ; indent-tabs-mode : nil -*-
#include "Vec3e.h"
#include "PushRecover.h"

int test1(void);
int test2(void);
int test3(void);
int test4(void);

int main(void){

    test1();
    test2();
    test3();
    test4();

    return 0;
}

#define print_traj(v) \
    std::cout << #v << ".p= [" << (v.p).transpose() << "]" << std::endl;\
    std::cout << #v << ".body_p= [" << (v.body_p).transpose() << "]" << std::endl;\
    std::cout << #v << ".footl_p= [" << (v.footl_p).transpose() << "]" << std::endl;\
    std::cout << #v << ".footr_p= [" << (v.footr_p).transpose() << "]" << std::endl;\
    std::cout << #v << ".dp= [" << (v.dp).transpose() << "]" << std::endl;\
    std::cout << #v << ".body_dp= [" << (v.body_dp).transpose() << "]" << std::endl

int test1(void){
    std::cout << __func__ << std::endl;
    PushRecover::TrajectoryElement<hrp::Vector3e>  prev_ref_traj;
    PushRecover::TrajectoryElement<hrp::Vector3e>  ref_traj;

    print_traj(prev_ref_traj);
    print_traj(ref_traj);

    std::cout << "clear()" << std::endl;
    prev_ref_traj.clear();
    ref_traj.clear();

    print_traj(prev_ref_traj);
    print_traj(ref_traj);

};

int test2(void){
    std::cout << __func__ << std::endl;
    PushRecover::TrajectoryElement<hrp::Vector3e>  prev_ref_traj;
    PushRecover::TrajectoryElement<hrp::Vector3e>  ref_traj;

    prev_ref_traj.clear();
    ref_traj.clear();

    prev_ref_traj.p      = hrp::Vector3e(1,2,3);
    prev_ref_traj.body_p = hrp::Vector3e(4,5,6);
    prev_ref_traj.footl_p = hrp::Vector3e(7,8,9);
    prev_ref_traj.footr_p = hrp::Vector3e(10,11,12);
    prev_ref_traj.dp = hrp::Vector3e(13,14,15);
    prev_ref_traj.body_dp = hrp::Vector3e(16,17,18);

    print_traj(prev_ref_traj);
    print_traj(ref_traj);

};

int test3(void){
    std::cout << __func__ << std::endl;
    PushRecover::TrajectoryElement<hrp::Vector3e>  prev_ref_traj;
    PushRecover::TrajectoryElement<hrp::Vector3e>  ref_traj;

    prev_ref_traj.clear();
    ref_traj.clear();

    prev_ref_traj.p      = hrp::Vector3e(1,2,3);
    prev_ref_traj.body_p = hrp::Vector3e(4,5,6);
    prev_ref_traj.footl_p = hrp::Vector3e(7,8,9);
    prev_ref_traj.footr_p = hrp::Vector3e(10,11,12);
    prev_ref_traj.dp = hrp::Vector3e(13,14,15);
    prev_ref_traj.body_dp = hrp::Vector3e(16,17,18);

    ref_traj = prev_ref_traj;

    print_traj(prev_ref_traj);
    print_traj(ref_traj);

};

int test4(void){
    std::cout << __func__ << std::endl;
    PushRecover::TrajectoryElement<hrp::Vector3e>  prev_ref_traj;
    PushRecover::TrajectoryElement<Vec3e>  ref_traj;

    std::cout << "Clear ref_traj and prev_ref_traj" << std::endl;
    prev_ref_traj.clear();
    ref_traj.clear();

    print_traj(prev_ref_traj);
    print_traj(ref_traj);

    prev_ref_traj.p       = hrp::Vector3e(1,2,3);
    prev_ref_traj.body_p  = hrp::Vector3e(4,5,6);
    prev_ref_traj.footl_p = hrp::Vector3e(7,8,9);
    prev_ref_traj.footr_p = hrp::Vector3e(10,11,12);
    prev_ref_traj.dp      = hrp::Vector3e(13,14,15);
    prev_ref_traj.body_dp = hrp::Vector3e(16,17,18);

    Vec3e v1(51.0f,52.0f,53.0f);
    std::cout << "v1=" << v1.transpose() << std::endl;
    std::cout << "v1=pref_ref_traj.p" << std::endl;
    v1 = prev_ref_traj.p;
    std::cout << "v1=" << v1.transpose() << std::endl;

    hrp::Vector3e v2(0.0f,0.1f,0.2f);
    std::cout << "v2=" << v2.transpose() << std::endl;
    std::cout << "v2=v1" << std::endl;
    v2=v1;
    std::cout << "v2=" << v2.transpose() << std::endl;

    ref_traj = prev_ref_traj;

    //ref_traj.dp = ref_traj.p - ((hrp::Vector3e)prev_ref_traj.p);
    //ref_traj.body_dp = ref_traj.body_p - ((hrp::Vector3e)prev_ref_traj.body_p);
};
