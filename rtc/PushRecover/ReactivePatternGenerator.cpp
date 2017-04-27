// -*- tab-width : 4 ; mode : C++ ; indent-tabs-mode : nil -*-
#include <link_physprof.h>  /* link paramter m,c,I */
#include "ReactivePatternGenerator.h"
#include <sched.h>
#include <errno.h>

void* ReactivePatternGenerator::func(void* arg){
    ReactivePatternGenerator* self = (ReactivePatternGenerator*)arg;
    // リアルタイムスレッド
    // 優先度はちょっと落とす
#if 0  /* need to be run with permission */
    //RealtimeContext rt_context( REALTIME_PRIO_MAX-4, 1000 );
    {
        sched_param param;
        param.sched_priority = sched_get_priority_max(SCHED_FIFO)-4;
        if( sched_setscheduler( 0, SCHED_FIFO, &param ) == -1 ){
            std::cout << "[Scheduler] " << MAKE_CHAR_COLOR_RED << "Could not set scheduler setting" << MAKE_CHAR_COLOR_DEFAULT << "errno=[" << errno << "]=" << strerror(errno) << std::endl;
            throw std::runtime_error( "sched_setscheduler" );
        }
    }
#endif

    // きめうっちんぐ軌道
    const int supports_step_lfirst[] = {-1, 1,-1,0};
    const int supports_step_rfirst[] = { 1,-1, 1,0};
    const float x_range = 0.5f;
    const float ystep = 0.095f;
    const Vec3 dpref_base_lfirst[] = {
        Vec3(-x_range, ystep*2,0.0f),
        Vec3(-x_range,-ystep*2,0.0f),
        Vec3( 0.0f, ystep,0.0f) };
    const Vec3 dpref_base_rfirst[] = {
        Vec3(-x_range,-ystep*2,0.0f),
        Vec3(-x_range, ystep*2,0.0f),
        Vec3( 0.0f, -ystep,0.0f) };
    const int dpref_step_n[] = { 49, 169, 289 };
    const Vec3 dpref_gain_lfirst[] = { Vec3(2.0f*x_range,-0.1f,0.0f) };
    const Vec3 dpref_gain_rfirst[] = { Vec3(2.0f*x_range, 0.1f,0.0f) };
    const int dpref_len = sizeof(dpref_base_lfirst)/sizeof(Vec3);
    const int traj_len = 300;
    const float swing_height = 0.04f;

    typedef DPrefModifier<2> Modifier;

    //template<int dpref_len,
    //         int opt_traj_len,  //最適化をかける長さ
    //         int max_len,       //生成される軌道の全長
    //         int pad,           //FootTrajectoryGeneratorに渡される
    //         template<int> class Trajectory,
    //         template<class> class _Evaluator,
    //         class Modifier >
    typedef ReactiveWalkGenerator<dpref_len, traj_len, 5000, 20,
                                  FastFootTrajectoryGenerator<SWING_TABLE_SIZE>::FootTrajectoryAccel,
                                  FootAccelEvaluator, Modifier > RWG;

    _CRT_ALIGN(64) LinkParam link_param( m, c, I );

    // パラメータ
    TrajectoryEvalParam params_lfirst(
                                      swing_height, ystep, supports_step_lfirst, dpref_base_lfirst, dpref_gain_lfirst, dpref_step_n );
    TrajectoryEvalParam params_rfirst(
                                      swing_height, ystep, supports_step_rfirst, dpref_base_rfirst, dpref_gain_rfirst, dpref_step_n );
    // 環境構築
    RWG::Environment environment_lfirst( link_param,
                                         traj_body_init,
                                         InitialLfoot_p,
                                         InitialRfoot_p );
    RWG::Environment environment_rfirst( link_param,
                                         traj_body_init,
                                         InitialLfoot_p,
                                         InitialRfoot_p );
    while(true){
        // シグナル待ち
        if( self->m_waitcond.wait() ){
            return 0;
        }

        /* 初期条件から右足踏み出しか左足踏み出しか決定する */
        Vec3 x0_lfirst[3];
        std::copy(self->m_x0, self->m_x0+3, x0_lfirst );
        x0_lfirst[0][1] -= ystep;
        RWG rwg_lfirst( &environment_lfirst, &params_lfirst, x0_lfirst, 8 ); /* constructor内で評価 */ /* 8はiteration num of DIRECT */

        Vec3 x0_rfirst[3];
        std::copy(self->m_x0, self->m_x0+3, x0_rfirst );
        x0_rfirst[0][1] += ystep;
        RWG rwg_rfirst( &environment_rfirst, &params_rfirst, x0_rfirst, 8 ); /* constructor内で評価 */

        if( rwg_lfirst.eval_value < rwg_rfirst.eval_value ){
            self->gen = &rwg_lfirst;
#ifdef DEBUG_STEP_FORWARD
            std::cout << "Gen Left First. Eval=" << rwg_lfirst.eval_value << std::endl;
#endif
        }else{
            self->gen = &rwg_rfirst;
#ifdef DEBUG_STEP_FORWARD
            std::cout << "Gen Right First. Eval=" << rwg_rfirst.eval_value << std::endl;
#endif
        }

        /* 初期条件から右足踏み出しか左足踏み出しか決定する */
        _mm_mfence(); /* assure memory access */
        if(( rwg_lfirst.eval_value > 1.0 ) && (rwg_rfirst.eval_value > 1.0) ) {
#ifdef DEBUG_STEP_FORWARD
            std::cerr << "[ReactivePatternGenerator] Foot Colide" << std::endl;
#endif
        }else{
            self->is_ready = 1;
        }

        std::cout << "[ReactivePatternGenerator] L_first=[" << rwg_lfirst.eval_value << "], R_first=[" << rwg_rfirst.eval_value << "]" << std::endl;

        for( int i = 0; i < 5; i++ ){
            /* template<bool dump> void iterateOnce( const int offset, const int calc_len, const double feedback_gain ) */
#if defined(__INTEL_COMPILER)||defined(__ICC)
            if( rwg_lfirst.eval_value < rwg_rfirst.eval_value ){
                rwg_lfirst.iterateOnce<false>( 0, 1500+i*100, 1.0 );
            }else{
                rwg_rfirst.iterateOnce<false>( 0, 1500+i*100, 1.0 );
            }
#elif defined(__GNUC__)
            if( rwg_lfirst.eval_value < rwg_rfirst.eval_value ){
                rwg_lfirst.template iterateOnce<false,LegIKParam,LEG_IK_TYPE>( 0, 1500+i*100, 1.0 );
            }else{
                rwg_rfirst.template iterateOnce<false,LegIKParam,LEG_IK_TYPE>( 0, 1500+i*100, 1.0 );
            }
#endif
            self->is_ready = i + 1;
            std::cout << "[ReactivePatternGenerator] "<< MAKE_CHAR_COLOR_BLUE << "is_ready=" << self->is_ready << MAKE_CHAR_COLOR_DEFAULT << std::endl;
        }
    } /* End of while(true) */

    self->is_ready = 0;
    _mm_mfence();

    return 0;
} /* End of ReactivePatternGenerator::func() */

