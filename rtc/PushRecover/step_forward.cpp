#include <link_physprof.h>  /* link paramter m,c,I */
#include "step_forward.h"

void* StepForward::func(void* arg){
    StepForward* self = (StepForward*)arg;
    // リアルタイムスレッド
    // 優先度はちょっと落とす
    //RealtimeContext rt_context( REALTIME_PRIO_MAX-4, 1000 );

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
#define DOWHILE_EN
#ifdef DOWHILE_EN
    while(true){
#endif
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
            std::cerr << "[StepForward] Foot Colide" << std::endl;
#endif
#if 1
        }
#else
        }else{
            self->is_ready = 1;
        }
#endif
        //std::cout << "[StepForward] L_first=[" << rwg_lfirst.eval_value << "], R_first=[" << rwg_rfirst.eval_value << "]" << std::endl;

        for( int i = 0; i < 5; i++ ){
            /* template<bool dump> void iterateOnce( const int offset, const int calc_len, const double feedback_gain ) */
            if( rwg_lfirst.eval_value < rwg_rfirst.eval_value ){
                rwg_lfirst.iterateOnce<false>( 0, 1500+i*100, 1.0 );
            }else{
                rwg_rfirst.iterateOnce<false>( 0, 1500+i*100, 1.0 );
            }
            self->is_ready = i + 1;
        }
#ifdef DOWHILE_EN
    } /* End of while(true) */
#endif
    self->is_ready = 0;
    _mm_mfence();

    return 0;
} /* End of StepForward::func() */
