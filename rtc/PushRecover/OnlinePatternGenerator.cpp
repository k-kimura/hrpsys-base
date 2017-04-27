// -*- tab-width : 4 ; mode : C++ ; indent-tabs-mode : nil -*-
#include <link_physprof.h>  /* link paramter m,c,I */
#include "OnlinePatternGenerator.h"
#include <sched.h>
#include <errno.h>
#include <float.h>

namespace{
    const int iterate_num = 2;
    const int feedback_gain = 0.5;
    const float dT = 0.001f;
};

void* OnlinePatternGenerator::func(void *arg){
    OnlinePatternGenerator* self = (OnlinePatternGenerator*)arg;
    while(true){
        self->debug_counter++;
        // wait a signal
        if( self->m_waitcond.wait() ){
            break;
            //return 0;
        }
        const Vec3 Dx_offset = (1.0f/dT) * self->m_pstate->dx_offset;
        self->m_generator.calcBodyTrajectoryIterate(
                                                    self->m_modify_frame,     /*int init_frame*/
                                                    iterate_num,              /*int iterate_num*/
                                                    feedback_gain,            /*double feedback_gain*/
                                                    self->m_pstate->x_offset, /*Vec3 x_offset*/
                                                    Dx_offset                 /*Vec3 Dx_offset*/
                                                    );
    } /* End of while(true)*/

    _mm_mfence();

    return 0;
} /* End of OnlinePatternGenerator::func */

#if defined(__INTEL_COMPILER)||defined(__ICC)
OnlinePatternGenerator::State::State()
    : rot_offsets_len( 300 ),
      com_p( 0.0f ),
      x_offset( 0.0f ),
      dx_offset( 0.0f ),
      rot( 0.0f ),
      filtered_rot( 0.0f ),
      lpf_rot( 0.0f ),
      rot_offset( 0.0f ),
      rot_offsets( new Vec3[rot_offsets_len] ),
      rot_offsets_index( 0 ),
      filtered_q0( 0.0f ),
      filtered_q1( 0.0f ),
      lpf_q0( 0.0f ),
      lpf_q1( 0.0f ),
      filter_coef( 0.96f ),
      lpf_coef( 0.995f ),
      first_magnify( 1.0f ),
      max_gain( 0.0f )
{
    for( int i = 0; i < rot_offsets_len; i++ )
        rot_offsets[i] = QzVec3Zero();
}

OnlinePatternGenerator::OnlinePatternGenerator()
    : gen(0),
      m_pstate( new State() ),
      m_frame( 0 ), m_modify_frame( -1 ),
      m_generator( 0.001f, 0.50, 0.095f ),
      m_x_step( 0.0f ), m_y_step( 0.0f ),
      m_modify_thre( FLT_MAX ),
      m_command( new JoyCommand )
{
    // スレッド起動
    if ( pthread_create( &m_thread, NULL, func, this ) ){
        // スレッド生成に失敗
        throw std::runtime_error( "::OnlinePatternGenerator failed to pthread_create" );
    }
};

OnlinePatternGenerator::~OnlinePatternGenerator(){
    m_waitcond.signal( true );
    if ( pthread_join ( m_thread, NULL ) ) {
        // スレッドのjoinに失敗
        throw std::runtime_error( "::~OnlinePatternGenerator failed to pthread_join" );
    }
};
#elif defined(__GNUC__)
OnlinePatternGenerator::State::State()
    : rot_offsets_len( 300 ),
      com_p( Vec3Zero() ),
      x_offset( Vec3Zero() ),
      dx_offset( Vec3Zero() ),
      rot( Vec3Zero() ),
      filtered_rot( Vec3Zero() ),
      lpf_rot( Vec3Zero() ),
      rot_offset( Vec3Zero() ),
      rot_offsets( new Vec3[rot_offsets_len] ),
      rot_offsets_index( 0 ),
      filtered_q0( Vec3Zero() ),
      filtered_q1( Vec3Zero() ),
      lpf_q0( Vec3Zero() ),
      lpf_q1( Vec3Zero() ),
      filter_coef( 0.96f ),
      lpf_coef( 0.995f ),
      first_magnify( 1.0f ),
      max_gain( 0.0f )
{
    for( int i = 0; i < rot_offsets_len; i++ )
        rot_offsets[i] = Vec3Zero();
}

OnlinePatternGenerator::OnlinePatternGenerator()
    : gen(0),
      m_pstate( new State() ),
      m_frame( 0 ), m_modify_frame( -1 ),
      m_generator( 0.001f, 0.50, 0.095f ),
      m_x_step( 0.0f ), m_y_step( 0.0f ),
      m_modify_thre( FLT_MAX ),
      m_command( new JoyCommand )
{
    // スレッド起動
    if ( pthread_create( &m_thread, NULL, func, this ) ){
        // スレッド生成に失敗
        throw std::runtime_error( "::OnlinePatternGenerator failed to pthread_create" );
    }
};

OnlinePatternGenerator::~OnlinePatternGenerator(){
    m_waitcond.signal( true );
    if ( pthread_join ( m_thread, NULL ) ) {
        // スレッドのjoinに失敗
        throw std::runtime_error( "::~OnlinePatternGenerator failed to pthread_join" );
    }
};
#endif
void OnlinePatternGenerator::signal_filter( Vec3 &state, const Vec3 input )
{
    state = Vec3(m_pstate->filter_coef)*state + Vec3(1.0f-m_pstate->filter_coef)*input;
}

void OnlinePatternGenerator::lpf_filter( Vec3 &state, const Vec3 input )
{
    state = Vec3(m_pstate->lpf_coef)*state + Vec3(1.0f-m_pstate->lpf_coef)*input;
}

void OnlinePatternGenerator::incrementFrame( PatternState &state, ControlState &cstate ){
    //const long long st = rdtsc(); // record start cpu time
    int hoge = -1;
    const int step_time = 400;

    const float xstep_max = 0.5;
    const float ystep_max = 0.35;

    const bool preCommanded = isCommanded();
    const int sample_time = 50;

    /* max_gain の更新 */
    if( m_pstate->max_gain < cstate.pxstate->gain[1] ){
        m_pstate->max_gain = cstate.pxstate->gain[1];
    }

    /* JoyStick入力の解釈 */
    if( m_frame % sample_time == 0 ){
        //const int dash = m_jsr.getButton(JS_BUTTON_DASH);
        const int dash = 0;
        const float x_step_abs = dash ? 0.45f : 0.20;
        const float y_step_abs = dash ? 0.30f : 0.05f;
        // スティック
        const float axis_offset = 0.0;

        m_x_step = x_step_abs*( m_command->x - axis_offset)/(1.0f-axis_offset);
        m_y_step = y_step_abs*( m_command->y - axis_offset)/(1.0f-axis_offset);
    }

    /* ================================================================ */
    /* 体幹位置姿勢入力に対してフィルタをかける */
    /* 信号LPF signal_filter(output, input) */
    /* body_q は関節角度から計算される実値 */
    /* ================================================================ */
    signal_filter( m_pstate->filtered_rot, m_pstate->rot ); /* 体幹姿勢の更新 */
    signal_filter( m_pstate->filtered_q0, Vec3(/* 体幹位置の更新 */
                                               cstate.pxstate->body_q[0],
                                               cstate.pystate->body_q[0],
                                               0.0f ) );
    signal_filter( m_pstate->filtered_q1, Vec3(/* 体幹速度の更新 */
                                               cstate.pxstate->body_q[1],
                                               cstate.pystate->body_q[1],
                                               0.0f ) );


    /* ================================================================ */
    /* 体幹位置姿勢入力に対してフィルタをかける */
    /* オフセットLPF */
    /* body_q は関節角度から計算される体幹速度を利用して積分演算で出力される体幹絶対位置 */
    /* ================================================================ */
    lpf_filter( m_pstate->lpf_rot, m_pstate->rot ); /* 体幹姿勢の更新 */
    lpf_filter( m_pstate->lpf_q0, Vec3(/* 体幹位置の更新 */
                                       cstate.pxstate->body_q[0],
                                       cstate.pystate->body_q[0],
                                       0.0f ) );
    lpf_filter( m_pstate->lpf_q1, Vec3(/* 体幹速度の更新 */
                                       cstate.pxstate->body_q[1],
                                       cstate.pystate->body_q[1],
                                       0.0f ) );
    const Vec3 dif( cstate.pxstate->body_q[0],
                    cstate.pystate->body_q[0],
                    0.0 );



    /* ================================================================ */
    /* ジョイスティック入力に応じてフットステップをプッシュし */
    /* 重心・足先軌道を生成する */
    /* ================================================================ */
    if( isComplete() ){
        hoge=1;
        /* ================================================================ */
        /* 軌道生成が終了し、アイドル状態の場合 */
        /* ジョイスティック入力が来たら初期歩容を生成する */
        /* ================================================================ */
        m_frame = 0;
        m_generator.m_steps.clear();
        if( isCommanded() ){
            hoge=2;
            // 歩行軌道
            m_generator.addStep( 200,  0, Vec3( 0.0f,  0.0f, 0.0f ) );
            m_generator.addStep( step_time,  1, Vec3( 0.0f,  0.095f, 0.0f ) );
            m_generator.addStep( step_time, -1, Vec3( m_x_step, -0.190f+(m_y_step<0?m_y_step:0.0f), 0.0f ) );
            for( int i = 0; i < 1; i++ ){
                m_generator.addStep( step_time,  1, Vec3( m_x_step,  0.190f+(m_y_step>0?m_y_step:0.0f), 0.0f ) );
                m_generator.addStep( step_time, -1, Vec3( m_x_step, -0.190f+(m_y_step<0?m_y_step:0.0f), 0.0f ) );
            }
            m_generator.addStep( 2000,  0, Vec3( 0.0f,  0.095f, 0.0f ) );

            m_generator.modifyStepOffset();
            m_generator.calcSwingTrajectory();
            m_generator.calcBodyTrajectoryIterate( 0, iterate_num, feedback_gain );
        }
        else{
            hoge=3;
            m_generator.addStep( 200,  0, Vec3( 0.0f,  0.0f, 0.0f ) );
        }/* end of if( isCommanded() ) */
    }else{
        hoge=4;
        /* ================================================================ */
        /* 生成軌道が残っている場合 */
        /* ================================================================ */
        /* 現在計画されているフットステップの残りフレーム */
        const int traj_len = m_generator.calcTrajectoryLength();

        /* 初期生成フットステップ(2200+4*step_time)よりも長い場合には、最大６歩分計算する。*/
        /* そうでない場合には、最大２歩分計算する。*/
        const int start_frame = traj_len > (2200+4*step_time) ? traj_len-(2000+6*step_time) : traj_len-(2000+2*step_time);
        /* 残り2000+２歩分フレーム以上は余っており、かつstart_frameを越えている場合 */
        if(( m_frame <= traj_len-2700 ) && (m_frame >= start_frame ) && (m_frame % sample_time == 0))
            {
                hoge=5;
                const int calc_margin = 10; // 最大演算時間 10frame先まで見る
                const int future_index = m_frame + calc_margin;
                /* trajectory generatorはm_modify_frameをinit_frameとして計算する */
                m_modify_frame = future_index;

                /* ジョイスティック入力の停止の検出 */
                const bool stop_command = preCommanded && (!isCommanded());
                if( isCommanded() || stop_command ){
                    hoge=6;
                    /* dif は dif=Vec3( cstate.pxstate->body_q[0],
                       cstate.pystate->body_q[0],
                       0.0 );
                       と定義されている.
                       コンストラクタ時点ではm_modify_thre=FLT_MAXなので適切な値にどこかで設定する必要がある。
                    */
                    /* m_modify_threでxy平面上の移動範囲を制限していると考える */
                    const int x_modify_enabled = (dif[0] > m_modify_thre) || (dif[0] < -m_modify_thre);
                    const int y_modify_enabled = (dif[1] > m_modify_thre) || (dif[1] < -m_modify_thre);
                    /* 体幹姿勢変化による重心位置の偏差 */
                    const Vec3 rot_offset(
                                          x_modify_enabled ? (m_pstate->filtered_rot[1]-m_pstate->lpf_rot[1])*0.5f : 0.0f,
                                          0.0f,
                                          0.0f );
                    // 補正軌道生成
                    m_pstate->rot_offset = rot_offset;

                    const Vec3 x_offset = Vec3(
                                               x_modify_enabled ? cstate.pxstate->body_q[0]-m_pstate->lpf_q0[0] : 0.0f,
                                               y_modify_enabled ? cstate.pystate->body_q[0]-m_pstate->lpf_q0[1] : 0.0f,
                                               0.0f ) + rot_offset;

                    const Vec3 dx_offset(
                                         x_modify_enabled ? m_pstate->filtered_q1[0]-m_pstate->lpf_q1[0] : 0.0f,
                                         0.0f,
                                         0.0f );
                    m_pstate->x_offset = x_offset;
                    m_pstate->dx_offset = dx_offset;
                    const Vec3 Dx_offset = (1.0f/dT) * dx_offset;

                    const int margin = 100;

                    /* margin frame より先のフットステップは破棄する */
                    m_generator.deleteFuture( future_index+margin );
                    /* フットステップの挿入 */
                    {
                        const bool is_odd = (*boost::prior(m_generator.m_steps.end()))->support_step < 0;
                        /* 現在の支持脚に応じて一歩挿入 */
                        if( is_odd ){
                            m_generator.addStep( step_time,  1, Vec3( m_x_step,  0.190f+(m_y_step>0?m_y_step:0.0f), 0.0f ) );
                        }else{
                            m_generator.addStep( step_time, -1, Vec3( m_x_step, -0.190f+(m_y_step<0?m_y_step:0.0f), 0.0f ) );
                        }
                        /* 停止用歩容挿入 */
                        for( int i = 0; i < 1; i++ ){
                            m_generator.addStep( step_time,  1, Vec3( m_x_step,  0.190f+(m_y_step>0?m_y_step:0.0f), 0.0f ) );
                            m_generator.addStep( step_time, -1, Vec3( m_x_step, -0.190f+(m_y_step<0?m_y_step:0.0f), 0.0f ) );
                        }
                        /* 停止用歩容挿入 */
                        if( is_odd ){
                            m_generator.addStep( 2000,  0, Vec3( 0.0f,  0.095f, 0.0f ) );
                        }
                        else{
                            m_generator.addStep( step_time,  1, Vec3( m_x_step,  0.190f+(m_y_step>0?m_y_step:0.0f), 0.0f ) );
                            m_generator.addStep( 2000,  0, Vec3( 0.0f,  -0.095f, 0.0f ) );
                        }
                    }

                    const int dframe = m_generator.getDFrame( future_index );
                    const float dratio = float(step_time-dframe)/float(step_time);
                    const float dxstep_max = xstep_max * dratio;
                    const float dystep_max = ystep_max * dratio;
                    m_generator.modifyStep(
                                           future_index, margin,
                                           xstep_max, ystep_max,
                                           x_offset, Dx_offset,
                                           m_pstate->first_magnify );
                    m_generator.calcSwingTrajectory( future_index );
                }/* if( isCommanded() || stop_command ) */
                else {
                    m_pstate->rot_offset = Vec3(0.0f);
                    m_pstate->x_offset = Vec3(0.0f);
                    m_pstate->dx_offset = Vec3(0.0f);
                    cstate.pxstate->gain[1] = m_pstate->max_gain;
                    cstate.pystate->gain[1] = m_pstate->max_gain;
                }

                m_waitcond.signal( false );
            }/* end of if(( m_frame <= traj_len-2700 ) && (m_frame >= start_frame ) && (m_frame % sample_time == 0)) */
    }/* end of if( isComplete() ) */

    if( m_frame == m_modify_frame ){
        //TODO ここはなくていい？
    }


    const int traj_len = m_generator.calcTrajectoryLength();
    Vec3 rot_offset = m_pstate->rot_offsets[m_pstate->rot_offsets_index];
    if( traj_len > 1 ){
        // 状態を取得
        m_generator.getModifiedFootLState( m_frame, state.foot_l_p );
        m_generator.getModifiedFootRState( m_frame, state.foot_r_p );

        state.p = m_generator.getPref( m_frame );
        state.body_p = m_generator.getBody( m_frame )-rot_offset;

        // 進める 最大len-2まで
        if( m_pstate->rot_offsets_index < m_pstate->rot_offsets_len-2 ) m_pstate->rot_offsets_index++;
    }

#if 0
    {
        if(m_frame%sample_time==0){
            printf("========================================\n");
            printf("traj_len=%d\n",traj_len);
            printf("m_frame=%d\n",m_frame);
            printf("m_modify_frame=%d\n",m_modify_frame);
            printf("m_x_step=%lf, %lf\n",m_x_step,m_y_step);
            printf("hoge,debug_counter=%d, %d\n",hoge, debug_counter);
            printf("state.body_p=[%lf,%lf,%lf]\n",state.body_p[0],state.body_p[1],state.body_p[2]);
            const int start_frame = traj_len > (2200+4*step_time) ? traj_len-(2000+6*step_time) : traj_len-(2000+2*step_time);
            printf("start_frame=%d\n",start_frame);
            printf("if=(%d & %d & %d)\n",( m_frame <= traj_len-2700 ) , (m_frame >= start_frame ) , (m_frame % sample_time == 0));
            printf("========================================\n");
        }
    }
#endif

    // フレームカウンタを操作
    if( !isComplete() ){
        m_frame++;
    }

    //const long long ed = rdtsc();
    //const float cycle = ed-st;

} /* end of OnlinePatternGenerator::incrementFrame */

bool OnlinePatternGenerator::isComplete() const
{
    const int traj_len = m_generator.calcTrajectoryLength();
    return traj_len <= m_frame+2;
}

void OnlinePatternGenerator::update(  const UpdateState& state )
{
    m_pstate->com_p = state.com_p;
    m_pstate->rot = state.rot;
}


void OnlinePatternGenerator::command( const JoyCommand& command ){
    m_command->x = command.x;
    m_command->y = command.y;
}
