#ifndef _QUICK_TURN_GENERATOR_H_
#define _QUICK_TURN_GENERATOR_H_
#include "reactive_walk_generator.h"
#include <boost/scoped_ptr.hpp>
#include <pthread.h>
#include "BodyIKMethod.h"
#include "SimpleLogger.h" /* for make_char_color macro */
#include "step_forward.h" /* for PThreadWaitCond */

struct ITrajectoryGeneratorRotational : ITrajectoryGenerator{
    /**
     * @brief indexの軌道を取得
     * @param[in] offset
     * @param[out] body_p
     * @param[out] body_R
     * @param[out] footl_p
     * @param[out] footl_p
     */
    virtual void getTrajectoryFrame( const int index, Vec3& pref, Vec3& body_p, Mat3 &body_R, Vec3 &footl_p, Vec3 &footr_p ) = 0;
};

class QuickTurn {
    pthread_t m_thread; /* Reactive Walk Generatorは専用スレッドにて実効する */
    ITrajectoryGeneratorRotational* gen;
    static void* func(void* arg);
    PThreadWaitCond<bool> m_waitcond;
public:
    volatile int is_ready;
    Vec3* m_x0;
    QuickTurn() : gen( 0 ), is_ready( 0 ){
        //初期化
        m_x0 = (Vec3*)_mm_malloc( 3*sizeof(Vec3), sizeof(Vec3) );
        std::fill( m_x0, m_x0+3, Vec3( QzVec3Zero() ) );
        // スレッド起動
        if ( pthread_create( &m_thread, NULL, func, this ) )
            {
                // スレッド生成に失敗
                throw std::runtime_error( "::QuickTurnGeneratorTask failed to pthread_create" );
            }
    };
    ~QuickTurn(){
        // 終了フラグ
        m_waitcond.signal( true );
        // スレッド終了待ち
        if ( pthread_join ( m_thread, NULL ) ) {
            // スレッドのjoinに失敗
            throw std::runtime_error( "::~QuickTurnGeneratorTask failed to pthread_join" );
        }
        _mm_free( m_x0 );
    };
    /**
     * @param x0 コピーされる
     * ex.
     * const Vec3 x0[] = {
     *  Vec3( 0.0f, -0.095f, 0.0f ),
     *  Vec3( 0.0f,    0.0f, 0.0f ),
     *  Vec3( 0.4f,    0.0f, 0.0f )};
     */
    void start( const Vec3 x0[] ){
        std::cout << "[pr] " << MAKE_CHAR_COLOR_RED << "Calling QuickTurnGenerator start()" << MAKE_CHAR_COLOR_DEFAULT << std::endl;
        std::copy( x0, x0+3, m_x0 );
        is_ready = false;
        if(gen!=NULL){
            //delete gen;
            gen = NULL;
        }
        m_waitcond.signal( false );
    }

    void reset(void){
        is_ready = false;
        if(gen!=NULL){
            gen = NULL;
        }
    }

    ITrajectoryGeneratorRotational* getReady(){
        if( is_ready > 0 ){
            _mm_mfence();
            return gen;
        }else{
            return 0;
        }
    }
};

template<int max_len, int i_max_angle, typename FT = float>
struct QuickTurnGenerator : ITrajectoryGeneratorRotational {
    FT dt;
    FT max_angle;
    struct TurnConfig {
        FT angle;   /* turn angle */
        FT time;    /* */
        Vec3 zmp_offset;
    };
    QuickTurnGenerator(FT _dt, TurnConfig *_p_turnconf) : dt(_dt), max_angle((FT)i_max_angle){
    };
    /**
     * @brief indexの軌道を取得
     * @param[in] offset
     * @param[out] body_p
     * @param[out] footl_p
     * @param[out] footl_p
     */
    virtual void getTrajectoryFrame( const int index, Vec3& pref, Vec3& body_p, Mat3& body_R, Vec3 &footl_p, Vec3 &footr_p )
    {
        const int i = index < max_len ? index : max_len-1;
    }
};

#endif  /* _QUICK_TURN_GENERATOR_H_ */
