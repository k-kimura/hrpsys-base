#ifndef __step_forward_h__
#define __step_forward_h__
#include <reactive_walk_generator.h>
#include <boost/scoped_ptr.hpp>
//#include <realtime_task.h>
#include <pthread.h>
#include "BodyIKMethod.h"

template<class T>
class PThreadWaitCond{
    pthread_mutex_t m_mutex;
    pthread_cond_t m_cond;
    T m_val;
public:
    PThreadWaitCond(){
        pthread_mutex_init( &m_mutex, NULL );
        pthread_cond_init( &m_cond, NULL );
    }
    ~PThreadWaitCond(){
        pthread_mutex_destroy( &m_mutex );
        pthread_cond_destroy( &m_cond );
    }
    void signal( T _val ){
        pthread_mutex_lock( &m_mutex );
        m_val = _val;
        pthread_cond_signal( &m_cond );
        pthread_mutex_unlock( &m_mutex );
    }
    T wait(){
        pthread_mutex_lock( &m_mutex );
        pthread_cond_wait( &m_cond, &m_mutex );
        T _val = m_val;
        pthread_mutex_unlock( &m_mutex );
        return _val;
    }
};

class StepForward {
    pthread_t m_thread; /* Reactive Walk Generatorは専用スレッドにて実効する */
    ITrajectoryGenerator* gen;
    static void* func(void* arg);
    PThreadWaitCond<bool> m_waitcond;
public:
    volatile int is_ready;
    Vec3* m_x0;
    StepForward() : gen( 0 ), is_ready( 0 ){
        //初期化
        m_x0 = (Vec3*)_mm_malloc( 3*sizeof(Vec3), sizeof(Vec3) );
        std::fill( m_x0, m_x0+3, Vec3( QzVec3Zero() ) );
        // スレッド起動
        if ( pthread_create( &m_thread, NULL, func, this ) )
            {
                // スレッド生成に失敗
                throw std::runtime_error( "::StepForwardTask failed to pthread_create" );
            }
    }
    ~StepForward() {
        // 終了フラグ
        m_waitcond.signal( true );
        // スレッド終了待ち
        if ( pthread_join ( m_thread, NULL ) ) {
            // スレッドのjoinに失敗
            throw std::runtime_error( "::~StepForwardTask failed to pthread_join" );
        }
        _mm_free( m_x0 );
    }
    /**
     * @param x0 コピーされる
     * ex.
     * const Vec3 x0[] = {
     *  Vec3( 0.0f, -0.095f, 0.0f ),
     *  Vec3( 0.0f,    0.0f, 0.0f ),
     *  Vec3( 0.4f,    0.0f, 0.0f )};
     */
    void start( const Vec3 x0[] ){
        std::copy( x0, x0+3, m_x0 );
        is_ready = false;
        if(gen!=NULL){
            //delete gen;
            gen = NULL;
        }
        m_waitcond.signal( false );
    }

    ITrajectoryGenerator* getReady(){
        if( is_ready > 0 ){
            _mm_mfence();
            return gen;
        }else{
            return 0;
        }
    }
};

#if 0
struct StepForwardInvoker {
    StepForward stpf;
    bool is_readyToPush() const {
        return frame_offset > 4000;
    }

};
#endif

#endif /* __step_forward_h__ */
