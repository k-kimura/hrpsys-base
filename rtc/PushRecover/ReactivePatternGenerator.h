// -*- tab-width : 4 ; mode : C++ ; indent-tabs-mode : nil -*-
#ifndef __ReactivePatternGenerator_h__
#define __ReactivePatternGenerator_h__
#include <reactive_walk_generator.h>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
//#include "realtime_task.h"
#include <pthread.h>
#include "BodyIKMethod.h"
#include "SimpleLogger.h" /* for make_char_color macro */
#include "PThreadWaitcond.h"
#include "PatternGenerator.h"

class ReactivePatternGenerator : public IPatternGenerator {
    pthread_t m_thread; /* Reactive Walk Generatorは専用スレッドにて実効する */
    ITrajectoryGenerator* gen;
    static void* func(void* arg);
    PThreadWaitCond<bool> m_waitcond;
public:
    volatile int is_ready;
    Vec3* m_x0;
#if defined(__INTEL_COMPILER)||defined(__ICC)
    ReactivePatternGenerator() : gen( 0 ), is_ready( 0 ){
        //初期化
        m_x0 = (Vec3*)_mm_malloc( 3*sizeof(Vec3), sizeof(Vec3) );
        std::fill( m_x0, m_x0+3, Vec3( QzVec3Zero() ) );
        // スレッド起動
        if ( pthread_create( &m_thread, NULL, func, this ) )
            {
                // スレッド生成に失敗
                throw std::runtime_error( "::ReactivePatternGeneratorTask failed to pthread_create" );
            }
    }
    ~ReactivePatternGenerator() {
        // 終了フラグ
        m_waitcond.signal( true );
        // スレッド終了待ち
        if ( pthread_join ( m_thread, NULL ) ) {
            // スレッドのjoinに失敗
            throw std::runtime_error( "::~ReactivePatternGeneratorTask failed to pthread_join" );
        }
        _mm_free( m_x0 );
    }
#elif defined(__GNUC__)
    ReactivePatternGenerator() : gen( 0 ), is_ready( 0 ){
        //初期化
        m_x0 = (Vec3*) new Vec3[3];
        std::fill( m_x0, m_x0+3, Vec3( Vec3Zero() ) );
        // スレッド起動
        if ( pthread_create( &m_thread, NULL, func, this ) ){
            // スレッド生成に失敗
            throw std::runtime_error( "::ReactivePatternGeneratorTask failed to pthread_create" );
        }
    }
    ~ReactivePatternGenerator() {
        // 終了フラグ
        m_waitcond.signal( true );
        // スレッド終了待ち
        if ( pthread_join ( m_thread, NULL ) ) {
            // スレッドのjoinに失敗
            throw std::runtime_error( "::~ReactivePatternGeneratorTask failed to pthread_join" );
        }
        delete m_x0;
    }
#else
#error "ReactivePatternGenerator() cannot build."
#endif
    /**
     * @param x0 コピーされる
     * ex.
     * const Vec3 x0[] = {
     *  Vec3( 0.0f, -0.095f, 0.0f ),
     *  Vec3( 0.0f,    0.0f, 0.0f ),
     *  Vec3( 0.4f,    0.0f, 0.0f )};
     */
    void start( const Vec3 x0[] ){
        std::cout << "[pr] " << MAKE_CHAR_COLOR_RED << "Calling ReactivePatternGenerator start()" << MAKE_CHAR_COLOR_DEFAULT << std::endl;
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

    ITrajectoryGenerator* getReady(){
        if( is_ready > 0 ){
            _mm_mfence();
            return gen;
        }else{
            return 0;
        }
    }

    // virtual functions from IPatternGenerator
    void incrementFrame( PatternState &state , ControlState &cstate){};
    bool isComplete() const {return true;};
    void update(  const UpdateState& state ){};
};

#if 0
struct ReactivePatternGeneratorInvoker {
    ReactivePatternGenerator stpf;
    bool is_readyToPush() const {
        return frame_offset > 4000;
    }

};
#endif

#endif /* __ReactivePatternGenerator_h__ */
