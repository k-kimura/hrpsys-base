// -*- tab-width : 4 ; mode : C++ ; indent-tabs-mode : nil -*-
#ifndef __pthread_waitcod_h__
#define __pthread_waitcod_h__

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


#endif /* Pthread_waitcond */
