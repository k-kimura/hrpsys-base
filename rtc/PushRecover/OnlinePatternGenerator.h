#ifndef __OnlinePatternGenerator_h__
#define __OnlinePatternGenerator_h__
#include <reactive_walk_generator.h>
#include <walking_pattern_generator.h>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_array.hpp>
#include <pthread.h>
#include "BodyIKMethod.h"
#include "SimpleLogger.h" /* for make_char_color macro */
#include "PThreadWaitcond.h"
#include "PatternGenerator.h"

class JoyCommand {
public:
    double x;
    double y;
    JoyCommand() : x(0.0), y(0.0) {};
};

class EAbortWalking : public std::runtime_error {
public:
    EAbortWalking() : std::runtime_error( "EAbortWalking" ){};
};


class OnlinePatternGenerator : public IPatternGenerator {
    pthread_t m_thread;
    ITrajectoryGenerator* gen;
    static void* func(void* arg);
    PThreadWaitCond<bool> m_waitcond;
    // 状態
    struct State : public Aligned<16> {
        const int rot_offsets_len;
        Vec3 com_p;
        Vec3 x_offset;
        Vec3 dx_offset;
        Vec3 rot;
        Vec3 filtered_rot;
        Vec3 lpf_rot;
        Vec3 rot_offset;
        boost::shared_array<Vec3> rot_offsets;
        int rot_offsets_index;
        State();
        Vec3 filtered_q0;
        Vec3 filtered_q1;
        Vec3 lpf_q0;
        Vec3 lpf_q1;
        float filter_coef;
        float lpf_coef;
        float first_magnify;
        float max_gain;
    };
    boost::scoped_ptr<State> m_pstate;
    WalkingPatternGenerator m_generator;
    int m_frame;
    int m_modify_frame;

    void signal_filter( Vec3 &state, const Vec3 input );
    void lpf_filter( Vec3 &state, const Vec3 input );
    float m_x_step;
    float m_y_step;
    float m_modify_thre;
    JoyCommand* m_command;
    bool isCommanded() const { return (m_x_step != 0.0f) || (m_y_step != 0.0f ); };
    int debug_counter;
public:
    OnlinePatternGenerator();
    ~OnlinePatternGenerator();
    void incrementFrame(PatternState &state, ControlState &cstate);
    bool isComplete() const;
    void update( const UpdateState& state);
    void command( const JoyCommand& command );
};


#endif /* __OnlinePatternGenerator_h__ */
