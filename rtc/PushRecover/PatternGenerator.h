#ifndef __PatternGenerator_H__
#define __PatternGenerator_H__


struct PatternState{
    Vec3 p;
    Vec3 body_p;
    Vec3 foot_l_p;
    Vec3 foot_r_p;
    Mat3 body_R;
    float gain_l_p;
    float gain_r_p;
    float foot_l_pitch;
    float foot_r_pitch;
    PatternState() : 
        body_p( QzVec3Zero() ),
        p( QzVec3Zero() ),
        foot_l_p( QzVec3Zero() ),
        foot_r_p( QzVec3Zero() ),
        body_R( Mat3::identity() ),
        gain_l_p( 1.0f ),
        gain_r_p( 1.0f ),
        foot_l_pitch( 0.0f ),
        foot_r_pitch( 0.0f )
    {};
};

struct UpdateState{
    Vec3 pref;
    Vec3 zmp;
    Vec3 com_p;
    Vec3 com_dp;
    Vec3 rot;
    Vec3 rate;
};

struct BodyControlContext {
    /// ゲイン
    boost::array<double,7> gain;
    /// body状態[q(n),dq(n),dDq(n),dDDq(n)]
    boost::array<double,4> body_q;
};

struct ControlState{
    BodyControlContext* pxstate;
    BodyControlContext* pystate;
ControlState(
             BodyControlContext* _pxstate,
             BodyControlContext* _pystate )
: pxstate( _pxstate ), pystate( _pystate )
    {}
};

class IPatternGenerator {
public:
    virtual void incrementFrame( PatternState &state, ControlState &cstate ) = 0;
    virtual bool isComplete() const = 0;
    virtual void update(  const UpdateState& state ) = 0;
    virtual ~IPatternGenerator(){};
};


#endif /*__PatternGenerator_H__*/
