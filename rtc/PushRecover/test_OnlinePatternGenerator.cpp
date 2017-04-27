#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "OnlinePatternGenerator.h"

void InitContext(BodyControlContext* _ct);

void main(void){
    OnlinePatternGenerator* p_gen = new OnlinePatternGenerator();
    PatternState state;
    BodyControlContext ctx;
    BodyControlContext cty;
    ControlState cstate(&ctx,&cty);
    JoyCommand command;
    Vec3 body_p_offset = Vec3(0.0f);
    Vec3 pref_offset = Vec3(0.0f);
    Vec3 body_p_prev = Vec3(0.0f);
    Vec3 pref_prev = Vec3(0.0f);

    {
        state.body_p = Vec3(0.0f);
        state.p = Vec3(0.0f);
    }

    InitContext(&ctx);
    InitContext(&cty);

    const std::size_t max_frame = 10000;
    for(int frame=0;frame < max_frame; frame++){
        if(p_gen->isComplete() == 1){
            body_p_offset += state.body_p;
            pref_offset += state.p;
        }
        if(frame>0 && frame <= 500){
            command.x = 0.0;
            command.y = 0.0;
        }else if(frame>500 && frame <= 2000){
            command.x = 0.2;
            command.y = 0.0;
        }else if(frame > 2000 && frame <= 5000){
            command.x = -0.2;
            command.y = -0.03;
        }else if(frame > 5000 && frame <= 6000){
            command.x = 0.0;
            command.y = 0.0;
        }else if(frame > 6000 && frame <= 8000){
            command.x = 0.0;
            command.y = +0.03;
        }else{
            command.x = 0.0;
            command.y = 0.0;
        }
        p_gen->command(command);
        p_gen->incrementFrame(state, cstate);
        const Vec3 rot_rp = Vec3(0.0f);
        const Vec3 rate = Vec3(0.0f);
        UpdateState ustate = { pref_prev, pref_prev, body_p_prev, Vec3(0.0f), rot_rp, rate };
        p_gen->update(ustate);
        cstate.pxstate->body_q[0] = state.body_p[0];
        cstate.pystate->body_q[0] = state.body_p[1];
        cstate.pxstate->body_q[1] = state.body_p[0] - body_p_prev[0];
        cstate.pystate->body_q[1] = state.body_p[1] - body_p_prev[1];
        pref_prev = state.p;
        body_p_prev = state.body_p;

#if 0
        if(frame%100==0){
            std::cout << "Frame[" << frame << "]" << std::endl;
            std::cout << "Pref=" << std::endl;
            std::cout << state.p << std::endl;
            std::cout << "Body_p=" << std::endl;
            std::cout << state.body_p << std::endl;
        }
#elif 1
        if(frame%1==0){
            std::cout << frame << ", " << state.p[0] << ", " << state.p[1] << ", " << state.p[2] << ", " << state.body_p[0] << ", " << state.body_p[1] << ", " << state.body_p[2] << ", " << state.foot_l_p[0] << ", " << state.foot_l_p[1] << ", " << state.foot_l_p[2] << ", " << state.foot_r_p[0] << ", " << state.foot_r_p[1] << ", " << state.foot_r_p[2] << ", " << pref_offset[0] << ", " << pref_offset[1] << ", " << pref_offset[2] << ", " << body_p_offset[0] << ", " << body_p_offset[1] << ", " << body_p_offset[2] << std::endl;
        }
#else
#endif
        usleep(1000);
    }
}


void InitContext(BodyControlContext* _ct){
    // for(std::size_t i=0; i<_ct->gain.size(); ++i){
    //     _ct->gain[i] = 1.0;
    // }
    // for(std::size_t i=0; i<_ct->gain.size(); ++i){
    //     _ct->body_q[i] = 0.0;
    // }
}
