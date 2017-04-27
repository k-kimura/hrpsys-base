// -*- tab-width : 4 ; mode : C++ ; indent-tabs-mode : nil -*-
#ifndef _FRAME_RATE_MATCHER_H_
#define _FRAME_RATE_MATCHER_H_

class FrameRateMatcher {
    unsigned int source_frame_rate; /* Hz */
    unsigned int target_frame_rate; /* Hz */
    double convert_rate;
    unsigned int current_source_frame;
    unsigned int current_target_frame;
public:
    FrameRateMatcher(const unsigned int _source_frame_rate, const unsigned int _target_frame_rate)
        : source_frame_rate(_source_frame_rate), target_frame_rate(_target_frame_rate), current_target_frame(0), current_source_frame(0)
    {
        convert_rate = (double)target_frame_rate/(double)source_frame_rate;
    };
    void setCurrentFrame(const unsigned int cur_frame){
        current_source_frame = cur_frame;
        current_target_frame = cur_frame;
    }
    unsigned int getCurrentFrame(void){
        return current_source_frame;
    }
    unsigned int getConvertedFrame(void){
        return current_target_frame;
    }
    void incrementFrame() {
        current_source_frame++;
        current_target_frame = (unsigned int)((double)current_source_frame * convert_rate);
    }
};

#endif /* _FRAME_RATE_MATCHER_H_ */
