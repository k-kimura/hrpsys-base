// -*- tab-width : 4 ; mode : C++ ; indent-tabs-mode : nil -*-
/*!
 * @file  WheelController.h
 * @brief null component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef __WHEEL_CONTROLLER_H__
#define __WHEEL_CONTROLLER_H__
#include <iostream>
#include <memory>
#include "PushRecoverService_impl.h"
#include <vector>

namespace WheelLeg {

    class WheelController;
    typedef std::shared_ptr<WheelController> WheelControllerPtr;

    class WheelController {
    public:
        enum CTRL_MODE_TYPE {IDLE=0, KEEP_BALANCE=1};
        explicit WheelController(const int _drive_num) : drive_num(_drive_num), m_mode(IDLE), m_is_active(false)
        {
            m_param.drivenum = drive_num;
            m_param.param.length(drive_num);
            for(size_t i=0; i<drive_num; i++){
                /* Set Wheel ID */
                m_param.param[i].id = i;
                enable(i);
                for(size_t j=0; j<3; j++){
                    m_param.param[i].imu_offset[j] = 0.0;
                }
            }
            setPgain(1.0);
            setDgain(1.0);
            setTHgain(0.0);
            setPHIgain(0.0);
            setDTHgain(0.0);
            setDPHIgain(0.0);
            setOdom(0,0.0);
            setOdom(1,0.0);
            for(size_t k=0; k<3; k++){
                m_param.imu_zero[k] = 0.0;
            }
            m_param.complementary_filter_reset = true;
            m_param.complementary_filter_pitch = 0.0;
            m_param.phi_reset = false;

            m_context.resize(drive_num);
            for(auto it=m_context.begin(); it!=m_context.end(); it++){
                (*it).x = 0.0;
                (*it).prev_x = 0.0;
                (*it).dx = 0.0;
                (*it).brakeState = false;
                (*it).th = 0.0;
                (*it).phi = 0.0;
                (*it).dth = 0.0;
                (*it).dphi = 0.0;
            }
        };
        struct Context {
            double x;
            double prev_x;
            double dx;
            bool   brakeState;
            double th;
            double phi;
            double dth;
            double dphi;
        };
        bool setControlMode(const long mode){
            switch(mode){
            case 0:
                m_mode = IDLE;
                break;
            case 1:
                m_mode = KEEP_BALANCE;
                break;
            default:
                std::cerr << "[WheelController.setControlMode("<<mode<<") : Undefined mode" << std::endl;
                return false;
            }
            return true;
        };
        bool getParam(OpenHRP::PushRecoverService::WheelControllerParamSet& o_param){
            const size_t len = m_param.drivenum;
            o_param.drivenum = len;
            o_param.param.length(len);
            for(size_t i=0; i<len; i++){
                o_param.param[i].id = m_param.param[i].id;
                o_param.param[i].enable = m_param.param[i].enable;
                o_param.param[i].odom = m_param.param[i].odom;
                o_param.param[i].pgain = m_param.param[i].pgain;
                o_param.param[i].dgain = m_param.param[i].dgain;
                o_param.param[i].thgain = m_param.param[i].thgain;
                o_param.param[i].phigain = m_param.param[i].phigain;
                o_param.param[i].dthgain = m_param.param[i].dthgain;
                o_param.param[i].dphigain = m_param.param[i].dphigain;
                for(size_t j=0; j<3; j++){
                    o_param.param[i].imu_offset[j] = m_param.param[i].imu_offset[j];
                }
            }
            o_param.datal = 100; //test
            for(size_t k=0; k<3; k++){
                o_param.imu_zero[k] = m_param.imu_zero[k];
            }
            o_param.complementary_filter_reset = m_param.complementary_filter_reset;
            o_param.complementary_filter_pitch = m_param.complementary_filter_pitch;
            o_param.phi_reset = m_param.phi_reset;
            return true;
        };
        bool setParam(const OpenHRP::PushRecoverService::WheelControllerParamSet& i_param){
            const size_t len = (drive_num<=i_param.drivenum)?drive_num:i_param.drivenum;
            for(size_t i=0; i<len; i++){
                m_param.param[i] = i_param.param[i];
            }
            for(size_t k=0; k<3; k++){
                m_param.imu_zero[k] = i_param.imu_zero[k];
            }
            m_param.complementary_filter_reset = i_param.complementary_filter_reset;
            // m_param.complementary_filter_pitch = i_param.complementary_filter_pitch;
            m_param.phi_reset = i_param.phi_reset;
            return true;
        };
        void setPgain(const double gain){
            for(size_t i=0; i<drive_num; i++){
                m_param.param[i].pgain = gain;
            }
        };
        void setDgain(const double gain){
            for(size_t i=0; i<drive_num; i++){
                m_param.param[i].dgain = gain;
            }
        };
        void setTHgain(const double gain){
            for(size_t i=0; i<drive_num; i++){
                m_param.param[i].thgain = gain;
            }
        };
        void setPHIgain(const double gain){
            for(size_t i=0; i<drive_num; i++){
                m_param.param[i].phigain = gain;
            }
        };
        void setDTHgain(const double gain){
            for(size_t i=0; i<drive_num; i++){
                m_param.param[i].dthgain = gain;
            }
        };
        void setDPHIgain(const double gain){
            for(size_t i=0; i<drive_num; i++){
                m_param.param[i].dphigain = gain;
            }
        };
        /* Check if the controller is currently used or not. */
        bool isActive(){ return m_is_active; };
        void activate(){ m_is_active = true; };
        void deactivate(){ m_is_active = false; };
        void setOdom(const unsigned int index, const double odom){ m_param.param[index].odom = odom; }
        void enable(const unsigned int index){ m_param.param[index].enable = true; }
        void disable(const unsigned int index){ m_param.param[index].enable = false; }
        bool   brakeState(const unsigned int index){
            return m_context[index].brakeState;
        }
        double calcOutput(const unsigned int index, const double body_pitch, const double body_rate, const bool debug=false){
            double ret;
            const double max_pitch = 45 * 3.14159265/180.0;
            const OpenHRP::PushRecoverService::WheelControllerParam* p_param = &m_param.param[index];
            if(debug){
                std::cout << "wheelcontroller.calcOutput() : index = " << index << ", body_pitch = " << body_pitch << ", body_rate = " << body_rate << std::endl;
            }
            m_context[index].x = body_pitch;
            m_context[index].dx = body_rate;
            if(p_param->enable){
                switch(m_mode){
                case IDLE:
                    if(debug){
                        std::cout << "wheelcontroller.calcOutput() : Idle\n";
                    }
                    ret = 0.0;
                    m_context[index].brakeState = false;
                    break;
                case KEEP_BALANCE:
                    if( m_context[index].x < max_pitch && m_context[index].x > -max_pitch){
                        ret = p_param->pgain * (m_context[index].x - p_param->imu_offset[1]) + p_param->dgain * m_context[index].dx;
                        m_context[index].brakeState = false;
                        if(debug){
                            std::cout << "wheelcontroller.calcOutput() : keep_balance, ret = " << ret << ", x = " << m_context[index].x << ", pgain = " << p_param->pgain << std::endl;
                        }
                    }else{
                        ret = 0.0;
                        m_context[index].brakeState = true;
                    }
                    break;
                default:
                    ret = 0.0;
                    m_context[index].brakeState = true;
                    break;
                }
            }else{
                /* Disabled */
                if(debug){
                    std::cout << "wheelcontroller.calcOutput() : Disabled\n";
                }
                ret = 0.0;
                m_context[index].brakeState = false;
            }
            //m_context[index].dx = m_context[index].x - m_context[index].prev_x;
            const double max_val = 39.9;
            if(ret>max_val){
                ret = max_val;
            }else if(ret<-max_val){
                ret = -max_val;
            }
            return ret;
        };
        double stateFeedback(const unsigned int index, const double body_theta, const double wheel_phi, const double body_dtheta, const double wheel_dphi, const bool debug=false){
            double ret;
            // const double max_theta = 45 * 3.14159265/180.0;
            const double max_theta = 85 * 3.14159265/180.0;
            const OpenHRP::PushRecoverService::WheelControllerParam* p_param = &m_param.param[index];
            if(debug){
                std::cout << "wheelcontroller.stateFeedback() : index = " << index << ", body_theta = " << body_theta << ", wheel_phi = " << wheel_phi << ", body_dtheta = " << body_dtheta << ", wheel_dphi = " << wheel_dphi << std::endl;
            }
            m_context[index].th = body_theta;
            m_context[index].phi = wheel_phi;
            m_context[index].dth = body_dtheta;
            m_context[index].dphi = wheel_dphi;
            if(p_param->enable){
                switch(m_mode){
                case IDLE:
                    if(debug){
                        std::cout << "wheelcontroller.stateFeedback() : Idle\n";
                    }
                    ret = 0.0;
                    m_context[index].brakeState = false;
                    break;
                case KEEP_BALANCE:
                    if( m_context[index].th < max_theta && m_context[index].th > -max_theta ){
                        ret = - (p_param->thgain * m_context[index].th + p_param->phigain * m_context[index].phi + p_param->dthgain * m_context[index].dth + p_param->dphigain * m_context[index].dphi);
                        // ret = 2.0;
                        m_context[index].brakeState = false;
                        if(debug){
                            std::cout << "wheelcontroller.stateFeedback() : keep_balance, ret = " << ret << ", thgain = " << p_param->thgain << ", phigain = " << p_param->phigain << ", dthgain = " << p_param->dthgain << ", dphigain = " << p_param->dphigain << ", th = " << m_context[index].th << ", phi = " << m_context[index].phi << ", dth = " << m_context[index].dth << ", dphi = " << m_context[index].dphi << std::endl;
                        }
                    }else{
                        // std::cout << "|th| is Over max_theta, So ret = 0.0  and  brakeState = true" << std::endl;
                        ret = 0.0;
                        m_context[index].brakeState = true;
                    }
                    break;
                default:
                    ret = 0.0;
                    m_context[index].brakeState = true;
                    break;
                }
            }else{
                /* Disabled */
                if(debug){
                    std::cout << "wheelcontroller.stateFeedback() : Disabled\n";
                }
                ret = 0.0;
                m_context[index].brakeState = false;
            }
            // //m_context[index].dx = m_context[index].x - m_context[index].prev_x;
            // const double sat_val = 39.9;
            // if(ret > sat_val){
            //     std::cout << "[KIM warning] wheelcontroller.stateFeedback() : ret = " << ret << " is Over sat_val = " << sat_val << " !!!" << std::endl;
            //     ret = sat_val;
            // }else if(ret < -sat_val){
            //     std::cout << "[KIM warning] wheelcontroller.stateFeedback() : ret = " << ret << " is Under -sat_val = " << -sat_val << " !!!" << std::endl;
            //     ret = -sat_val;
            // }
            return ret;
        };
        hrp::Vector3 imuZero(){
            hrp::Vector3 ret;
            for(size_t k=0; k<3; k++){
                ret[k] = m_param.imu_zero[k];
            }
            return ret;
        };
        void setComplementaryFilterReset(const bool a){ m_param.complementary_filter_reset = a; };
        bool getComplementaryFilterReset(){ return m_param.complementary_filter_reset; };
        void setComplementaryFilterPitch(const double pitch){ m_param.complementary_filter_pitch = pitch; };
        void setPhiReset(const bool a){ m_param.phi_reset = a; };
        bool getPhiReset(){ return m_param.phi_reset; };
    private:
        const unsigned int drive_num;
        long m_mode;
        bool m_is_active;
        OpenHRP::PushRecoverService::WheelControllerParamSet m_param;
        std::vector<Context> m_context;
    };
}
#endif
