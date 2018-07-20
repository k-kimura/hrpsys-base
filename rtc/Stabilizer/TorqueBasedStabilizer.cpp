#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include "Stabilizer.h"

typedef coil::Guard<coil::Mutex> Guard;

//#define DEBUG_HOGE
#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
#define DEBUGP2 (0)

void Stabilizer::torqueST()
{
    {
        struct timeval tv;
        gettimeofday(&tv,NULL);
        dlog.sectime   = ((float)(tv.tv_sec-stv.tv_sec)) + ((float)(tv.tv_usec - stv.tv_usec)/1000000.0f);
        dlog.frame = dlog.loop = (float)loop;
    }
    // Actual world frame =>
    hrp::Vector3 foot_origin_pos;
    hrp::Matrix33 foot_origin_rot;
    // update by current joint angles
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
        m_robot->joint(i)->q = m_qCurrent.data[i];
        m_robot->joint(i)->dq =  (m_robot->joint(i)->q - qold(i)) / dt;
        qold(i) = m_robot->joint(i)->q;
        dlog.act_q[i] = m_robot->joint(i)->q;
    }
    // tempolary
    m_robot->rootLink()->p = hrp::Vector3::Zero();
    m_robot->calcForwardKinematics();
    hrp::Sensor* sen = m_robot->sensor<hrp::RateGyroSensor>("gyrometer");
    hrp::Matrix33 senR = sen->link->R * sen->localR;
    hrp::Matrix33 act_Rs(hrp::rotFromRpy(m_rpy.data.r, m_rpy.data.p, m_rpy.data.y));
    m_robot->rootLink()->R = act_Rs * (senR.transpose() * m_robot->rootLink()->R);
    m_robot->calcForwardKinematics();
    calcFootOriginCoords (foot_origin_pos, foot_origin_rot);
#ifdef DEBUG_HOGE
    if(loop%500==0){
        std::cout << "[st] rootLink()->R = " << m_robot->rootLink()->R << std::endl;
    }
#endif
    hrp::Vector3 f_ga, tau_ga;
    std::vector<hrp::dvector6> ee_force, ee_force2;
    std::vector<int> enable_ee, enable_ee2;
    std::vector<int> enable_joint, enable_joint2;
    for (size_t i = 0; i < act_el_p.size(); i++) {
        if (ref_contact_states[i]) {
            enable_ee.push_back(i);
            hrp::JointPath jp(m_robot->rootLink(), m_robot->link(stikp[i].target_name));
            for (size_t j =0; j < jp.numJoints(); j++) {
                enable_joint.push_back(jp.joint(j)->jointId);
            }
        }
    }
    std::sort(enable_joint.begin(), enable_joint.end());
    enable_joint.erase(std::unique(enable_joint.begin(), enable_joint.end()), enable_joint.end());
    hrp::Matrix33 Kpp = hrp::Matrix33::Identity() * 450;
    Kpp(2, 2) *= 2;
    hrp::Matrix33 Kpd = ((Kpp.array() * 2 * m_robot->totalMass()).sqrt() * 1.6).matrix();
    hrp::Matrix33 Krp = hrp::Matrix33::Identity() * 500;
    hrp::Matrix33 Krd = hrp::Matrix33::Identity() * 50;
    generateForce(foot_origin_rot, Kpp, Kpd, Krp, Krd, f_ga, tau_ga);
    {
        dlog.tqst_force_com0  = dlog::V3(f_ga);
        dlog.tqst_moment_com0 = dlog::V3(tau_ga);
#ifdef DEBUG_HOGE
        if(loop%500==0){
            std::cout << "[st] act_cog  = " << act_cog.transpose() << std::endl;
            std::cout << "[st] ref_cog  = " << ref_cog.transpose() << std::endl;
            std::cout << "[st] f_org_ro = \n" << foot_origin_rot << std::endl;
            std::cout << "[st] f*(ac-rc) = " << (foot_origin_rot * (act_cog - ref_cog)).transpose() << std::endl;
        }
#endif
#ifdef DEBUG_HOGE
        if(loop%500==0){
          std::cout << "[st] force_com0 = " << f_ga.transpose() << std::endl;
          std::cout << "[st] moment_com0 = " << tau_ga.transpose() << std::endl;
        }
#endif
    }
    std::vector<hrp::dmatrix> Gc1;
    std::vector<hrp::dmatrix> Gc2;
    std::vector<hrp::dvector6> tmp_f;
    hrp::Vector3 f_foot = hrp::Vector3::Zero();
    hrp::Vector3 tau_foot = hrp::Vector3::Zero();
    Kpp = hrp::Matrix33::Identity() * 500;
    Kpd = hrp::Matrix33::Identity() * 50;
    Krp = hrp::Matrix33::Identity() * 100;
    Krd = hrp::Matrix33::Identity() * 10;
    for (size_t i = 0; i < act_el_p.size(); i++) {
        if (!ref_contact_states[i]) {
            Gc1.push_back(hrp::dmatrix::Zero(3, 6));
            Gc2.push_back(hrp::dmatrix::Zero(3, 6));
            Gc1.back().block(0, 0, 3, 3) = act_el_R[i];
            Gc2.back().block(0, 0, 3, 3) = hrp::hat(act_el_p[i] - act_cog) * act_ee_R[i];
            Gc1.back().block(0, 3, 3, 3) = hrp::dmatrix::Zero(3, 3);
            Gc2.back().block(0, 3, 3, 3) = act_el_R[i];
            generateSwingFootForce(Kpp, Kpd, Krp, Krd, f_foot, tau_foot, i);
            tmp_f.push_back(hrp::dvector6::Zero());
            tmp_f.back() << f_foot, tau_foot;
            f_ga -= Gc1.back() * tmp_f.back();
            tau_ga -= Gc2.back() * tmp_f.back();
        }
    }
    {
        dlog.tqst_force_com1  = dlog::V3(f_ga);
        dlog.tqst_moment_com1 = dlog::V3(tau_ga);
#ifdef DEBUG_HOGE
        if(loop%500==0){
            std::cout << "[st] force_com1 = " << f_ga.transpose() << std::endl;
            std::cout << "[st] moment_com1 = " << tau_ga.transpose() << std::endl;
        }
#endif
        for( std::size_t i = 0; i<tmp_f.size(); i++){
            const hrp::dvector6 tmp = tmp_f[i];
            dlog.tqst_force[i]  = dlog::V3(tmp[0],tmp[1],tmp[2]);
            dlog.tqst_moment[i] = dlog::V3(tmp[3],tmp[4],tmp[5]);
#ifdef DEBUG_HOGE
            if(loop%500==0){
                std::cout << "[st] force_moment["<<i<<"] = " << tmp.transpose() << std::endl;
            }
#endif
        }
    }
    size_t k = 0;
    size_t l = 0;
    distributeForce(f_ga, tau_ga, enable_ee, enable_joint, ee_force);
    for (size_t i = 0; i < act_el_p.size(); i++) {
        enable_ee2.push_back(i);
        hrp::JointPath jp(m_robot->rootLink(), m_robot->link(stikp[i].target_name));
        for (size_t j =0; j < jp.numJoints(); j++) {
            enable_joint2.push_back(jp.joint(j)->jointId);
        }
        if (ref_contact_states[i]) {
            ee_force2.push_back(ee_force[k]);
            k++;
        } else {
            ee_force2.push_back(tmp_f[l]);
            l++;
        }
    }
    std::sort(enable_joint2.begin(), enable_joint2.end());
    enable_joint2.erase(std::unique(enable_joint2.begin(), enable_joint2.end()), enable_joint2.end());
    calcForceMapping(ee_force2, enable_ee2, enable_joint2);

    {
        dlog.contact_state = dlog::V3(ref_contact_states[0],ref_contact_states[1],0.0f);

        for(std::size_t i = 0; i<ee_force2.size(); i++){
            if(i>=4){
                std::cerr << "[st] ee_force2.size() overed 4." << std::endl;
            }else{
                const hrp::dvector6 tmp = ee_force2[i];
                dlog.ee_force[i]  = dlog::V3(tmp[0], tmp[1], tmp[2]);
                dlog.ee_moment[i] = dlog::V3(tmp[3], tmp[4], tmp[5]);
#ifdef DEBUG_HOGE
                if(loop%500==0){
                    std::cout << "[st] ee_force_moment["<<i<<"] = " << tmp.transpose() << std::endl;
                }
#endif
            }
        }
        for(std::size_t i = 0; i<12; i++){
            dlog.tau_ref[i] = m_robot->joint(i)->u;
        }
    }

    for ( int i = 0; i < m_robot->numJoints(); i++ ){
        m_robot->joint(i)->q = qrefv[i];
        dlog.ref_q[i]        = qrefv[i];
    }
    m_robot->rootLink()->p = target_root_p;
    m_robot->rootLink()->R = target_root_R;
    if ( !(control_mode == MODE_IDLE || control_mode == MODE_AIR) ) {
        for (size_t i = 0; i < jpe_v.size(); i++) {
            if (is_ik_enable[i]) {
                for ( int j = 0; j < jpe_v[i]->numJoints(); j++ ){
                    int idx = jpe_v[i]->joint(j)->jointId;
                    m_robot->joint(idx)->q = qorg[idx];
                }
            }
        }
        m_robot->rootLink()->p(0) = current_root_p(0);
        m_robot->rootLink()->p(1) = current_root_p(1);
        m_robot->rootLink()->R = current_root_R;
        m_robot->calcForwardKinematics();
    }

    dlogger.push(dlog);
};

void Stabilizer::generateForce(const hrp::Matrix33& foot_origin_rot, const hrp::Matrix33& Kpp, const hrp::Matrix33& Kpd, const hrp::Matrix33 Krp, const hrp::Matrix33 Krd, hrp::Vector3& f_ga, hrp::Vector3& tau_ga)
{
    hrp::Vector3 g(0, 0, 9.80665);

    //calc f_ga
    //world frame
    f_ga = m_robot->totalMass() * g - Kpp * foot_origin_rot * (act_cog - ref_cog) - Kpd * foot_origin_rot * (act_cogvel - ref_cogvel);
    //foot origin frame
    f_ga = foot_origin_rot.transpose() * f_ga;

    //fix act_base_R
    hrp::Vector3 foot_origin_rpy = hrp::rpyFromRot(foot_origin_rot);
    hrp::Matrix33 zrot = hrp::rotationZ(foot_origin_rpy(2) - act_base_rpy(2));
    hrp::Matrix33 new_act_base_R = zrot.transpose() * act_base_R;
    //calc tau_ga
    Eigen::Quaternion<double> q(target_root_R.transpose() * new_act_base_R);
    hrp::Vector3 e = q.vec();
    double d = q.w();
    hrp::Vector3 tau_r = -2 * (d * hrp::Matrix33::Identity() + hrp::hat(e)) * Krp * e;
    //world frame
    tau_ga = new_act_base_R * (tau_r - Krd * act_base_omega);
    //foot origin frame
    tau_ga = foot_origin_rot.transpose() * tau_ga;
};

void Stabilizer::generateSwingFootForce(const hrp::Matrix33& Kpp, const hrp::Matrix33& Kpd, const hrp::Matrix33 Krp, const hrp::Matrix33 Krd, hrp::Vector3& f_foot, hrp::Vector3& tau_foot, size_t i)
{
    f_foot = act_el_R[i].transpose() * Kpp * (act_el_p[i] - ref_el_p[i]) + act_el_R[i].transpose() * Kpd * (act_el_vel[i] - ref_el_vel[i]);
    Eigen::Quaternion<double> q(ref_el_R[i].transpose() * act_el_R[i]);
    hrp::Vector3 e = q.vec();
    double d = q.w();
    tau_foot = 2 * (d * hrp::Matrix33::Identity() + hrp::hat(e)) * Krp * e + Krd * act_el_omega[i];
};

void Stabilizer::distributeForce(const hrp::Vector3& f_ga, const hrp::Vector3& tau_ga, const std::vector<int>& enable_ee, const std::vector<int>& enable_joint, std::vector<hrp::dvector6>& ee_force)
{
    hrp::dvector6 f_tau;
    f_tau << f_ga, tau_ga;
    size_t ee_num = enable_ee.size();
    size_t state_dim = 6 * ee_num;
    double a = 100, b = 100, c = 1;

    //calc Gc
    hrp::dmatrix Gc(6, state_dim);
    for (size_t i = 0; i < ee_num; i++) {
        Gc.block(0, i * 6, 3, 3) = act_el_R[enable_ee[i]];
        Gc.block(3, i * 6, 3, 3) = hrp::hat(act_el_p[enable_ee[i]]  - act_cog) * act_el_R[enable_ee[i]];
        Gc.block(0, i * 6 + 3, 3, 3) = hrp::dmatrix::Zero(3, 3);
        Gc.block(3, i * 6 + 3, 3, 3) = act_el_R[enable_ee[i]];
    }
    hrp::Vector3 foot_origin_pos;
    hrp::Matrix33 foot_origin_rot;
    calcFootOriginCoords(foot_origin_pos, foot_origin_rot);
    hrp::dmatrix tmp_matrix;
    calcEforce2ZmpMatrix(tmp_matrix, enable_ee, zmp_origin_off+foot_origin_pos(2));
    hrp::dmatrix Gc2 = tmp_matrix.block(0, 0, 2, state_dim) - ref_zmp.segment(0, 2) * tmp_matrix.block(2, 0, 1, state_dim);

    //calc QP param
    hrp::dmatrix I1(6, 6);
    I1 << hrp::dmatrix::Identity(3, 3) * a, hrp::dmatrix::Zero(3, 3),
        hrp::dmatrix::Zero(3, 3), hrp::dmatrix::Identity(3, 3) * b;
    hrp::dmatrix I2 = hrp::dmatrix::Identity(state_dim, state_dim)*c;

    //joint torque constraint
    hrp::dmatrix ef2tau;
    calcEforce2TauMatrix(ef2tau, enable_ee, enable_joint);
    size_t tau_dim = enable_joint.size();
    hrp::dvector upperTauLimit;
    hrp::dvector lowerTauLimit;
    double pgain[] = {3300, 8300, 3300, 3300, 4700, 3300, 3300, 8300, 3300, 3300, 4700, 3300};
    double dgain[] = {24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24};
    makeJointTorqueLimit(tau_dim, enable_joint, pgain, dgain, upperTauLimit, lowerTauLimit);

    //friction constraint
    hrp::dmatrix friction;
    hrp::dvector upperFrictionLimit;
    hrp::dvector lowerFrictionLimit;
    size_t friction_dim = makeFrictionConstraint(enable_ee, 0.9, friction, upperFrictionLimit, lowerFrictionLimit);

    //cop constraint
    hrp::dmatrix cop;
    hrp::dvector upperCopLimit;
    hrp::dvector lowerCopLimit;
    size_t cop_dim = makeCopConstraint(enable_ee, cop, upperCopLimit, lowerCopLimit);

    //tau_z constraint
    hrp::dmatrix tauz;
    hrp::dvector upperTauzLimit;
    hrp::dvector lowerTauzLimit;
    size_t tauz_dim = makeTauz2Constraint(enable_ee, 0.9, tauz, upperTauzLimit, lowerTauzLimit);

    size_t const_dim = tau_dim + friction_dim + cop_dim + tauz_dim;
    Eigen::Matrix<double, -1, -1, Eigen::RowMajor> Const(const_dim, state_dim);
    Const << ef2tau, friction, cop, tauz;
    hrp::dvector upperConstLimit(const_dim);
    hrp::dvector lowerConstLimit(const_dim);
    upperConstLimit << upperTauLimit, upperFrictionLimit, upperCopLimit, upperTauzLimit;
    lowerConstLimit << lowerTauLimit, lowerFrictionLimit, lowerCopLimit, lowerTauzLimit;

    Eigen::Matrix<double, -1, -1, Eigen::RowMajor> Q = Gc.transpose() * I1 * Gc + I2 + 5 * Gc2.transpose() * Gc2;
    Eigen::Matrix<double, -1, -1, Eigen::RowMajor> C = -Gc.transpose() * I1 * f_tau;
    hrp::dmatrix upperStateLimit(6, ee_num);
    hrp::dmatrix lowerStateLimit(6, ee_num);
    for (size_t i = 0; i < ee_num; i++) {
        upperStateLimit(0, i) =  1e10;
        upperStateLimit(1, i) =  1e10;
        upperStateLimit(2, i) =  1e10;
        upperStateLimit(3, i) =  1e10;
        upperStateLimit(4, i) =  1e10;
        upperStateLimit(5, i) =  1e10;
        lowerStateLimit(0, i) = -1e10;
        lowerStateLimit(1, i) = -1e10;
        lowerStateLimit(2, i) =    0;
        lowerStateLimit(3, i) = -1e10;
        lowerStateLimit(4, i) = -1e10;
        lowerStateLimit(5, i) = -1e10;
    }
    hrp::dmatrix output(6, ee_num);

    qpOASES::real_t* H = (qpOASES::real_t*)Q.data();
    qpOASES::real_t* g = (qpOASES::real_t*)C.data();
    qpOASES::real_t* A = (qpOASES::real_t*)Const.data();
    qpOASES::real_t* lb = (qpOASES::real_t*)lowerStateLimit.data();
    qpOASES::real_t* ub = (qpOASES::real_t*)upperStateLimit.data();
    qpOASES::real_t* lbA = (qpOASES::real_t*)lowerConstLimit.data();
    qpOASES::real_t* ubA = (qpOASES::real_t*)upperConstLimit.data();
    qpOASES::real_t* xOpt = (qpOASES::real_t*)output.data();

    //solve QP
    if (qp_solver.getNV() != state_dim || qp_solver.getNC() != const_dim) {
        qp_solver = qpOASES::SQProblem(state_dim, const_dim);
    }
    qpOASES::Options options;
    options.printLevel = qpOASES::PL_NONE;
    qp_solver.setOptions(options);
    int nWSR = 100;
    qpOASES::real_t time = 0.0000;
    qpOASES::returnValue status;
    if (qp_solver.isInitialised()) {
        status = qp_solver.hotstart(H, g, A, lb, ub, lbA, ubA, nWSR, &time);
    } else {
        status = qp_solver.init(H, g, A, lb, ub, lbA, ubA, nWSR);
    }
    int qpcounter = 1;
    while (!qp_solver.isSolved() && !qp_solver.isInfeasible() && qpcounter < state_dim + const_dim) {
        status = qp_solver.hotstart(g, lb, ub, lbA, ubA, nWSR, &time);
        qpcounter++;
    }
    qp_solver.getPrimalSolution(xOpt);
    for (size_t i = 0; i < ee_num; i++) {
        ee_force.push_back(output.col(i));
    }
    if (DEBUGP) {
        hrp::dvector y(state_dim+const_dim);
        qpOASES::real_t* yOpt = (qpOASES::real_t*)y.data();
        qp_solver.getDualSolution(yOpt);
        size_t tmp_index = 0;
        Eigen::Map<hrp::dmatrix> tmp1(y.segment(tmp_index, state_dim).data(), 6, ee_num);
        tmp_index += state_dim;
        Eigen::Map<hrp::dvector> tmp2(y.segment(tmp_index, tau_dim).data(), tau_dim);
        tmp_index += tau_dim;
        Eigen::Map<hrp::dmatrix> tmp3(y.segment(tmp_index, friction_dim).data(), 4, ee_num);
        tmp_index += friction_dim;
        Eigen::Map<hrp::dvector> tmp4(y.segment(tmp_index, cop_dim).data(), cop_dim);
        tmp_index += cop_dim;
        Eigen::Map<hrp::dvector> tmp5(y.segment(tmp_index, tauz_dim).data(), 8, ee_num);
        tmp_index += tauz_dim;
        std::cerr << "[" << m_profile.instance_name << "] qp result" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "] try num = " << qpcounter << std::endl;
        if (qp_solver.isInfeasible()) {
            std::cerr << "[" << m_profile.instance_name << "] optimization failed " << qpcounter << std::endl;
        }
        std::cerr << "[" << m_profile.instance_name << "] end efector force =" << std::endl;
        std::cerr << output.transpose() << std::endl;
        std::cerr << "[" << m_profile.instance_name << "] end efector force and friction limit" << std::endl;
        for (size_t i = 0; i < ee_num; i++) {
            std::string axis1[6] = {"fx", "fy", "fz", "tx", "ty", "tz"};
            std::string axis2[4] = {"x lower", "x upper", "y lower", "y upper"};
            std::string axis3[8] = {"lower", "lower", "lower", "lower", "upper", "upper", "upper", "upper"};
            for (size_t j = 0; j < 6; j++){
                if (tmp1(j, i) > 0)
                    std::cerr << "[" << m_profile.instance_name << "]     ee("
                              << i << ")." << axis1[j] << " lower limit" << std::endl;
                else if (tmp1(j, i) < 0)
                    std::cerr << "[" << m_profile.instance_name << "]     ee("
                              << i << ")." << axis1[j] << " upper limit" << std::endl;
            }
            for (size_t j = 0; j < 4; j++){
                if (tmp3(j, i) > 0)
                    std::cerr << "[" << m_profile.instance_name << "]     friction("
                              << i << ")." << axis2[j] << " limit" << std::endl;
            }
            for (size_t j = 0; j < 8; j++){
                if (tmp5(j, i) > 0)
                    std::cerr << "[" << m_profile.instance_name << "]     tau_z friction("
                              << i << ")." << axis3[j] << " limit" << std::endl;
            }
        }
        std::cerr << "[" << m_profile.instance_name << "] torque limit" << std::endl;
        for (size_t i = 0; i < tau_dim; i++) {
            if (tmp2(i) > 0)
                std::cerr << "[" << m_profile.instance_name << "]     torque(" << i << ") lower limit" << std::endl;
            else if (tmp2(i) < 0)
                std::cerr << "[" << m_profile.instance_name << "]     torque(" << i << ") upper limit" << std::endl;
        }
        std::cerr << "[" << m_profile.instance_name << "] cop limit" << std::endl;
        for (size_t i = 0; i < cop_dim; i++) {
            if (tmp4(i) > 0)
                std::cerr << "[" << m_profile.instance_name << "]     side(" << i << ") limit" << std::endl;
        }
        std::cerr << "[" << m_profile.instance_name << "] optimized value = " << qp_solver.getObjVal() << std::endl;
    }
};

size_t Stabilizer::makeFrictionConstraint(const std::vector<int>& enable_ee, double coef, hrp::dmatrix& const_matrix, hrp::dvector& upper_limit, hrp::dvector& lower_limit)
{
    size_t ee_num = enable_ee.size();
    const_matrix = hrp::dmatrix::Zero(4 * ee_num, 6 * ee_num);
    for (size_t i = 0; i < ee_num; i++){
        const_matrix.block(i * 4, i * 6, 4, 6)
            << 1, 0,  coef, 0, 0, 0,
            -1, 0, coef, 0, 0, 0,
            0,  1, coef, 0, 0, 0,
            0, -1, coef, 0, 0, 0;
        hrp::dmatrix convert_matrix = hrp::dmatrix::Identity(6, 6);
        convert_matrix.block(3, 0, 3, 3) = - hrp::hat(stikp[enable_ee[i]].localp);
        hrp::dmatrix tmpR = hrp::dmatrix::Zero(6, 6);
        tmpR.block(0, 0, 3, 3) = stikp[enable_ee[i]].localR.transpose();
        tmpR.block(3, 3, 3, 3) = stikp[enable_ee[i]].localR.transpose();
        convert_matrix = tmpR * convert_matrix;
        const_matrix.block(i * 4, i * 6, 4, 6) = const_matrix.block(i * 4, i * 6, 4, 6) * convert_matrix;
    }
    upper_limit = hrp::dvector::Ones(4 * ee_num) * 1e10;
    lower_limit = hrp::dvector::Zero(4 * ee_num);
    return 4 * ee_num;
};

size_t Stabilizer::makeTauzConstraint(const std::vector<int>& enable_ee, double coef, hrp::dmatrix& const_matrix, hrp::dvector& upper_limit, hrp::dvector& lower_limit)
{
    size_t ee_num = enable_ee.size();
    const_matrix = hrp::dmatrix::Zero(2 * ee_num, 6 * ee_num);
    for (size_t i = 0; i < ee_num; i++){
        double x1, x2, y1, y2;
        x1 = szd->get_leg_front_margin();
        x2 = szd->get_leg_rear_margin();
        if (stikp[enable_ee[i]].ee_name == "rleg") {
            y1 = szd->get_leg_inside_margin();
            y2 = szd->get_leg_outside_margin();
        } else {
            y1 = szd->get_leg_outside_margin();
            y2 = szd->get_leg_outside_margin();
        }
        const_matrix.block(i * 2, i * 6, 2, 6)
            << 0.5*(y1-y2), 0.5*(x2-x1), 0.5*coef*(x1+x2+y1+y2), 0, 0,  1,
            0.5*(y2-y1), 0.5*(x1-x2), 0.5*coef*(x1+x2+y1+y2), 0, 0, -1;
        hrp::dmatrix convert_matrix = hrp::dmatrix::Identity(6, 6);
        convert_matrix.block(3, 0, 3, 3) = - hrp::hat(stikp[enable_ee[i]].localp);
        hrp::dmatrix tmpR = hrp::dmatrix::Zero(6, 6);
        tmpR.block(0, 0, 3, 3) = stikp[enable_ee[i]].localR.transpose();
        tmpR.block(3, 3, 3, 3) = stikp[enable_ee[i]].localR.transpose();
        convert_matrix = tmpR * convert_matrix;
        const_matrix.block(i * 2, i * 6, 2, 6) = const_matrix.block(i * 2, i * 6, 2, 6) * convert_matrix;
    }
    upper_limit = hrp::dvector::Ones(2 * ee_num) * 1e10;
    lower_limit = hrp::dvector::Zero(2 * ee_num);
    return 2 * ee_num;
};

size_t Stabilizer::makeTauz2Constraint(const std::vector<int>& enable_ee, double coef, hrp::dmatrix& const_matrix, hrp::dvector& upper_limit, hrp::dvector& lower_limit)
{
    size_t ee_num = enable_ee.size();
    const_matrix = hrp::dmatrix::Zero(8 * ee_num, 6 * ee_num);
    for (size_t i = 0; i < ee_num; i++){
        double x1, x2, y1, y2, x, y, x_, y_;
        hrp::dmatrix convert_matrix = hrp::dmatrix::Identity(6, 6);
        x1 = szd->get_leg_front_margin();
        x2 = szd->get_leg_rear_margin();
        if (stikp[enable_ee[i]].ee_name == "rleg") {
            y1 = szd->get_leg_inside_margin();
            y2 = szd->get_leg_outside_margin();
        } else {
            y1 = szd->get_leg_outside_margin();
            y2 = szd->get_leg_outside_margin();
        }
        x = (x1 + x2) / 2;
        y = (y1 + y2) / 2;
        x_ = (x1 - x2) / 2;
        y_ = (y1 - y2) / 2;
        const_matrix.block(i * 8, i * 6, 8, 6) << y, x, coef * (x + y), -coef, -coef, 1,
            -y, x, coef * (x + y), coef, -coef, 1,
            y, -x, coef * (x + y), -coef, coef, 1,
            -y, -x, coef * (x + y), coef, coef, 1,
            y, x, coef * (x + y), coef, coef, -1,
            -y, x, coef * (x + y), -coef, coef, -1,
            y, -x, coef * (x + y), coef, -coef, -1,
            -y, -x, coef * (x + y), -coef, -coef, -1;
        convert_matrix.block(3, 0, 3, 3)
            << 0, 0, -y_,
            0, 0, x_,
            y_, -x_, 0;
        const_matrix.block(i * 8, i * 6, 8, 6) = const_matrix.block(i * 8, i * 6, 8, 6) * convert_matrix;
        convert_matrix = hrp::dmatrix::Identity(6, 6);
        convert_matrix.block(3, 0, 3, 3) = - hrp::hat(stikp[enable_ee[i]].localp);
        hrp::dmatrix tmpR = hrp::dmatrix::Zero(6, 6);
        tmpR.block(0, 0, 3, 3) = stikp[enable_ee[i]].localR.transpose();
        tmpR.block(3, 3, 3, 3) = stikp[enable_ee[i]].localR.transpose();
        convert_matrix = tmpR * convert_matrix;
        const_matrix.block(i * 8, i * 6, 8, 6) = const_matrix.block(i * 8, i * 6, 8, 6) * convert_matrix;
    }
    upper_limit = hrp::dvector::Ones(8 * ee_num) * 1e10;
    lower_limit = hrp::dvector::Zero(8 * ee_num);
    return 8 * ee_num;
};

void Stabilizer::makeJointTorqueLimit(size_t num, const std::vector<int>& enable_joint, double pgain[], double dgain[], hrp::dvector& upper_limit, hrp::dvector& lower_limit)
{
    upper_limit = hrp::dvector(num);
    lower_limit = hrp::dvector(num);
    for (size_t i = 0; i < num; i++) {
        double tlimit = m_robot->joint(enable_joint[i])->climit * m_robot->joint(enable_joint[i])->gearRatio * m_robot->joint(enable_joint[i])->torqueConst;
        double ulimit = -pgain[enable_joint[i]] * (m_robot->joint(enable_joint[i])->q - m_robot->joint(enable_joint[i])->ulimit)
            -dgain[enable_joint[i]] * m_robot->joint(enable_joint[i])->dq;
        double llimit = -pgain[enable_joint[i]] * (m_robot->joint(enable_joint[i])->q - m_robot->joint(enable_joint[i])->llimit)
            -dgain[enable_joint[i]] * m_robot->joint(enable_joint[i])->dq;
        upper_limit(i) =  std::max(std::min(tlimit, ulimit), -tlimit);
        lower_limit(i) =  std::min(std::max(-tlimit, llimit), tlimit);
    }
};

size_t Stabilizer::makeCopConstraint(const std::vector<int>& enable_ee, hrp::dmatrix& const_matrix, hrp::dvector& upper_limit, hrp::dvector& lower_limit)
{
    size_t ee_num = enable_ee.size();
    std::vector<std::vector<Eigen::Vector2d> > support_polygon_vertices;
    szd->get_vertices(support_polygon_vertices);
    size_t const_dim = 0;
    for (size_t i = 0; i < ee_num; i++) {
        const_dim += support_polygon_vertices[enable_ee[i]].size();
    }
    const_matrix = hrp::dmatrix::Zero(const_dim, 6 * ee_num);
    size_t index = 0;

    for (size_t i = 0; i < ee_num; i++) {
        size_t ver_num = support_polygon_vertices[enable_ee[i]].size();
        //calc matrix to check if zmp is inside support polygon
        hrp::dmatrix check_matrix(ver_num, 3);
        for (size_t j = 0; j < ver_num - 1; j++) {
            check_matrix.block(j, 0, 1, 3) <<
                support_polygon_vertices[enable_ee[i]][j+1](1) - support_polygon_vertices[enable_ee[i]][j](1),
                support_polygon_vertices[enable_ee[i]][j](0) - support_polygon_vertices[enable_ee[i]][j+1](0),
                support_polygon_vertices[enable_ee[i]][j+1](0) * support_polygon_vertices[enable_ee[i]][j](1)
                - support_polygon_vertices[enable_ee[i]][j+1](1) * support_polygon_vertices[enable_ee[i]][j](0);
        }
        check_matrix.block(ver_num-1, 0, 1, 3) <<
            support_polygon_vertices[enable_ee[i]].front()(1) - support_polygon_vertices[enable_ee[i]].back()(1),
            support_polygon_vertices[enable_ee[i]].back()(0) - support_polygon_vertices[enable_ee[i]].front()(0),
            support_polygon_vertices[enable_ee[i]].front()(0) * support_polygon_vertices[enable_ee[i]].back()(1)
            - support_polygon_vertices[enable_ee[i]].front()(1) * support_polygon_vertices[enable_ee[i]].back()(0);
        //calc COP matrix
        hrp::dmatrix cop_num_den(3, 6);
        hrp::Link* target = m_robot->link(stikp[enable_ee[i]].target_name);
        hrp::Vector3 tp = - stikp[enable_ee[i]].localR.transpose() * stikp[enable_ee[i]].localp;
        cop_num_den <<
            -tp(2), 0, tp(0), 0, -1, 0,
            0, -tp(2), tp(1), 1,  0, 0,
            0, 0, 1, 0, 0, 0;
        hrp::dmatrix tmpR = hrp::dmatrix::Zero(6, 6);
        tmpR.block(0, 0, 3, 3) = stikp[enable_ee[i]].localR.transpose();
        tmpR.block(3, 3, 3, 3) = stikp[enable_ee[i]].localR.transpose();
        cop_num_den = cop_num_den * tmpR;
        const_matrix.block(index, 6 * i, ver_num, 6) = check_matrix * cop_num_den;
        index += ver_num;
    }

    upper_limit = hrp::dvector(const_dim);
    lower_limit = hrp::dvector(const_dim);
    for (size_t i = 0; i < const_dim; i++) {
        upper_limit(i) = 10e10;
        lower_limit(i) = 0;
    }
    return const_dim;
};

void Stabilizer::calcEforce2ZmpMatrix(hrp::dmatrix& ret, const std::vector<int>& enable_ee, const double zmp_z)
{
    size_t ee_num = enable_ee.size();
    ret = hrp::dmatrix(3, 6 * ee_num);
    for (size_t i = 0; i < ee_num; i++) {
        hrp::Link* target = m_robot->link(stikp[enable_ee[i]].target_name);
        hrp::dmatrix tmpR(6, 6);
        tmpR << target->R, hrp::dmatrix::Zero(3, 3), hrp::dmatrix::Zero(3, 3), target->R;
        ret.block(0, i * 6, 2, 6) <<
            -(target->p(2) - zmp_z), 0, target->p(0), 0, -1, 0,
            0, -(target->p(2) - zmp_z), target->p(1), 1,  0, 0;
        ret.block(2, i * 6, 1, 6) <<
            0, 0, 1, 0, 0, 0;
        ret.block(0, i * 6, 3, 6) = ret.block(0, i * 6, 3, 6) * tmpR;
    }
};

void Stabilizer::calcEforce2TauMatrix(hrp::dmatrix& ret, const std::vector<int>& enable_ee, const std::vector<int>& enable_joint)
{
    size_t ee_num = enable_ee.size();
    size_t joint_num = enable_joint.size();
    hrp::dmatrix J = hrp::dmatrix::Zero(6 * ee_num, joint_num);
    hrp::dmatrix CMJ = hrp::dmatrix::Zero(6 * ee_num, joint_num);
    hrp::dmatrix CMJ_tmp;
    m_robot->calcCM();
    m_robot->calcCMJacobian(NULL, CMJ_tmp);

    for (size_t i = 0; i < ee_num; i++) {
        //Jacobian
        hrp::Link* target = m_robot->link(stikp[enable_ee[i]].target_name);
        hrp::JointPath jp(m_robot->rootLink(), target);
        hrp::dmatrix JJ;
        jp.calcJacobian(JJ);
        //convert to end link frame
        hrp::dmatrix rotate = hrp::dmatrix::Zero(6, 6);
        rotate.block(0, 0, 3, 3) = target->R.transpose();
        rotate.block(3, 3, 3, 3) = target->R.transpose();
        JJ = rotate * JJ;
        for (size_t j = 0; j < jp.numJoints(); j++) {
            for (size_t k = 0; k < enable_joint.size(); k++) {
                if (enable_joint[k] == jp.joint(j)->jointId) {
                    J.block(i * 6, k, 6, 1) = JJ.col(j);
                    break;
                }
            }
        }

        //CM jacobian
        //convert to end link frame
        for (size_t j = 0; j < m_robot->numJoints(); j++) {
            for (size_t k = 0; k < enable_joint.size(); k++) {
                if (enable_joint[k] == j) {
                    CMJ.block(i * 6, k, 3, 1) = target->R.transpose() * CMJ_tmp.col(j);
                    break;
                }
            }
        }
    }
    ret = -(J-CMJ).transpose();
};

void Stabilizer::calcForceMapping(const std::vector<hrp::dvector6> ee_force, const std::vector<int>& enable_ee, const std::vector<int>& enable_joint)
{
    size_t ee_num = ee_force.size();
    hrp::dvector total_ee_force(6 * ee_num);
    for (size_t i = 0; i < ee_num; i++) {
        total_ee_force.block(i * 6, 0, 6, 1) = ee_force[i];
    }
    hrp::dmatrix mat;
    calcEforce2TauMatrix(mat, enable_ee, enable_joint);
    hrp::dvector joint_torques = mat * total_ee_force;
    for (size_t j = 0; j < m_robot->numJoints(); j++) {
        m_robot->joint(j)->u = 0;
        for (size_t k = 0; k < enable_joint.size(); k++) {
            if (enable_joint[k] == j) {
                m_robot->joint(j)->u = joint_torques(k);
                break;
            }
        }
    }
    if (DEBUGP) {
        std::cerr << "[" << m_profile.instance_name << "] torque ref" << std::endl;
        std::cerr << "[" << m_profile.instance_name << "]    "
                  << joint_torques.transpose() << std::endl;
    }
};

void Stabilizer::startTQStabilizer(void)
{
    std::cerr << "[" << m_profile.instance_name << "] " << "Start TQST DONE"  << std::endl;
};

void Stabilizer::stopTQStabilizer(void)
{
    std::cerr << "[" << m_profile.instance_name << "] " << "Stop TQST DONE"  << std::endl;
};

void Stabilizer::getTQSTParameter(OpenHRP::StabilizerService::tstParam& i_stp)
{
    std::cerr << "[" << m_profile.instance_name << "] getTQSTParameter" << std::endl;
};

void Stabilizer::setTQSTParameter(const OpenHRP::StabilizerService::tstParam& i_stp)
{
    Guard guard(m_mutex);
    std::cerr << "[" << m_profile.instance_name << "] setTQSTParameter" << std::endl;
};

bool Stabilizer::startLogging(void){
    bool result = true;
    std::cout << "[" << m_profile.instance_name << "] " << __func__ << std::endl;

    result =  dlogger.startDumpFile();
    return result;
};

