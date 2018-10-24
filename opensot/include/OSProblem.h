#ifndef _OSPROBLEM_H_
#define _OSPROBLEM_H_

#include <OpenSoT/tasks/torque/CartesianImpedanceCtrl.h>
#include <OpenSoT/tasks/torque/JointImpedanceCtrl.h>
#include <OpenSoT/constraints/torque/TorqueLimits.h>
#include <OpenSoT/utils/ForceOptimization.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/iHQP.h>

namespace OpenSoT{

class OSProblem{
public:
    typedef boost::shared_ptr<OSProblem> Ptr;

    OSProblem(XBot::ModelInterface::Ptr model, const double dT);
    ~OSProblem();

    bool solve(Eigen::VectorXd& x, const bool robot_on_ground);

    void update(const bool robot_on_ground);

    void log(XBot::MatLogger::Ptr& logger);

    OpenSoT::tasks::torque::CartesianImpedanceCtrl::Ptr _left_foot, _right_foot;
    OpenSoT::tasks::torque::CartesianImpedanceCtrl::Ptr _left_arm, _right_arm;
    OpenSoT::tasks::torque::CartesianImpedanceCtrl::Ptr _base_link;
    OpenSoT::tasks::torque::JointImpedanceCtrl::Ptr _postural;

    OpenSoT::constraints::torque::TorqueLimits::Ptr _torque_lims;

    OpenSoT::utils::ForceOptimization::Ptr _forza_giusta;

    Eigen::MatrixXd LFoot_ref, RFoot_ref;

private:
    AutoStack::Ptr _os_problem;

    solvers::iHQP::Ptr _solver;

    XBot::ModelInterface::Ptr _model;

    Eigen::VectorXd _tau, _tau_bar, _q, _h;
    std::vector<Eigen::Vector6d> _fc;

};

}

#endif
