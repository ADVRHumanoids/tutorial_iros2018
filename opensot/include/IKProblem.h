#ifndef _IKPROBLEM_H_
#define _IKPROBLEM_H_

#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/iHQP.h>

namespace OpenSoT{

class IKProblem{

public:
    typedef boost::shared_ptr<IKProblem> Ptr;

    IKProblem(XBot::ModelInterface::Ptr model, const double dT);
    ~IKProblem();

    bool solve(Eigen::VectorXd& x);
    void update(Eigen::VectorXd& x, bool use_inertia_matrix = false);
    void log(XBot::MatLogger::Ptr& logger);

    tasks::velocity::Cartesian::Ptr _left_arm, _right_arm;
    tasks::velocity::Postural::Ptr _posture;

private:
    constraints::velocity::JointLimits::Ptr _joint_limits;
    constraints::velocity::VelocityLimits::Ptr _vel_limits;

    AutoStack::Ptr _ik_problem;

    solvers::iHQP::Ptr _solver;

    Eigen::MatrixXd M;

    XBot::ModelInterface::Ptr _model;

};

}

#endif
