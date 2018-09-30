#ifndef _IKPROBLEM_H_
#define _IKPROBLEM_H_

#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/iHQP.h>

namespace OpenSoT{

/**
 * @brief The IKProblem class wraps the tasks, constraints and the solver used to solve the IK
 */
class IKProblem{

public:
    typedef boost::shared_ptr<IKProblem> Ptr;

    /**
     * @brief IKProblem constructor
     * @param model pointer to external model
     * @param dT control loop
     */
    IKProblem(XBot::ModelInterface::Ptr model, const double dT);
    ~IKProblem();

    /**
     * @brief solve call this after update()
     * @param x solution form solver (dq)
     * @return true if solved
     */
    bool solve(Eigen::VectorXd& x);

    /**
     * @brief update call after the model.update() to update the autostack
     * @param x input state q
     * @param use_inertia_matrix if true the postural task is weighted with the inertia matrix
     */
    void update(Eigen::VectorXd& x, bool use_inertia_matrix = false);

    /**
     * @brief log to log solver and autostaack status
     * @param logger a pointer to a MatLogger
     */
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
