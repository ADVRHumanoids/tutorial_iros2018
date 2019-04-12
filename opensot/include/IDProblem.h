#ifndef _IDPROBLEM_H_
#define _IDPROBLEM_H_

#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/tasks/acceleration/Cartesian.h>
#include <OpenSoT/tasks/acceleration/CoM.h>
#include <OpenSoT/constraints/acceleration/DynamicFeasibility.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/utils/InverseDynamics.h>
#include <OpenSoT/constraints/force/FrictionCone.h>

namespace OpenSoT{

/**
 * @brief The IDProblem class wraps the tasks, constraints and the solver used to solve the ID
 */
class IDProblem{

public:
    typedef boost::shared_ptr<IDProblem> Ptr;

    /**
     * @brief IDProblem constructor
     * @param model pointer to external model
     * @param dT control loop
     */
    IDProblem(XBot::ModelInterface::Ptr model, const double dT);
    ~IDProblem();

    /**
     * @brief solve call this after update()
     * @param x solution form solver (tau)
     * @return true if solved
     */
    bool solve(Eigen::VectorXd& x);

    /**
     * @brief update call after the model.update() to update the autostack
     * @param x input state q
     */
    void update();

    /**
     * @brief log to log solver and autostaack status
     * @param logger a pointer to a MatLogger
     */
    void log(XBot::MatLogger::Ptr& logger);

    /**
     * @brief _left_arm, _right_arm two Cartesian tasks
     */
    tasks::acceleration::Cartesian::Ptr _left_arm, _right_arm;
    tasks::acceleration::Cartesian::Ptr _left_foot, _right_foot;
    tasks::acceleration::Cartesian::Ptr _waist;
    tasks::acceleration::CoM::Ptr _com;

    /**
     * @brief _posture a postural task
     */
    tasks::acceleration::Postural::Ptr _postural;

private:
    /**
     * @brief _dynamics constraint relates the floating base with the contact forces
     */
    constraints::acceleration::DynamicFeasibility::Ptr _dynamics;

    /**
     * @brief _friction_cones constraints
     */
    constraints::force::FrictionCones::Ptr _friction_cones;

    /**
     * @brief _x_lims some bounds
     */
    constraints::GenericConstraint::Ptr _x_lims;

    /**
     * @brief _id_problem the final ID problem
     */
    AutoStack::Ptr _id_problem;

    /**
     * @brief _solver iHQP solver
     */
    solvers::iHQP::Ptr _solver;

    /**
     * @brief _model
     */
    XBot::ModelInterface::Ptr _model;

    /**
     * @brief _id inverse dynamics computation & variable helper
     */
    OpenSoT::utils::InverseDynamics::Ptr _id;

    /**
     * @brief _x decision variables
     */
    Eigen::VectorXd _x;

    Eigen::VectorXd _qddot;

};

}

#endif
