#include <IKProblem.h>

OpenSoT::IKProblem::IKProblem(XBot::ModelInterface &model, const double dT)
{
    Eigen::VectorXd q;
    model.getJointPosition(q);

    _arm = boost::make_shared<tasks::velocity::Cartesian>("arm", q, model, "hand_left_palm_link",
                                                         model.chain("torso").getBaseLinkName());
    _posture = boost::make_shared<tasks::velocity::Postural>(q);

    Eigen::VectorXd qmax, qmin;
    model.getJointLimits(qmin, qmax);
    _joint_limits = boost::make_shared<constraints::velocity::JointLimits>(q, qmax, qmin);

    _vel_limits = boost::make_shared<constraints::velocity::VelocityLimits>(M_PI, dT, q.size());

    _ik_problem = (_arm/_posture)<<_joint_limits<<_vel_limits;

    _solver = boost::make_shared<solvers::iHQP>(_ik_problem->getStack(), _ik_problem->getBounds(), 1e8);
}

OpenSoT::IKProblem::~IKProblem()
{

}

bool OpenSoT::IKProblem::solve(Eigen::VectorXd &x)
{
    bool a = _solver->solve(x);
    if(!a)
        x.setZero(x.size());
    return a;
}

void OpenSoT::IKProblem::update(Eigen::VectorXd& x)
{
    _ik_problem->update(x);
}

void OpenSoT::IKProblem::log(XBot::MatLogger::Ptr &logger)
{
    _ik_problem->log(logger);
    _solver->log(logger);
}
