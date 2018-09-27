#include <IKProblem.h>

OpenSoT::IKProblem::IKProblem(XBot::ModelInterface::Ptr model, const double dT)
{
    Eigen::VectorXd q;

    _model = model;

    _model->getJointPosition(q);

    _model->getInertiaMatrix(M);

    _arm = boost::make_shared<tasks::velocity::Cartesian>("arm", q, *model,
                                                          _model->chain("left_arm").getTipLinkName(),
                                                          _model->chain("torso").getBaseLinkName());
    _arm->setLambda(0.01);

    _posture = boost::make_shared<tasks::velocity::Postural>(q);
    _posture->setWeight(M);
    _posture->setLambda(0.01);

    Eigen::VectorXd qmax, qmin;
    model->getJointLimits(qmin, qmax);
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

void OpenSoT::IKProblem::update(Eigen::VectorXd& x, bool use_inertia_matrix)
{
    if(use_inertia_matrix)
        _model->getInertiaMatrix(M);
    else
        M.setIdentity(M.rows(), M.cols());
    _posture->setWeight(M);
    _ik_problem->update(x);
}

void OpenSoT::IKProblem::log(XBot::MatLogger::Ptr &logger)
{
    _ik_problem->log(logger);
    _solver->log(logger);
}
