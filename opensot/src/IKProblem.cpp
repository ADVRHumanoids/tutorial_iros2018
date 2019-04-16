#include <IKProblem.h>



OpenSoT::IKProblem::IKProblem(XBot::ModelInterface::Ptr model, const double dT):
    _model(model)
{
    Eigen::VectorXd q;
    // We gate the actual joint position from the model
    _model->getJointPosition(q);

    // Initialization of Cartesian tasks including task ids, joint position, model,
    // end-effector name, base_link from where the tasks are controlled
    _left_arm = boost::make_shared<tasks::velocity::Cartesian>("larm", q, *model,
                                                          _model->chain("left_arm").getTipLinkName(),
                                                          _model->chain("torso").getBaseLinkName());
    // Proportional gain for the task error
    _left_arm->setLambda(0.1);
    
    _right_arm = boost::make_shared<tasks::velocity::Cartesian>("rarm", q, *model,
                                                          _model->chain("right_arm").getTipLinkName(),
                                                          _model->chain("torso").getBaseLinkName());
    _right_arm->setLambda(0.1);
    
    // Ids to select rows of the right arm, in particular we are considering just the position part
    std::list<uint> pos_idx = {0, 1, 2};
    auto right_arm_pos = _right_arm % pos_idx;
        
    // CoM task initialization
    _com = boost::make_shared<tasks::velocity::CoM>(q, *model);
    
    // CoMStabilizer task initialization
    
    
    Eigen::Affine3d ankle;
    _model->getPose("l_ankle", "l_sole", ankle);
    
    Eigen::Vector2d foot_size;
    foot_size<< 0.21,0.11;
    
    double Fzmin = 10.;
    double dt = 0.001;
    Eigen::Vector3d K(0.09,0.09,0.);
    Eigen::Vector3d C(-0.005,-0.005,0.);
    
    _com_stab = boost::make_shared<tasks::velocity::CoMStabilizer>(q, *model,
                                                                   dt,
                                                                   _model->getMass(), 
                                                                   fabs(ankle(2,3)),
                                                                   foot_size,
                                                                   Fzmin,
                                                                   K,C,
                                                                   Eigen::Vector3d(DEFAULT_MaxLimsx, DEFAULT_MaxLimsy, DEFAULT_MaxLimsz),
                                                                   Eigen::Vector3d(DEFAULT_MinLimsx, DEFAULT_MinLimsy, DEFAULT_MinLimsz)
                                                                   );

                                
    _posture = boost::make_shared<tasks::velocity::Postural>(q);

    // Get Inertia matrix from the model computed at q
    _model->getInertiaMatrix(M);

    // Proportional gain for the task error
    _posture->setLambda(0.01);

    // Initialization of joint limits, joint limits are taken from the model
    Eigen::VectorXd qmax, qmin;
    model->getJointLimits(qmin, qmax);
    _joint_limits = boost::make_shared<constraints::velocity::JointLimits>(q, qmax, qmin);

    // Initialization of joint velocity limits
    _vel_limits = boost::make_shared<constraints::velocity::VelocityLimits>(M_PI, dT, q.size());

    /*
     *  The Math of Tasks
     */
    _ik_problem = (  _com /   //_com_stab   //  (_left_arm + _left_arm)
                    _posture) << _joint_limits << _vel_limits;

    // Initialization of the solver, qpOASES is the default back end, 1e8 is the value of the regularization term
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
