#include <IKProblem.h>



OpenSoT::IKProblem::IKProblem(XBot::ModelInterface::Ptr model, XBot::RobotInterface::Ptr robot, const double dT):
    _model(model),
    _robot(robot)
{
    Eigen::VectorXd q;
    // We gate the actual joint position from the model
    _model->getJointPosition(q);

    // Initialization of Cartesian tasks including task ids, joint position, model,
    // end-effector name, base_link from where the tasks are controlled
    _left_arm = boost::make_shared<tasks::velocity::Cartesian>("larm", q, *model,
                                                          _model->chain("left_arm").getTipLinkName(),
                                                          "world");
    // Proportional gain for the task error
    _left_arm->setLambda(0.1);
    
    _right_arm = boost::make_shared<tasks::velocity::Cartesian>("rarm", q, *model,
                                                          _model->chain("right_arm").getTipLinkName(),
                                                          "world");
    _right_arm->setLambda(0.1);
    
// _model->chain("left_leg").getBaseLinkName()
    _left_foot = boost::make_shared<tasks::velocity::Cartesian>("lfoot", q, *model,
                                                          "l_sole",
                                                          "world");
    
    
    
    _right_foot = boost::make_shared<tasks::velocity::Cartesian>("rfoot", q, *model,
                                                          "r_sole",
                                                          "world");
    
//     std::cout << _model->chain("right_leg").getTipLinkName() << std::endl;
//     std::cout << _model->chain("left_leg").getTipLinkName() << std::endl;
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
    Eigen::Vector3d K;
    Eigen::Vector3d C;
    
    K << 0.1,0.1,0.;
//     C << -0.005,-0.005,0.;
//      K << 0.005,0.005,0.;
     C << 0.001, 0.001,0.;
//     K << 0.5,0.5,0.;
     
        Eigen::Affine3d _l_sole_pose, _r_sole_pose;
        _model->getPose("l_sole", _l_sole_pose);
        _model->getPose("r_sole", _r_sole_pose);
     

                                
    _com_stab = boost::make_shared<tasks::velocity::CoMStabilizer>( q, *model,
                                                                    _l_sole_pose, _r_sole_pose,
                                                                    _robot->getForceTorque().at("l_leg_ft"), _robot->getForceTorque().at("r_leg_ft"),
                                                                    dT,
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
    _ik_problem = ( (_right_foot + _left_foot) /
                    _com_stab / 
                    (_left_arm + _right_arm)/ //_com   // _com_stab
                    _posture) << _joint_limits << _vel_limits;
                    
    _ik_problem->update(q);

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
