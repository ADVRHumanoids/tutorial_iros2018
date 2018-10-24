#include <OSProblem.h>

using namespace OpenSoT;

OSProblem::OSProblem(XBot::ModelInterface::Ptr model, const double dT):
    _model(model)
{
    std::vector<std::string> links_in_contact;
    links_in_contact.push_back("left_sole_link");
    links_in_contact.push_back("right_sole_link");

    _forza_giusta = boost::make_shared<OpenSoT::utils::ForceOptimization>(_model, links_in_contact);


    _model->getJointPosition(_q);
    _model->computeNonlinearTerm(_h);

    std::list<unsigned int> id = {0,1,2};
    _left_foot = boost::make_shared<OpenSoT::tasks::torque::CartesianImpedanceCtrl>
            ("l_foot", _q, *_model, links_in_contact[0], "world");
    _left_foot->getActualPose(LFoot_ref);
    _right_foot = boost::make_shared<OpenSoT::tasks::torque::CartesianImpedanceCtrl>
            ("r_foot", _q, *_model, links_in_contact[1], "world");
    _right_foot->getActualPose(RFoot_ref);

    _base_link = boost::make_shared<OpenSoT::tasks::torque::CartesianImpedanceCtrl>
            ("base_link", _q, *_model, "base_link", "world");
    Eigen::MatrixXd K(6,6); K.setIdentity(6,6);


    K.diagonal()<<1,1,1,1,1,1;
    _base_link->setStiffnessDamping(1000.*K, 2.*std::sqrt(7000.)*K);
    K.diagonal()<<1,1,1,1,1,1;
    _right_foot->setStiffnessDamping(0.*K, 1.*K);
    _left_foot->setStiffnessDamping(0.*K, 1.*K);

    Eigen::MatrixXd Kj(_model->getJointNum(),_model->getJointNum()),Dj(_model->getJointNum(),_model->getJointNum());
    _postural = boost::make_shared<OpenSoT::tasks::torque::JointImpedanceCtrl>(_q, *_model);
    _postural->getStiffnessDamping(Kj, Dj);
    //Kj.block(0,0,6,6) = Eigen::MatrixXd::Zero(6,6); Dj.block(0,0,6,6) = Eigen::MatrixXd::Zero(6,6);
    _postural->setStiffnessDamping(10.*Kj, Dj);



    Eigen::VectorXd torque_lims;
    _model->getEffortLimits(torque_lims);
    //torque_lims.head(6) = Eigen::Vector6d::Zero();
    _torque_lims = boost::make_shared<OpenSoT::constraints::torque::TorqueLimits>(1000.*torque_lims, -1000.*torque_lims);

    _os_problem = ((_left_foot + _right_foot)/
                   (_base_link)/
                   (_postural))<<_torque_lims;


    _solver = boost::make_shared<OpenSoT::solvers::iHQP>(_os_problem->getStack(), _os_problem->getBounds(), 0.);

    _tau.setZero(_model->getJointNum());
    _tau_bar.setZero(_model->getJointNum());
    Eigen::Vector6d fc; fc.setZero();
    _fc.push_back(fc);_fc.push_back(fc);
}

OSProblem::~OSProblem()
{

}

void OSProblem::update(const bool robot_on_ground)
{
    _model->getJointPosition(_q);

    Eigen::VectorXd tau_lims;
    _model->getEffortLimits(tau_lims);

    if(!robot_on_ground){
        //tau_lims.head(6) = Eigen::Vector6d::Zero();
        _solver->setActiveStack(0, false);}
    else
        _solver->setActiveStack(0, true);

    _torque_lims->setTorqueLimits(1000.*tau_lims, -1000.*tau_lims);

    _os_problem->update(_q);
}

bool OSProblem::solve(Eigen::VectorXd& tau, const bool robot_on_ground)
{

    bool a = _solver->solve(_tau_bar);
    if(!a)
        return false;

    _model->computeNonlinearTerm(_h);
    if(robot_on_ground)
    {
        _tau_bar.noalias() += _h;
        a = _forza_giusta->compute(_tau_bar, _fc, _tau);
    }
    else
    {
        _h.head(6) = Eigen::Vector6d::Zero();
        _tau_bar.noalias() += _h;
        _tau = _tau_bar;
    }

    tau = _tau;
    return a;
}

void OSProblem::log(XBot::MatLogger::Ptr& logger)
{
    _forza_giusta->log(logger);
    _os_problem->log(logger);
    _solver->log(logger);
    logger->add("tau_bar", _tau_bar);
}
