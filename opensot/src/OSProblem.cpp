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

    _left_foot = boost::make_shared<OpenSoT::tasks::torque::CartesianImpedanceCtrl>
            ("l_foot", _q, *_model, links_in_contact[0], "world");
    _right_foot = boost::make_shared<OpenSoT::tasks::torque::CartesianImpedanceCtrl>
            ("r_foot", _q, *_model, links_in_contact[1], "world");
    _base_link = boost::make_shared<OpenSoT::tasks::torque::CartesianImpedanceCtrl>
            ("base_link", _q, *_model, "base_link", "world");

    _postural = boost::make_shared<OpenSoT::tasks::torque::JointImpedanceCtrl>(_q, *_model);


    Eigen::VectorXd torque_lims;
    _model->getEffortLimits(torque_lims);

    _torque_lims = boost::make_shared<OpenSoT::constraints::torque::TorqueLimits>(torque_lims, -torque_lims);

    _os_problem = ((_left_foot + _right_foot)/
                   (_base_link)/
                   (_postural))<<_torque_lims;

    _solver = boost::make_shared<OpenSoT::solvers::iHQP>(_os_problem->getStack(), _os_problem->getBounds(), 1.);

    _tau.setZero(_model->getJointNum());
    _tau_bar.setZero(_model->getJointNum());
    Eigen::Vector6d fc; fc.setZero();
    _fc.push_back(fc);_fc.push_back(fc);
}

OSProblem::~OSProblem()
{

}

void OSProblem::update()
{
    _model->getJointPosition(_q);
    _os_problem->update(_q);
}

bool OSProblem::solve(Eigen::VectorXd& tau)
{
    bool a = _solver->solve(_tau_bar);
    if(!a)
        return false;
    _model->computeNonlinearTerm(_h);
    _tau_bar.noalias() += _h;
    a = _forza_giusta->compute(_tau_bar, _fc, _tau);
    tau = _tau;
    return a;
}

void OSProblem::log(XBot::MatLogger::Ptr& logger)
{
    _forza_giusta->log(logger);
    _os_problem->log(logger);
    _solver->log(logger);
}
