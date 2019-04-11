#include <IDProblem.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/tasks/MinimizeVariable.h>

using namespace OpenSoT;

IDProblem::IDProblem(XBot::ModelInterface::Ptr model, const double dT):
    _model(model)
{
    //
    // First we define which links are in contact with the environment
    //
    std::vector<std::string> links_in_contact;
    links_in_contact.push_back("lf_foot");
    links_in_contact.push_back("rf_foot");
    links_in_contact.push_back("lh_foot");
    links_in_contact.push_back("rh_foot");

    //
    //  This utility internally creates the right variables which later we will use to
    //  create all the tasks and constraints
    //
    _id = boost::make_shared<OpenSoT::utils::InverseDynamics>(links_in_contact, *_model);

    //
    // Here we create all the tasks: the feet has to be created wrt the world frame
    //
    _lh_foot = boost::make_shared<OpenSoT::tasks::acceleration::Cartesian>(links_in_contact[0], *_model, links_in_contact[0],
            "world", _id->getJointsAccelerationAffine());
    _lh_foot->setLambda(1.);

    _rh_foot = boost::make_shared<OpenSoT::tasks::acceleration::Cartesian>(links_in_contact[1], *_model, links_in_contact[1],
            "world", _id->getJointsAccelerationAffine());
    _rh_foot->setLambda(1.);

    _lf_foot = boost::make_shared<OpenSoT::tasks::acceleration::Cartesian>(links_in_contact[2], *_model, links_in_contact[2],
            "world", _id->getJointsAccelerationAffine());
    _lf_foot->setLambda(1.);

    _rf_foot = boost::make_shared<OpenSoT::tasks::acceleration::Cartesian>(links_in_contact[3], *_model, links_in_contact[3],
            "world", _id->getJointsAccelerationAffine());
    _rf_foot->setLambda(1.);

//   -----adding arm tasks----
    _arm = boost::make_shared<OpenSoT::tasks::acceleration::Cartesian>("arm", *_model, "_teleop_link5",
            "world", _id->getJointsAccelerationAffine()); //_model->chain("right_arm").getTipLinkName()
    _arm->setLambda(10.);

    _waist = boost::make_shared<OpenSoT::tasks::acceleration::Cartesian>("waist", *_model, "base_link",
        "world", _id->getJointsAccelerationAffine());
    _waist->setLambda(10.);
//   --------------------------        
    _postural = boost::make_shared<OpenSoT::tasks::acceleration::Postural>(*_model, _id->getJointsAccelerationAffine());
    _postural->setLambda(10.);

    _com = boost::make_shared<OpenSoT::tasks::acceleration::CoM>(*_model, _id->getJointsAccelerationAffine());
    _com->setLambda(10.);

    //
    // Here we create the constraints & bounds
    //
    _dynamics = boost::make_shared<OpenSoT::constraints::acceleration::DynamicFeasibility>("dynamics", *_model,
        _id->getJointsAccelerationAffine(), _id->getContactsWrenchAffine(), links_in_contact);

    OpenSoT::constraints::force::FrictionCones::friction_cones mus;
    Eigen::Matrix3d R; R.setIdentity();
    for(unsigned int i = 0; i < links_in_contact.size(); ++i)
        mus.push_back(std::pair<Eigen::Matrix3d,double> (R,0.5));
    _friction_cones = boost::make_shared<OpenSoT::constraints::force::FrictionCones>(
                links_in_contact, _id->getContactsWrenchAffine(),*_model,mus);

    /// HERE WE SET SOME BOUNDS
    OpenSoT::AffineHelper I = OpenSoT::AffineHelper::Identity(_id->getSerializer()->getSize());
    Eigen::VectorXd xmax = 1000.*Eigen::VectorXd::Ones(_id->getSerializer()->getSize());

    xmax[_model->getJointNum()] = xmax[_model->getJointNum()+6] =
            xmax[_model->getJointNum()+12] = xmax[_model->getJointNum()+18] = 2000;
    xmax[_model->getJointNum()+1] = xmax[_model->getJointNum()+7] =
            xmax[_model->getJointNum()+13] = xmax[_model->getJointNum()+19] = 2000;
    xmax[_model->getJointNum()+2] = xmax[_model->getJointNum()+8] =
            xmax[_model->getJointNum()+14] = xmax[_model->getJointNum()+20] = 2000;

    xmax[_model->getJointNum()+3] = xmax[_model->getJointNum()+9] =
            xmax[_model->getJointNum()+15] = xmax[_model->getJointNum()+21] = 0;
    xmax[_model->getJointNum()+4] = xmax[_model->getJointNum()+10] =
            xmax[_model->getJointNum()+16] = xmax[_model->getJointNum()+22] = 0;
    xmax[_model->getJointNum()+5] = xmax[_model->getJointNum()+11] =
            xmax[_model->getJointNum()+17] = xmax[_model->getJointNum()+23] = 0;

    Eigen::VectorXd xmin = -xmax;
    xmin[_model->getJointNum()+2] = xmin[_model->getJointNum()+8] =
            xmin[_model->getJointNum()+14] = xmin[_model->getJointNum()+20] = 0; //FORCES CAN ONLY PUSH!

    _x_lims = boost::make_shared<OpenSoT::constraints::GenericConstraint>(
                "acc_wrench_lims", I, xmax, xmin, OpenSoT::constraints::GenericConstraint::Type::BOUND);

    // Notice that we just control the orientation of the waist
    std::list<unsigned int> idw = {3,4,5};
    std::list<unsigned int> idc = {0,1,2};
    
    _id_problem = ((_lh_foot%idc + _rh_foot%idc + _lf_foot%idc + _rf_foot%idc)/
                   (_com + _waist%idw)/
                   _arm/
                   _postural)<<_x_lims<<_dynamics<<_friction_cones;

    _solver = boost::make_shared<OpenSoT::solvers::iHQP>(_id_problem->getStack(), _id_problem->getBounds(), 1e6);

    _x.setZero(_id->getSerializer()->getSize());

    _qddot.setZero(_model->getJointNum());
}

void IDProblem::update()
{
    _id_problem->update(Eigen::VectorXd(1));
}

bool IDProblem::solve(Eigen::VectorXd& tau)
{
    bool a = _solver->solve(_x);

    if(!a)
        return false;
    a = _id->computedTorque(_x, tau, _qddot);
    return a;
}

void IDProblem::log(XBot::MatLogger::Ptr& logger)
{
    _id->log(logger);
    _id_problem->log(logger);
    _solver->log(logger);
}

IDProblem::~IDProblem()
{

}
