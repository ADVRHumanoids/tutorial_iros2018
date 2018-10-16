#include <IDProblem.h>
#include <OpenSoT/utils/Affine.h>

using namespace OpenSoT;

IDProblem::IDProblem(XBot::ModelInterface::Ptr model, const double dT):
    _model(model)
{
    std::list<unsigned int> id = {3,4,5};
    std::vector<std::string> links_in_contact;
    links_in_contact.push_back("left_sole_link");
    links_in_contact.push_back("right_sole_link");

    _id = boost::make_shared<OpenSoT::utils::InverseDynamics>(links_in_contact, *_model);

    _left_foot = boost::make_shared<OpenSoT::tasks::acceleration::Cartesian>("left_leg", *_model, links_in_contact[0],
            "world", _id->getJointsAccelerationAffine());

    _right_foot = boost::make_shared<OpenSoT::tasks::acceleration::Cartesian>("right_leg", *_model, links_in_contact[1],
            "world", _id->getJointsAccelerationAffine());

//   -----adding arm tasks----
    _left_arm = boost::make_shared<OpenSoT::tasks::acceleration::Cartesian>("left_arm", *_model, "hand_left_palm_link",
            "world", _id->getJointsAccelerationAffine()); //_model->chain("right_arm").getTipLinkName()
//     _left_arm->setLambda(0.1);
    
    _right_arm = boost::make_shared<OpenSoT::tasks::acceleration::Cartesian>("right_arm", *_model, "hand_right_palm_link",
            "world", _id->getJointsAccelerationAffine());
//     _right_arm->setLambda(0.1);

    _waist = boost::make_shared<OpenSoT::tasks::acceleration::Cartesian>("waist", *_model, "torso_2_link",
        "world", _id->getJointsAccelerationAffine());
//   --------------------------        
    _postural = boost::make_shared<OpenSoT::tasks::acceleration::Postural>(*_model, _id->getJointsAccelerationAffine());

    _com = boost::make_shared<OpenSoT::tasks::acceleration::CoM>(*_model, _id->getJointsAccelerationAffine());

    _dynamics = boost::make_shared<OpenSoT::constraints::acceleration::DynamicFeasibility>("dynamics", *_model,
        _id->getJointsAccelerationAffine(), _id->getContactsWrenchAffine(), links_in_contact);

    OpenSoT::constraints::force::FrictionCone::friction_cones mus;
    for(unsigned int i = 0; i < links_in_contact.size(); ++i)
        mus.push_back(std::pair<std::string, double>(links_in_contact[i], 0.3));
    _friction_cones = boost::make_shared<OpenSoT::constraints::force::FrictionCone>(_id->getContactsWrenchAffine(),*_model,mus);

    /// HERE WE SET SOME BOUNDS
    OpenSoT::AffineHelper I = OpenSoT::AffineHelper::Identity(_id->getSerializer()->getSize());
    Eigen::VectorXd xmax = 20.*Eigen::VectorXd::Ones(_id->getSerializer()->getSize());
    xmax[_model->getJointNum()] = xmax[_model->getJointNum()+6] = 1000;
    xmax[_model->getJointNum()+1] = xmax[_model->getJointNum()+7]= 1000;
    xmax[_model->getJointNum()+2] = xmax[_model->getJointNum()+8]= 1000;

    xmax[_model->getJointNum()+3] = xmax[_model->getJointNum()+9] = 1000;
    xmax[_model->getJointNum()+4] = xmax[_model->getJointNum()+10]= 1000;
    xmax[_model->getJointNum()+5] = xmax[_model->getJointNum()+11]= 1000;

    Eigen::VectorXd xmin = -xmax;
    xmin[_model->getJointNum()+2] = xmin[_model->getJointNum()+8]= 0; //FORCES CAN ONLY PUSH!

    _x_lims = boost::make_shared<OpenSoT::constraints::GenericConstraint>(
                "acc_wrench_lims", I, xmax, xmin, OpenSoT::constraints::GenericConstraint::Type::BOUND);

    _id_problem = ((_left_foot + _right_foot)/
                   (_com + _waist%id)/
                   (_left_arm + _right_arm)/
                   (_postural))<<_x_lims<<_dynamics<<_friction_cones;

    _solver = boost::make_shared<OpenSoT::solvers::iHQP>(_id_problem->getStack(), _id_problem->getBounds(), 1e6);

    _x.setZero(_id->getSerializer()->getSize());
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
    a = _id->computedTorque(_x, tau);
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
