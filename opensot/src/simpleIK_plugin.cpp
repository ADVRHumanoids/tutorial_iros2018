#include <simpleIK_plugin.h>

REGISTER_XBOT_PLUGIN_(XBotPlugin::simpleIK)

namespace XBotPlugin {

bool simpleIK::init_control_plugin(XBot::Handle::Ptr handle)
{
    /* This function is called outside the real time loop, so we can
     * allocate memory on the heap, print stuff, ...
     * The RT plugin will be executed only if this init function returns true. */


    /* Save robot to a private member. */
    _robot = handle->getRobotInterface();
    
    
    Eigen::VectorXd qhome;
    
    _model = XBot::ModelInterface::getModel(handle->getPathToConfigFile());
    
    _model->getRobotState("home", qhome);
    _model->setJointPosition(qhome);
    _model->update();

    /* Allocate all vectors to be used inside control loop */
    _q.setZero(_robot->model().getJointNum());
    _dq.setZero(_robot->model().getJointNum());

    /* Initialize a logger which saves to the specified file. Remember that
     * the current date/time is always appended to the provided filename,
     * so that logs do not overwrite each other. Preallocate 10000 samples per variable. */

    _logger = XBot::MatLogger::getLogger("/tmp/simpleIK_log");
    _logger->createScalarVariable("time", 1, 10000);
    _logger->createVectorVariable("qref", _robot->getJointNum(), 1, 10000);
    _logger->createVectorVariable("qrefdot", _robot->getJointNum(), 1, 10000);

    // Control loop time
    double dt = 0.001;

    // Initialization of IKProblem class
    _opensot = boost::make_shared<OpenSoT::IKProblem>(_model, _robot, dt);

    return true;
}

void simpleIK::on_start(double time)
{
    /* This function is called on plugin start, i.e. when the start command
     * is sent over the plugin switch port (e.g. 'rosservice call /SimpleHoming_switch true').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* Save the robot starting config to a class member */
    

    
    
    /* Save the plugin starting time to a class member */
    _start_time = time;


    _model->syncFrom(*_robot, XBot::Sync::Position, XBot::Sync::MotorSide);
    
    _model->getJointPosition(_q);
    
    _model->getPose("l_sole", _l_sole_pose);
    _model->getPose("r_sole", _r_sole_pose);

    _model->getCOM(_com_ref);
    
    
    
//     if (_stab == 1)
//     {
     
        Eigen::Vector6d left_wrench;
        _robot->getForceTorque().at("l_leg_ft")->getWrench(left_wrench);
        Eigen::Vector6d right_wrench;
        _robot->getForceTorque().at("r_leg_ft")->getWrench(right_wrench);
        

       
        _zmp_ref = _com_ref.head(2);
        _opensot->_com_stab->setZMP(_zmp_ref);
        _opensot->_com_stab->setSoleRef(_l_sole_pose, _r_sole_pose);
//         _opensot->_com_stab->setWrench(left_wrench, right_wrench);
        _opensot->_com_stab->setReference(_com_ref);
        
        _opensot->update(_q);
        
     Eigen::Affine3d _pose;
    _robot->model().getPose(_opensot->_right_arm->getDistalLink(), _opensot->_right_arm->getBaseLink(), _pose);
    _opensot->_right_arm->setReference(_pose.matrix());

    _robot->model().getPose(_opensot->_left_arm->getDistalLink(), _opensot->_left_arm->getBaseLink(), _pose);
    _opensot->_left_arm->setReference(_pose.matrix());
        
    _opensot->_posture->setReference(_q);
        
//     }
//     else
//     {
//        _opensot->_com->setReference(_com_ref); 
//     }
    
    
//     _com_ref[2] -= 0.1;
    

    
}

void simpleIK::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /SimpleHoming_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
}

bool simpleIK::close()
{
    /* This function is called exactly once, at the end of the experiment.
     * It can be used to do some clean-up, or to save logging data to disk. */

    /* Save logged data to disk */
    _logger->flush();

    return true;
}

void simpleIK::control_loop(double time, double period)
{
        
//     _model->getCOM(_com_ref);
    
//      if (_stab == 1)
//     {
    Eigen::Vector6d left_wrench;
    _robot->getForceTorque().at("l_leg_ft")->getWrench(left_wrench);
    Eigen::Vector6d right_wrench;
    _robot->getForceTorque().at("r_leg_ft")->getWrench(right_wrench);
    
//     left_wrench *= -1;
//     right_wrench *= -1;
     
     
//     _zmp_ref = _com_initial;
//     
//     _opensot->_com_stab->setZMP(_zmp_ref);
    
//     _model->getPose("l_sole", _l_sole_pose);
//     _model->getPose("r_sole", _r_sole_pose);
    
//     _opensot->_com_stab->setSoleRef(_l_sole_pose.translation(), _r_sole_pose.translation());
    
    
    
//     _opensot->_com_stab->setWrench(left_wrench, right_wrench);
    
    
    
//     _opensot->_com_stab->setReference(_com_ref);
    
//     std::cout << com_ref: << com_ref << std::endl;
//     }
//     else
//     {
//         _opensot->_com->setReference(_com_ref);
//     }
    

    

    /* ik update */
    _opensot->update(_q);
    

    /* solve */
    if(!_opensot->solve(_dq))
        XBot::Logger::error("OpenSoT can not solve!");
    
    /* Update q */
    _q += _dq;
    
    /* Model Update*/
    _model->setJointPosition(_q);
    _model->update();


    _logger->add("time", time);
    _logger->add("q", _q);
    _logger->add("dq", _dq);
    _opensot->log(_logger);


    _robot->setPositionReference(_q.segment(6, _q.size()-6));
    
    _robot->move();




}

}
