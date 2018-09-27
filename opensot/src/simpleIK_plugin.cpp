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

    double dt = 0.001;
    _opensot = boost::make_shared<OpenSoT::IKProblem>(std::shared_ptr<XBot::ModelInterface>(&(_robot->model())), dt);

    return true;
}

void simpleIK::on_start(double time)
{
    /* This function is called on plugin start, i.e. when the start command
     * is sent over the plugin switch port (e.g. 'rosservice call /SimpleHoming_switch true').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* Save the robot starting config to a class member */
    _robot->sense(true);
    _robot->model().getJointPosition(_q);

    /* Save the plugin starting time to a class member */
    _start_time = time;

    //Eigen::Affine3d _pose;
    _robot->model().getPose(_opensot->_arm->getDistalLink(), _opensot->_arm->getBaseLink(), _pose);

    _opensot->_arm->setReference(_pose.matrix());

    _opensot->_posture->setReference(_q);

    _opensot->update(_q);

    _pose.translation()[1] += 0.1;
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
    /* This function is called on every control loop from when the plugin is start until
     * it is stopped.
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* The following code checks if any command was received from the plugin standard port
     * (e.g. from ROS you can send commands with
     *         rosservice call /SimpleHoming_cmd "cmd: 'MY_COMMAND_1'"
     * If any command was received, the code inside the if statement is then executed. */

    if(!current_command.str().empty()){

        if(current_command.str() == "MY_COMMAND_1"){
            XBot::Logger::info(Logger::Severity::HIGH, "My command 1 executed! \n");
        }

        if(current_command.str() == "MY_COMMAND_2"){
            XBot::Logger::info(Logger::Severity::HIGH, "My command 2 executed! \n");
        }

    }

    if(time-_start_time > 2.){
        _opensot->_arm->setReference(_pose.matrix());
        _opensot->_arm->setLambda(0.001);
    }



    /* Model Update*/
    _robot->model().setJointPosition(_q);
    _robot->model().update();

    /* ik update */
    _opensot->update(_q);

    /* solve */
    if(!_opensot->solve(_dq))
        XBot::Logger::error("OpenSoT can not solve!");
    _q += _dq;


    _logger->add("time", time);
    _logger->add("q", _q);
    _logger->add("dq", _dq);
    _opensot->log(_logger);





    _robot->setPositionReference(_q.segment(6, _q.size()-6));
    _robot->move();




}

}
