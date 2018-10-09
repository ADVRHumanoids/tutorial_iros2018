#include <simpleID_plugin.h>


REGISTER_XBOT_PLUGIN_(XBotPlugin::simpleID)

namespace XBotPlugin{
    bool simpleID::init_control_plugin(XBot::Handle::Ptr handle)
    {
        /* Save robot to a private member. */
        _robot = handle->getRobotInterface();

        _q.setZero(_robot->model().getJointNum());
        _qdot.setZero(_robot->model().getJointNum());
        _tau.setZero(_robot->model().getJointNum());

        _logger = XBot::MatLogger::getLogger("/tmp/simpleID_log");


        _sh_fb_pos = handle->getSharedMemory()->getSharedObject<Eigen::Vector3d>("/gazebo/floating_base_position");
        _sh_fb_rot = handle->getSharedMemory()->getSharedObject<Eigen::Quaterniond>("/gazebo/floating_base_orientation");
        _sh_fb_vel = handle->getSharedMemory()->getSharedObject<Eigen::Vector6d>("/gazebo/floating_base_velocity");

        _sh_fb_pos.set(Eigen::Vector3d::Zero());
        _sh_fb_rot.set(Eigen::Quaterniond::Identity());
        _sh_fb_vel.set(Eigen::Vector6d::Zero());

        return true;

    }

    void simpleID::control_loop(double time, double period)
    {


        /* Robot feedback*/
        sense();

        /* id update */
        opensot->update();

        /* solve */
        if(!opensot->solve(_tau))
            XBot::Logger::error("OpenSoT can not solve!");

        _logger->add("time", time);
        _logger->add("q", _q);
        _logger->add("dq", _qdot);
        _logger->add("floating_base_pose", _floating_base_pose.matrix());
        _logger->add("floating_base_twist", _sh_fb_vel.get());
        opensot->log(_logger);


        Eigen::VectorXd O(_robot->model().getActuatedJointNum());
        O.setZero(O.size());
        _robot->setStiffness(O);
        _robot->setDamping(O);
        _robot->setEffortReference(_tau.tail(_robot->model().getActuatedJointNum()));
        _robot->move();




    }

    void simpleID::on_start(double time)
    {
        sense();

        double dt = 0.001;
        opensot = boost::make_shared<OpenSoT::IDProblem>(std::shared_ptr<XBot::ModelInterface>(&(_robot->model())), dt);

        _start_time = time;

    }

    void simpleID::sense()
    {
        _floating_base_pose.translation() = _sh_fb_pos.get();
        _floating_base_pose.linear() = _sh_fb_rot.get().toRotationMatrix();

        _robot->sense(true);
        _robot->model().setFloatingBaseState(_floating_base_pose, _sh_fb_vel.get());

        _robot->model().getJointPosition(_q);
        _robot->model().getJointVelocity(_qdot);
    }

    void simpleID::on_stop(double time)
    {
        /* This function is called on plugin stop, i.e. when the stop command
         * is sent over the plugin switch port (e.g. 'rosservice call /SimpleHoming_switch false').
         * Since this function is called within the real-time loop, you should not perform
         * operations that are not rt-safe. */
    }

    bool simpleID::close()
    {
        /* This function is called exactly once, at the end of the experiment.
         * It can be used to do some clean-up, or to save logging data to disk. */

        /* Save logged data to disk */
        _logger->flush();

        return true;
    }

}


