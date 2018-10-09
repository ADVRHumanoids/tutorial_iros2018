#ifndef _SIMPLEID_PLUGIN_
#define _SIMPLEID_PLUGIN_

#include <XCM/XBotControlPlugin.h>
#include <IDProblem.h>

namespace XBotPlugin {
/**
 * @brief simpleID XBot RT Plugin
 *
 **/
class simpleID : public XBot::XBotControlPlugin
{

public:

    /**
     * @brief init_control_plugin called when the plugin is loaded
     * @param handle configuration getters
     * @return true if suceed
     */
    virtual bool init_control_plugin(XBot::Handle::Ptr handle);

    /**
     * @brief close is called when the plugin is unloaded with the rosservice by the user
     * @return
     */
    virtual bool close();

    /**
     * @brief on_start is called when the plugin is started with the rosservice by the user
     * @param time the moment in which is started
     */
    virtual void on_start(double time);

    /**
     * @brief on_stop is called when the plugin is stopped with the rosservice by the user
     * @param time the moment in which is stopped
     */
    virtual void on_stop(double time);

    OpenSoT::IDProblem::Ptr opensot;

    void sense();

protected:
    /**
     * @brief control_loop is the actual loop, RT-safe code needs to be implemented here
     * @param time at each control loop
     * @param period of the control loop
     */
    virtual void control_loop(double time, double period);


private:

    XBot::RobotInterface::Ptr _robot;

    double _start_time;

    Eigen::VectorXd _q, _qdot; //feedback
    Eigen::VectorXd _tau; //decision variables and control variables

    XBot::MatLogger::Ptr _logger;

    /// IN THIS EXAMPLE WE RETRIEVE THE POSE OF THE FLOATING BASE FROM GAZEBO
    XBot::SharedObject<Eigen::Vector3d> _sh_fb_pos;
    XBot::SharedObject<Eigen::Quaterniond> _sh_fb_rot;
    XBot::SharedObject<Eigen::Vector6d> _sh_fb_vel;
    Eigen::Affine3d _floating_base_pose;

};

}


#endif
