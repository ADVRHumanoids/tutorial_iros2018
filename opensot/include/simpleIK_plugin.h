#ifndef _SIMPLEIK_PLUGIN_
#define _SIMPLEIK_PLUGIN_

#include <XCM/XBotControlPlugin.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>

#include <IKProblem.h>

namespace XBotPlugin {

/**
 * @brief simpleIK XBot RT Plugin
 *
 **/
class simpleIK : public XBot::XBotControlPlugin
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

    Eigen::VectorXd _q, _dq;

    XBot::MatLogger::Ptr _logger;
    
    XBot::ModelInterface::Ptr _model;
    
    OpenSoT::IKProblem::Ptr _opensot;

    Eigen::Affine3d _pose;
    
    Eigen::Affine3d _l_sole_pose;
    Eigen::Affine3d _r_sole_pose;
    
    
    Eigen::Vector3d _com_ref;
    Eigen::Vector3d _zmp_ref;
    
    bool _first_sync_done;
     
};

}

#endif


