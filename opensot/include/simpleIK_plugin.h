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
     * @brief init_control_plugin
     * @param handle
     * @return
     */
    virtual bool init_control_plugin(XBot::Handle::Ptr handle);

    virtual bool close();

    virtual void on_start(double time);

    virtual void on_stop(double time);

protected:

    virtual void control_loop(double time, double period);

private:

    XBot::RobotInterface::Ptr _robot;

    double _start_time;

    Eigen::VectorXd _q, _dq;

    XBot::MatLogger::Ptr _logger;

    OpenSoT::IKProblem::Ptr _opensot;

    Eigen::Affine3d _pose;

};

}

#endif


