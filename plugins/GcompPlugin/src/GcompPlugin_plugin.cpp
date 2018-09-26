/*
 * Copyright (C) 2017 IIT-ADVR
 * Author:
 * email:
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <GcompPlugin_plugin.h>

/* Specify that the class XBotPlugin::GcompPlugin is a XBot RT plugin with name "GcompPlugin" */
REGISTER_XBOT_PLUGIN_(XBotPlugin::GcompPlugin)

namespace XBotPlugin {

bool GcompPlugin::init_control_plugin(XBot::Handle::Ptr handle)
{
    
    /* This function is called outside the real time loop, so we can
     * allocate memory on the heap, print stuff, ...
     * The RT plugin will be executed only if this init function returns true. */
    
    
    
    
    /* Get robot and imu objects */
    _robot = handle->getRobotInterface();
    if(!_robot->getImu().empty())
    {
        _imu = _robot->getImu().begin()->second; 
    }
    
    /* Define contact links from SRDF legs */
    for(int i = 0; i < _robot->legs(); i++)
    {
        std::cout << "Adding contact link " << _robot->leg(i).getTipLinkName() << std::endl;
        _contact_links.push_back(_robot->leg(i).getTipLinkName());
    }
    
    /* Get model and initialize it to homing */
    _model = XBot::ModelInterface::getModel(handle->getPathToConfigFile());
    Eigen::VectorXd qhome;
    _model->getRobotState("home", qhome);
    _model->setJointPosition(qhome);
    _model->update();
    
    /* Allocate vectors */
    _model->computeGravityCompensation(_gcomp);
    _tau_d = _gcomp;
    _robot->getStiffness(_k0);
    _k = _k0;
    
    /* Construct object for force optimization */
    _force_opt = boost::make_shared<ForceOptimization>(_model, _contact_links, _robot->legs() == 2);
    
    /* Register object inside shared memory */
    _shobj_stiffness = handle->getSharedMemory()->getSharedObject<double>("/desired_stiffness_gain");
    _shobj_stiffness.set(1.0);
    
    return true;


}

void GcompPlugin::on_start(double time)
{
    /* This function is called on plugin start, i.e. when the start command
     * is sent over the plugin switch port (e.g. 'rosservice call /GcompPlugin_switch true').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    _robot->getStiffness(_k0);
    _k = _k0;
}

void GcompPlugin::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /GcompPlugin_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
}


void GcompPlugin::control_loop(double time, double period)
{
    /* This function is called on every control loop from when the plugin is start until
     * it is stopped.
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* Take model state from robot state */
    _model->syncFrom(*_robot, XBot::Sync::Position, XBot::Sync::MotorSide);
    
    /* Set floating base state from IMU (orientation and angular velocity) */
    if(_imu)
    {
        _model->setFloatingBaseState(_imu);
        _model->update();
    }
    
    /* Compute gcomp (assumes all joints are actuated) */
    _model->computeGravityCompensation(_gcomp);
    
    /* Compute under-actuated torques and forces that realize the gcomp torque */
    _force_opt->compute(_gcomp, _Fc, _tau_d);
    _model->setJointEffort(_tau_d); // set them inside the model state
    
    /* Set the effort from the model as reference for robot torque controllers */
    _robot->setReferenceFrom(*_model, XBot::Sync::Effort); // , XBot::Sync::Impedance);
    
    /* Get desired stiffness from the GcompIO */
    double stiffness_gain = 1.0;
    if(_shobj_stiffness.try_get(stiffness_gain))
    {
        _k = stiffness_gain * _k0;
        _robot->setStiffness(_k);
    }
    
    _robot->move();

}

bool GcompPlugin::close()
{
    /* This function is called exactly once, at the end of the experiment.
     * It can be used to do some clean-up, or to save logging data to disk. */

    /* Save logged data to disk */
    _logger->flush();

    return true;
}

GcompPlugin::~GcompPlugin()
{
  
}

}
