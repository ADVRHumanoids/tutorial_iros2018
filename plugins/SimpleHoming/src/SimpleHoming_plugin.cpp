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

#include <SimpleHoming_plugin.h>

/* Specify that the class XBotPlugin::SimpleHoming is a XBot RT plugin with name "SimpleHoming" */
REGISTER_XBOT_PLUGIN_(XBotPlugin::SimpleHoming)

namespace XBotPlugin {

bool SimpleHoming::init_control_plugin(XBot::Handle::Ptr handle)
{
    /* This function is called outside the real time loop, so we can
     * allocate memory on the heap, print stuff, ...
     * The RT plugin will be executed only if this init function returns true. */


    /* Save robot to a private member. */
    _robot = handle->getRobotInterface();
    
    /* Get a homing posture from SRDF */
    _robot->getRobotState("home", _qhome);
    
    /* Allocate all vectors to be used inside control loop */
    _q0.setZero(_robot->getJointNum());
    _qref.setZero(_robot->getJointNum());
    _qdotref.setZero(_robot->getJointNum());

    /* Initialize a logger which saves to the specified file. Remember that
     * the current date/time is always appended to the provided filename,
     * so that logs do not overwrite each other. Preallocate 10000 samples per variable. */

    _logger = XBot::MatLogger::getLogger("/tmp/SimpleHoming_log");
    _logger->createScalarVariable("time", 1, 60000);
    _logger->createVectorVariable("qref", _robot->getJointNum(), 1, 60000);
    _logger->createVectorVariable("qrefdot", _robot->getJointNum(), 1, 60000);

    return true;


}

void SimpleHoming::on_start(double time)
{
    /* This function is called on plugin start, i.e. when the start command
     * is sent over the plugin switch port (e.g. 'rosservice call /SimpleHoming_switch true').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* Save the robot starting config to a class member */
    _robot->getMotorPosition(_q0);

    /* Save the plugin starting time to a class member */
    _start_time = time;
}

void SimpleHoming::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /SimpleHoming_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
}


void SimpleHoming::control_loop(double time, double period)
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
    

    
    double duration = 0.0; 
    const double MAX_SPEED = 1.0; // rad / s
    

    XBot::Utils::FifthOrderTrajectory(_start_time, _q0,    // [in] provide start time and posture
                                        _qhome,            // [in] target posture
                                        MAX_SPEED,         // [in] max joint speed
                                        time,              // [in] current time
                                        _qref, _qdotref,   // [out] q/qdot references are returned
                                        duration           // [out] total duration of the trajectory
                                        );
    
    _logger->add("time", time);
    _logger->add("qref", _qref);
    _logger->add("qdot_ref", _qdotref);
    
    _robot->setPositionReference(_qref);
    _robot->move();

    
    

}

bool SimpleHoming::close()
{
    /* This function is called exactly once, at the end of the experiment.
     * It can be used to do some clean-up, or to save logging data to disk. */

    /* Save logged data to disk */
    _logger->flush();

    return true;
}

SimpleHoming::~SimpleHoming()
{
  
}

}
