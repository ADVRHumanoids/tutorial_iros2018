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

#ifndef GcompPlugin_PLUGIN_H_
#define GcompPlugin_PLUGIN_H_

#include <XCM/XBotControlPlugin.h>
#include <OpenSoT/utils/ForceOptimization.h>


namespace XBotPlugin {

/**
 * @brief GcompPlugin XBot RT Plugin
 *
 **/
class GcompPlugin : public XBot::XBotControlPlugin
{

public:

    virtual bool init_control_plugin(XBot::Handle::Ptr handle);

    virtual bool close();

    virtual void on_start(double time);

    virtual void on_stop(double time);
    
    virtual ~GcompPlugin();

protected:

    virtual void control_loop(double time, double period);

private:

    XBot::RobotInterface::Ptr _robot;
    XBot::ModelInterface::Ptr _model;
    XBot::ImuSensor::ConstPtr _imu;
    
    XBot::SharedObject<double> _shobj_stiffness;

    double _start_time;

    std::vector<std::string> _contact_links;
    std::vector<Eigen::Vector6d> _Fc;
    
    Eigen::VectorXd _gcomp, _tau_d, _k0, _k;
    Eigen::MatrixXd _JC;

    XBot::MatLogger::Ptr _logger;
    
    OpenSoT::utils::ForceOptimization::Ptr _force_opt;

};

}

#endif // GcompPlugin_PLUGIN_H_
