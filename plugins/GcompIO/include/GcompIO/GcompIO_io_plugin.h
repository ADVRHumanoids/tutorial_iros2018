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

#ifndef GcompIO_IOPLUGIN_H_
#define GcompIO_IOPLUGIN_H_

#include <XCM/IOPlugin.h>
#include <XBotCore-interfaces/XBotSharedMemory.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace XBotPlugin {

/**
 * @brief GcompIO XBot IO Plugin. This plugin extends the CommunicationHandler
 * loop with custom functionalities.
 *
 **/
class GcompIO : public XBot::IOPlugin
{

public:

    virtual bool init(std::string path_to_config_file, XBot::SharedMemory::Ptr shmem);

    virtual void run();

    virtual void close();

protected:


private:
    
    void callback(const std_msgs::Float64ConstPtr& msg);

    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    XBot::SharedObject<double> _shobj_stiffness;


};

}

#endif // GcompIO_PLUGIN_H_