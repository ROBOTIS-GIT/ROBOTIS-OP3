/*
 * offset_tuner_server_node.cpp
 *
 *  Created on: 2016. 2. 15.
 *      Author: HJSONG
 */

#include <ros/ros.h>

#include "op3_offset_tuner_server/OP3OffsetTunerServer.h"

using namespace ROBOTIS;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offset_tuner_server_node");

    OffsetTunerServer* server = OffsetTunerServer::GetInstance();

    server->Initialize();

    ros::spin();

	return 0;
}
