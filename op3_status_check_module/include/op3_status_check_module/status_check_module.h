#ifndef OP3_STATUS_CHECK_MODULE_STATUS_CHECK_MODULE_H
#define OP3_STATUS_CHECK_MODULE_STATUS_CHECK_MODULE_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int16.h>
#include <boost/thread.hpp>

#include "robotis_framework_common/sensor_module.h"
#include "robotis_controller_msgs/StatusMsg.h"

namespace robotis_op
{

class StatusCheckModule
  : public robotis_framework::SensorModule,
    public robotis_framework::Singleton<StatusCheckModule>
{
private:
  int           control_cycle_msec_;
  boost::thread queue_thread_;

  /* subscriber & publisher */
  //ros::Subscriber sub1_;
  ros::Publisher  status_msg_pub_;

  void queueThread();

public:
  StatusCheckModule();
  virtual ~StatusCheckModule();

  /* ROS Topic Callback Functions */
  //void topicCallback(const std_msgs::Int16::ConstPtr &msg);

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
               std::map<std::string, robotis_framework::Sensor *> sensors);
};

}


#endif // OP3_STATUS_CHECK_MODULE_STATUS_CHECK_MODULE_H
