
// include
#include "op3_localization/op3_localization.h"

//node main
int main(int argc, char **argv)
{
  //init ros
  ros::init(argc, argv, "op3_localization");

  //create ros wrapper object
  robotis_op::OP3Localization op3_localization;

  //set node loop rate
  ros::Rate loop_rate(10);

  //node loop
  while ( ros::ok() )
  {
    op3_localization.process();

    //execute pending callbacks
    ros::spinOnce();

    //relax to fit output rate
    loop_rate.sleep();
  }

  //exit program
  return 0;
}

