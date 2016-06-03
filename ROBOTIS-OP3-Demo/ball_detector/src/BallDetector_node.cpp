
//ros dependencies
#include "ball_detector/BallDetector.h"

//node main
int main(int argc, char **argv)
{
      //init ros
      ros::init(argc, argv, "ball_detector_node");

      //create ros wrapper object
      robotis_op::BallDetector detector;

      //set node loop rate
      ros::Rate loop_rate(30);

      //node loop
      while ( ros::ok() )
      {

            //if new image , do things
            if ( detector.newImage() )
            {
                  detector.process();
                  detector.publishImage();
                  detector.publishCircles();
            }

            //execute pending callbacks
            ros::spinOnce();

            //relax to fit output rate
            loop_rate.sleep();
      }

      //exit program
      return 0;
}

