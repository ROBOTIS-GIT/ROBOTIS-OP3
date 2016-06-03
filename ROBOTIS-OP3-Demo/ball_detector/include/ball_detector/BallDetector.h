#ifndef _BALL_DETECTOR_H_
#define _BALL_DETECTOR_H_

//std
#include <string>

//ros dependencies
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/CameraInfo.h"
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include "ball_detector/circleSetStamped.h"
#include "ball_detector/DetectorConfig.h"
#include "ball_detector/detectorParamsConfig.h"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"


namespace robotis_op {

class BallDetector
{
      protected:
            //ros node handle
            ros::NodeHandle nh;

            //image publisher/subscriber
            image_transport::ImageTransport it;
            image_transport::Publisher imagePub;
            cv_bridge::CvImage cvImgPub;
            image_transport::Subscriber imageSubs;
            cv_bridge::CvImagePtr cvImgPtrSubs;

            bool init_param;

            //circle set publisher
             ball_detector::circleSetStamped circlesMsg;
             ros::Publisher circlesPub;

            //camera info subscriber
            sensor_msgs::CameraInfo cameraInfoMsg;
            ros::Subscriber cameraInfoSub;
            ros::Publisher cameraInfoPub;

            //dynamic reconfigure
            DetectorConfig params_config;
            std::string param_path;
            bool has_path;

            //flag indicating a new image has been received
            bool newImageFlag;

            //image time stamp and frame id
            ros::Time sub_time;
            std::string image_frame_id;

            //img encoding id
            unsigned int imgEncoding;

            /** \brief Set of detected circles
             *
             * Detected circles. For a circle i:
             *    x_i: circles[i][0]
             *    y_i: circles[i][1]
             *    radius_i: circles[i][2]
             *
             **/
            cv::vector<cv::Vec3f> circles;
            cv::Mat inImage;
            cv::Mat outImage;

            dynamic_reconfigure::Server<ball_detector::detectorParamsConfig> _param_server;
            dynamic_reconfigure::Server<ball_detector::detectorParamsConfig>::CallbackType _callback_fnc;

      protected:
            //callbacks to image subscription
            void imageCallback(const sensor_msgs::ImageConstPtr & msg);

            //callbacks to camera info subscription
            void cameraInfoCallback(const sensor_msgs::CameraInfo & msg);

            void dynParamCallback(ball_detector::detectorParamsConfig &config, uint32_t level);

            void printConfig();
            void saveConfig();
            void setInputImage(const cv::Mat & inIm);
            void getOutputImage(cv::Mat & outIm);
            void filterImage();
            void houghDetection(const unsigned int imgEncoding);
            void drawOutputImage();

      public:
            //constructor
            BallDetector();

            //destructor
            ~BallDetector();

            //checks if a new image has been received
            bool newImage();

            //execute circle detection with the cureent image
            void process();

            //publish the output image (input image + marked circles)
            void publishImage();

            //publish the circle set data
            void publishCircles();
};

}       // namespace
#endif  // _BALL_DETECTOR_H_
