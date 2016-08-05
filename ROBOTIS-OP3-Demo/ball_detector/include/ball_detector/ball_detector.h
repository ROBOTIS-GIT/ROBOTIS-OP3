/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Kayman Jung */

#ifndef _BALL_DETECTOR_H_
#define _BALL_DETECTOR_H_

//std
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//ros dependencies
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>

#include "ball_detector/circleSetStamped.h"
#include "ball_detector/ball_detector_config.h"
#include "ball_detector/detectorParamsConfig.h"

namespace robotis_op
{

class BallDetector
{
 public:
  BallDetector();
  ~BallDetector();

  //checks if a new image has been received
  bool newImage();

  //execute circle detection with the cureent image
  void process();

  //publish the output image (input image + marked circles)
  void publishImage();

  //publish the circle set data
  void publishCircles();

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
  void makeFilterMask(const cv::Mat &source_img, cv::Mat &mask_img, int range);
  void inRangeHsv(const cv::Mat &input_img, const HsvFilter &filter_value, cv::Mat &output_img);
  void mophology(const cv::Mat &intput_img, cv::Mat &output_img, int ellipse_size);
  void houghDetection(const unsigned int imgEncoding);
  void drawOutputImage();

  //ros node handle
  ros::NodeHandle nh_;

  //image publisher/subscriber
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  cv_bridge::CvImage cv_img_pub_;
  image_transport::Subscriber image_sub_;
  cv_bridge::CvImagePtr cv_img_ptr_sub_;

  bool init_param_;

  //circle set publisher
  ball_detector::circleSetStamped circles_msg_;
  ros::Publisher circles_pub_;

  //camera info subscriber
  sensor_msgs::CameraInfo camera_info_msg_;
  ros::Subscriber camera_info_sub_;
  ros::Publisher camera_info_pub_;

  //dynamic reconfigure
  DetectorConfig params_config_;
  std::string param_path_;
  bool has_path_;

  //flag indicating a new image has been received
  bool new_image_flag_;

  //image time stamp and frame id
  ros::Time sub_time_;
  std::string image_frame_id_;

  //img encoding id
  unsigned int img_encoding_;

  /** \brief Set of detected circles
   *
   * Detected circles. For a circle i:
   *    x_i: circles[i][0]
   *    y_i: circles[i][1]
   *    radius_i: circles[i][2]
   *
   **/
  cv::vector<cv::Vec3f> circles_;
  cv::Mat in_image_;
  cv::Mat out_image_;

  dynamic_reconfigure::Server<ball_detector::detectorParamsConfig> param_server_;
  dynamic_reconfigure::Server<ball_detector::detectorParamsConfig>::CallbackType callback_fnc_;
};

}       // namespace
#endif  // _BALL_DETECTOR_H_
