#include <yaml-cpp/yaml.h>
#include <fstream>

#include "ball_detector/ball_detector.h"

namespace robotis_op {

BallDetector::BallDetector() : nh_(ros::this_node::getName()) , it_(this->nh_), params_config_(), init_param_(false)
{
  has_path_ = nh_.getParam("yaml_path", param_path_);

  if(has_path_) std::cout << "Path : " << param_path_ << std::endl;

  //detector config struct
  DetectorConfig detect_config;

  /*
    //get user parameters from dynamic reconfigure (yaml entries)
    nh.getParam("gaussian_blur_size", detCfg.gaussian_blur_size, params.gaussian_blur_size);
    nh.getParam("gaussian_blur_sigma", detCfg.gaussian_blur_sigma);
    nh.getParam("canny_edge_th", detCfg.canny_edge_th);
    nh.getParam("hough_accum_resolution", detCfg.hough_accum_resolution);
    nh.getParam("min_circle_dist", detCfg.min_circle_dist);
    nh.getParam("hough_accum_th", detCfg.hough_accum_th);
    nh.getParam("min_radius", detCfg.min_radius);
    nh.getParam("max_radius", detCfg.max_radius);
    nh.getParam("filter_threshold", detCfg.filter_threshold);
    nh.param<bool>("filter_debug", detCfg.debug, false);
   */

  //get user parameters from dynamic reconfigure (yaml entries)
  nh_.param<int>("gaussian_blur_size", detect_config.gaussian_blur_size, params_config_.gaussian_blur_size);
  if(detect_config.gaussian_blur_size % 2 == 0) detect_config.gaussian_blur_size -= 1;
  if(detect_config.gaussian_blur_size <= 0) detect_config.gaussian_blur_size = 1;
  nh_.param<double>("gaussian_blur_sigma", detect_config.gaussian_blur_sigma, params_config_.gaussian_blur_sigma);
  nh_.param<double>("canny_edge_th", detect_config.canny_edge_th, params_config_.canny_edge_th);
  nh_.param<double>("hough_accum_resolution", detect_config.hough_accum_resolution, params_config_.hough_accum_resolution);
  nh_.param<double>("min_circle_dist", detect_config.min_circle_dist, params_config_.min_circle_dist);
  nh_.param<double>("hough_accum_th", detect_config.hough_accum_th, params_config_.hough_accum_th);
  nh_.param<int>("min_radius", detect_config.min_radius, params_config_.min_radius);
  nh_.param<int>("max_radius", detect_config.max_radius, params_config_.max_radius);
  nh_.param<int>("filter_h_min", detect_config.filter_threshold.h_min, params_config_.filter_threshold.h_min);
  nh_.param<int>("filter_h_max", detect_config.filter_threshold.h_max, params_config_.filter_threshold.h_max);
  nh_.param<int>("filter_s_min", detect_config.filter_threshold.s_min, params_config_.filter_threshold.s_min);
  nh_.param<int>("filter_s_max", detect_config.filter_threshold.s_max, params_config_.filter_threshold.s_max);
  nh_.param<int>("filter_v_min", detect_config.filter_threshold.v_min, params_config_.filter_threshold.v_min);
  nh_.param<int>("filter_v_max", detect_config.filter_threshold.v_max, params_config_.filter_threshold.v_max);
  nh_.param<bool>("use_second_filter", detect_config.use_second_filter, params_config_.use_second_filter);
  nh_.param<int>("filter2_h_min", detect_config.filter2_threshold.h_min, params_config_.filter2_threshold.h_min);
  nh_.param<int>("filter2_h_max", detect_config.filter2_threshold.h_max, params_config_.filter2_threshold.h_max);
  nh_.param<int>("filter2_s_min", detect_config.filter2_threshold.s_min, params_config_.filter2_threshold.s_min);
  nh_.param<int>("filter2_s_max", detect_config.filter2_threshold.s_max, params_config_.filter2_threshold.s_max);
  nh_.param<int>("filter2_v_min", detect_config.filter2_threshold.v_min, params_config_.filter2_threshold.v_min);
  nh_.param<int>("filter2_v_max", detect_config.filter2_threshold.v_max, params_config_.filter2_threshold.v_max);
  nh_.param<int>("ellipse_size", detect_config.ellipse_size, params_config_.ellipse_size);
  nh_.param<bool>("filter_debug", detect_config.debug, params_config_.debug);


  //sets publishers
  image_pub_ = it_.advertise("image_out", 100);
  circles_pub_ = nh_.advertise<ball_detector::circleSetStamped>("circle_set", 100);
  camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 100);

  //sets subscribers
  image_sub_ = it_.subscribe("image_in", 1, &BallDetector::imageCallback, this);
  camera_info_sub_ = nh_.subscribe("cameraInfo_in", 100, &BallDetector::cameraInfoCallback, this);

  //initializes newImageFlag
  new_image_flag_ = false;

  // dynamic_reconfigure
  callback_fnc_ = boost::bind(&BallDetector::dynParamCallback, this, _1, _2);
  param_server_.setCallback(callback_fnc_);

  //sets config and prints it
  params_config_ = detect_config;
  init_param_ = true;
  printConfig();
}

BallDetector::~BallDetector()
{

}

bool BallDetector::newImage()
{
  if ( new_image_flag_ )
  {
    new_image_flag_ = false;
    return true;
  }
  else
  {
    return false;
  }
}

void BallDetector::process()
{
  if ( cv_img_ptr_sub_!=NULL )
  {
    //sets input image
    setInputImage(cv_img_ptr_sub_->image);

    // test image filtering
    filterImage();

    //detect circles
    houghDetection(this->img_encoding_);
  }
}

void BallDetector::publishImage()
{
  //image_raw topic
  cv_img_pub_.header.seq ++;
  cv_img_pub_.header.stamp = sub_time_;
  cv_img_pub_.header.frame_id = image_frame_id_;
  switch(img_encoding_)
  {
    case IMG_RGB8: cv_img_pub_.encoding = sensor_msgs::image_encodings::RGB8; break;
    case IMG_MONO: cv_img_pub_.encoding = sensor_msgs::image_encodings::MONO8; break;
    default: cv_img_pub_.encoding = sensor_msgs::image_encodings::MONO8; break;
  }
  getOutputImage(cv_img_pub_.image);
  image_pub_.publish(cv_img_pub_.toImageMsg());
  camera_info_pub_.publish(camera_info_msg_);
}

void BallDetector::publishCircles()
{
  if(circles_.size() == 0) return;

  //clears and resize the message
  circles_msg_.circles.clear();
  circles_msg_.circles.resize(circles_.size());

  //fill header
  circles_msg_.header.seq ++;
  circles_msg_.header.stamp = sub_time_;
  circles_msg_.header.frame_id = "detector"; //To do: get frame_id from input image

  //fill circle data
  for(int idx = 0; idx < circles_.size(); idx++ )
  {
    circles_msg_.circles[idx].x = circles_[idx][0];    // x
    circles_msg_.circles[idx].y = circles_[idx][1];    // y
    circles_msg_.circles[idx].z = circles_[idx][2];    // radius
  }

  //publish message
  circles_pub_.publish(circles_msg_);
}

void BallDetector::imageCallback(const sensor_msgs::ImageConstPtr & msg)
{
  try
  {
    if ( msg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ) this->img_encoding_ = IMG_MONO;
    if ( msg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ) this->img_encoding_ = IMG_RGB8;
    this->cv_img_ptr_sub_ = cv_bridge::toCvCopy(msg, msg->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  //indicates a new image is available
  this->sub_time_ = msg->header.stamp;
  this->image_frame_id_ = msg->header.frame_id;
  this->new_image_flag_ = true;
  return;
}

void BallDetector::dynParamCallback(ball_detector::detectorParamsConfig &config, uint32_t level)
{
  params_config_.gaussian_blur_size       = config.gaussian_blur_size;
  params_config_.gaussian_blur_sigma      = config.gaussian_blur_sigma;
  params_config_.canny_edge_th            = config.canny_edge_th;
  params_config_.hough_accum_resolution   = config.hough_accum_resolution;
  params_config_.min_circle_dist          = config.min_circle_dist;
  params_config_.hough_accum_th           = config.hough_accum_th;
  params_config_.min_radius               = config.min_radius;
  params_config_.max_radius               = config.max_radius;
  params_config_.filter_threshold.h_min   = config.filter_h_min;
  params_config_.filter_threshold.h_max   = config.filter_h_max;
  params_config_.filter_threshold.s_min   = config.filter_s_min;
  params_config_.filter_threshold.s_max   = config.filter_s_max;
  params_config_.filter_threshold.v_min   = config.filter_v_min;
  params_config_.filter_threshold.v_max   = config.filter_v_max;
  params_config_.use_second_filter        = config.use_second_filter;
  params_config_.filter2_threshold.h_min  = config.filter2_h_min;
  params_config_.filter2_threshold.h_max  = config.filter2_h_max;
  params_config_.filter2_threshold.s_min  = config.filter2_s_min;
  params_config_.filter2_threshold.s_max  = config.filter2_s_max;
  params_config_.filter2_threshold.v_min  = config.filter2_v_min;
  params_config_.filter2_threshold.v_max  = config.filter2_v_max;
  params_config_.ellipse_size             = config.ellipse_size;
  params_config_.debug                    = config.debug_image;

  // gaussian_blur has to be odd number.
  if(params_config_.gaussian_blur_size % 2 == 0) params_config_.gaussian_blur_size -= 1;
  if(params_config_.gaussian_blur_size <= 0) params_config_.gaussian_blur_size = 1;

  printConfig();
  saveConfig();
}


void BallDetector::cameraInfoCallback(const sensor_msgs::CameraInfo & msg)
{
  camera_info_msg_ = msg;
}


void BallDetector::printConfig()
{
  if(init_param_ == false) return;

  std::cout << "Detetctor Configuration:" << std::endl
      << "    gaussian_blur_size: " << params_config_.gaussian_blur_size << std::endl
      << "    gaussian_blur_sigma: " << params_config_.gaussian_blur_sigma << std::endl
      << "    canny_edge_th: " << params_config_.canny_edge_th << std::endl
      << "    hough_accum_resolution: " << params_config_.hough_accum_resolution << std::endl
      << "    min_circle_dist: " << params_config_.min_circle_dist << std::endl
      << "    hough_accum_th: " << params_config_.hough_accum_th << std::endl
      << "    min_radius: " << params_config_.min_radius << std::endl
      << "    max_radius: " << params_config_.max_radius << std::endl
      << "    filter_h_min: " << params_config_.filter_threshold.h_min << std::endl
      << "    filter_h_max: " << params_config_.filter_threshold.h_max << std::endl
      << "    filter_s_min: " << params_config_.filter_threshold.s_min << std::endl
      << "    filter_s_max: " << params_config_.filter_threshold.s_max << std::endl
      << "    filter_v_min: " << params_config_.filter_threshold.v_min << std::endl
      << "    filter_v_max: " << params_config_.filter_threshold.v_max << std::endl
      << "    use_second_filter: " << params_config_.use_second_filter << std::endl
      << "    filter2_h_min: " << params_config_.filter2_threshold.h_min << std::endl
      << "    filter2_h_max: " << params_config_.filter2_threshold.h_max << std::endl
      << "    filter2_s_min: " << params_config_.filter2_threshold.s_min << std::endl
      << "    filter2_s_max: " << params_config_.filter2_threshold.s_max << std::endl
      << "    filter2_v_min: " << params_config_.filter2_threshold.v_min << std::endl
      << "    filter2_v_max: " << params_config_.filter2_threshold.v_max << std::endl
      << "    ellipse_size: " << params_config_.ellipse_size << std::endl
      << "    filter_image_to_debug: " << params_config_.debug << std::endl << std::endl;
}

void BallDetector::saveConfig()
{
  if(has_path_ == false) return;

  YAML::Emitter _out;

  _out << YAML::BeginMap;
  _out << YAML::Key << "gaussian_blur_size"       << YAML::Value << params_config_.gaussian_blur_size;
  _out << YAML::Key << "gaussian_blur_sigma"      << YAML::Value << params_config_.gaussian_blur_sigma;
  _out << YAML::Key << "canny_edge_th"            << YAML::Value << params_config_.canny_edge_th;
  _out << YAML::Key << "hough_accum_resolution"   << YAML::Value << params_config_.hough_accum_resolution;
  _out << YAML::Key << "min_circle_dist"          << YAML::Value << params_config_.min_circle_dist;
  _out << YAML::Key << "hough_accum_th"           << YAML::Value << params_config_.hough_accum_th;
  _out << YAML::Key << "min_radius"               << YAML::Value << params_config_.min_radius;
  _out << YAML::Key << "max_radius"               << YAML::Value << params_config_.max_radius;
  _out << YAML::Key << "filter_h_min"             << YAML::Value << params_config_.filter_threshold.h_min;
  _out << YAML::Key << "filter_h_max"             << YAML::Value << params_config_.filter_threshold.h_max;
  _out << YAML::Key << "filter_s_min"             << YAML::Value << params_config_.filter_threshold.s_min;
  _out << YAML::Key << "filter_s_max"             << YAML::Value << params_config_.filter_threshold.s_max;
  _out << YAML::Key << "filter_v_min"             << YAML::Value << params_config_.filter_threshold.v_min;
  _out << YAML::Key << "filter_v_max"             << YAML::Value << params_config_.filter_threshold.v_max;
  _out << YAML::Key << "use_second_filter"        << YAML::Value << params_config_.use_second_filter;
  _out << YAML::Key << "filter2_h_min"            << YAML::Value << params_config_.filter2_threshold.h_min;
  _out << YAML::Key << "filter2_h_max"            << YAML::Value << params_config_.filter2_threshold.h_max;
  _out << YAML::Key << "filter2_s_min"            << YAML::Value << params_config_.filter2_threshold.s_min;
  _out << YAML::Key << "filter2_s_max"            << YAML::Value << params_config_.filter2_threshold.s_max;
  _out << YAML::Key << "filter2_v_min"            << YAML::Value << params_config_.filter2_threshold.v_min;
  _out << YAML::Key << "filter2_v_max"            << YAML::Value << params_config_.filter2_threshold.v_max;
  _out << YAML::Key << "ellipse_size"             << YAML::Value << params_config_.ellipse_size;
  _out << YAML::Key << "filter_debug"             << YAML::Value << params_config_.debug;
  _out << YAML::EndMap;

  // output to file
  std::ofstream fout(param_path_.c_str());
  fout << _out.c_str();
}

void BallDetector::setInputImage(const cv::Mat & inIm)
{
  in_image_ = inIm.clone();

  if(params_config_.debug == false) out_image_ = in_image_.clone();
}

void BallDetector::getOutputImage(cv::Mat & outIm)
{
  this->drawOutputImage();
  outIm = out_image_.clone();
}

void BallDetector::filterImage()
{
  if(!in_image_.data) return;

  /*
  cv::Mat imgCrCb, imgCr;
  cv::cvtColor(in_image_, imgCrCb, cv::COLOR_BGR2YCrCb);
  std::vector<cv::Mat> _channels;
  cv::split(imgCrCb, _channels);
  imgCr = _channels[2];

  cv::inRange(imgCr, params_config_.filter_h_min, params_config_.filter_h_max, imgCr);
  cv::erode(imgCr, imgCr, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
  cv::dilate(imgCr, imgCr, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

  cv::dilate(imgCr, imgCr, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
  cv::erode(imgCr, imgCr, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

  cv::cvtColor(imgCr, in_image_, cv::COLOR_GRAY2RGB);
   */

  cv::Mat imgHsv, imgFiltered;
  cv::cvtColor(in_image_, imgHsv, cv::COLOR_RGB2HSV);

  inRangeHsv(imgHsv, params_config_.filter_threshold, imgFiltered);

  if(params_config_.use_second_filter == true)
  {
    // mask
    cv::Mat imgMask;

    // mophology : open and close
    int ellipse_size = 5;
    cv::erode(imgFiltered, imgMask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ellipse_size, ellipse_size)) );
    cv::dilate(imgMask, imgMask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ellipse_size * 10, ellipse_size * 10)) );

    // check hsv range
    cv::Mat imgFiltered2;
    inRangeHsv(imgHsv, params_config_.filter2_threshold, imgFiltered2);

    cv::bitwise_and(imgFiltered2, imgMask, imgFiltered2);

    // or
    cv::bitwise_or(imgFiltered, imgFiltered2, imgFiltered);
  }

  // mophology : open and close
  mophology(imgFiltered, imgFiltered, params_config_.ellipse_size);

  cv::cvtColor(imgFiltered, in_image_, cv::COLOR_GRAY2RGB);
}

void BallDetector::makeFilterMask(const cv::Mat &source_img, cv::Mat &mask_img, int range)
{
  // source_img.
  mask_img = cv::Mat::zeros(source_img.rows, source_img.cols, CV_8UC1);

  int source_height = source_img.rows;
  int source_width = source_img.cols;

  // channel : 1
  if(source_img.channels() != 1)
    return;

  for(int i = 0; i < source_height; i++)
  {
    for(int j = 0; j < source_width; j++)
    {
      uint8_t pixel = source_img.at<uint8_t>(i, j);

      if(pixel == 0)
        continue;

      for(int mask_i = i - range; mask_i <= i + range; mask_i++)
      {
        if(mask_i < 0 || mask_i >= source_height) continue;

        for(int mask_j = j - range; mask_j <= j + range; mask_j++)
        {
          if(mask_j < 0 || mask_j >= source_width) continue;

          mask_img.at<uchar>(mask_i, mask_j, 0) = 255;
        }
      }
    }
  }
}

void BallDetector::inRangeHsv(const cv::Mat &input_img, const HsvFilter &filter_value, cv::Mat &output_img)
{
  int scaled_hue_min = static_cast<int>(filter_value.h_min * 0.5);
  int scaled_hue_max = static_cast<int>(filter_value.h_max * 0.5);

  if(scaled_hue_min <= scaled_hue_max)
  {
    cv::Scalar min_value = cv::Scalar(scaled_hue_min , filter_value.s_min, filter_value.v_min, 0);
    cv::Scalar max_value = cv::Scalar(scaled_hue_max , filter_value.s_max, filter_value.v_max, 0);

    cv::inRange(input_img, min_value, max_value, output_img);
  }
  else
  {
    cv::Mat lower_hue_range, upper_hue_range;
    cv::Scalar min_value, max_value;

    min_value = cv::Scalar(0 , filter_value.s_min, filter_value.v_min, 0);
    max_value = cv::Scalar(scaled_hue_max , filter_value.s_max, filter_value.v_max, 0);
    cv::inRange(input_img, min_value, max_value, lower_hue_range);

    min_value = cv::Scalar(scaled_hue_min , filter_value.s_min, filter_value.v_min, 0);
    max_value = cv::Scalar(179 , filter_value.s_max, filter_value.v_max, 0);
    cv::inRange(input_img, min_value, max_value, upper_hue_range);

    cv::bitwise_or(lower_hue_range, upper_hue_range, output_img);
  }
}

void BallDetector::mophology(const cv::Mat &intput_img, cv::Mat &output_img, int ellipse_size)
{
  cv::erode(intput_img, output_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ellipse_size, ellipse_size)) );
  cv::dilate(output_img, output_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ellipse_size * 2, ellipse_size * 2)) );

  cv::dilate(output_img, output_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ellipse_size, ellipse_size)) );
  cv::erode(output_img, output_img, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ellipse_size, ellipse_size)) );
}

void BallDetector::houghDetection(const unsigned int imgEncoding)
{
  cv::Mat grayImage;
  cv::vector<cv::Vec3f> circlesCurrent;

  //clear previous circles
  circles_.clear();

  // If input image is RGB, convert it to gray
  if ( imgEncoding == IMG_RGB8 ) cv::cvtColor(in_image_, grayImage, CV_RGB2GRAY);

  //Reduce the noise so we avoid false circle detection
  cv::GaussianBlur( grayImage, grayImage, cv::Size(params_config_.gaussian_blur_size, params_config_.gaussian_blur_size), params_config_.gaussian_blur_sigma );

  //Apply the Hough Transform to find the circles
  cv::HoughCircles( grayImage, circlesCurrent, CV_HOUGH_GRADIENT, params_config_.hough_accum_resolution, params_config_.min_circle_dist, params_config_.canny_edge_th, params_config_.hough_accum_th, params_config_.min_radius, params_config_.max_radius );

  //set found circles to circles set. Apply some condition if desired.
  unsigned int ii;
  for (ii = 0; ii < circlesCurrent.size(); ii++ )
  {
    circles_.push_back(circlesCurrent.at(ii));
    // std::cout << "circle " << ii << ": (" << circles.at(ii)[0] << "," << circles.at(ii)[1] << ")" << std::endl;
  }
}

void BallDetector::drawOutputImage()
{
  cv::Point _center;
  int _radius = 0;
  size_t _ii;

  //draws results to output Image
  if(params_config_.debug == true) out_image_ = in_image_.clone();

  for( _ii = 0; _ii < circles_.size(); _ii++ )
  {
    if ( circles_[_ii][0] != -1 )
    {
      //      _center = cv::Point(cvRound(circles_[_ii][0]), cvRound(circles_[_ii][1]));
      //      _radius = cvRound(circles_[_ii][2]);
      //      cv::circle( out_image_, _center, 5, cv::Scalar(0,0,255), -1, 8, 0 );// circle center in green
      //      cv::circle( out_image_, _center, _radius, cv::Scalar(0,0,255), 3, 8, 0 );// circle outline in red
      int this_radius = cvRound(circles_[_ii][2]);
      if(this_radius > _radius)
      {
        _radius = this_radius;
        _center = cv::Point(cvRound(circles_[_ii][0]), cvRound(circles_[_ii][1]));
      }
    }
  }
  cv::circle( out_image_, _center, 5, cv::Scalar(0,0,255), -1, 8, 0 );// circle center in blue
  cv::circle( out_image_, _center, _radius, cv::Scalar(0,0,255), 3, 8, 0 );// circle outline in blue

}

}       // namespace robotis_op
