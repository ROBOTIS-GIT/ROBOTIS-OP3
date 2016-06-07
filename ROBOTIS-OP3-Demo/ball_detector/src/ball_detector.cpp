#include <yaml-cpp/yaml.h>
#include <fstream>

#include "ball_detector/ball_detector.h"

namespace robotis_op {

BallDetector::BallDetector() : nh_(ros::this_node::getName()) , it_(this->nh_), params_config_(), init_param_(false)
{
    has_path_ = nh_.getParam("yaml_path", param_path_);

    if(has_path_) std::cout << "Path : " << param_path_ << std::endl;

    //detector config struct
    DetectorConfig detCfg;

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
    nh_.param<int>("gaussian_blur_size", detCfg.gaussian_blur_size, params_config_.gaussian_blur_size);
    if(detCfg.gaussian_blur_size % 2 == 0) detCfg.gaussian_blur_size -= 1;
    if(detCfg.gaussian_blur_size <= 0) detCfg.gaussian_blur_size = 1;
    nh_.param<double>("gaussian_blur_sigma", detCfg.gaussian_blur_sigma, params_config_.gaussian_blur_sigma);
    nh_.param<double>("canny_edge_th", detCfg.canny_edge_th, params_config_.canny_edge_th);
    nh_.param<double>("hough_accum_resolution", detCfg.hough_accum_resolution, params_config_.hough_accum_resolution);
    nh_.param<double>("min_circle_dist", detCfg.min_circle_dist, params_config_.min_circle_dist);
    nh_.param<double>("hough_accum_th", detCfg.hough_accum_th, params_config_.hough_accum_th);
    nh_.param<int>("min_radius", detCfg.min_radius, params_config_.min_radius);
    nh_.param<int>("max_radius", detCfg.max_radius, params_config_.max_radius);
    nh_.param<int>("filter_range_min", detCfg.filter_range_min, params_config_.filter_range_min);
    nh_.param<int>("filter_range_max", detCfg.filter_range_max, params_config_.filter_range_max);
    nh_.param<bool>("filter_debug", detCfg.debug, params_config_.debug);


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
    params_config_ = detCfg;
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
    params_config_.filter_range_min         = config.filter_range_min;
    params_config_.filter_range_max         = config.filter_range_max;
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
              << "    filter_range_min: " << params_config_.filter_range_min << std::endl
              << "    filter_range_max: " << params_config_.filter_range_max << std::endl
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
    _out << YAML::Key << "filter_range_min"         << YAML::Value << params_config_.filter_range_min;
    _out << YAML::Key << "filter_range_max"         << YAML::Value << params_config_.filter_range_max;
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
    cv::Mat imgHSV;

    cv::cvtColor(inImage, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    cv::Mat imgThresholded, invertImage;

    cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    //morphological opening (removes small objects from the foreground)
    // cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    // cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

    //morphological closing (removes small holes from the foreground)
    // cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    // cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

    cv::bitwise_not ( imgThresholded, invertImage );
    cv::cvtColor(invertImage, inImage, cv::COLOR_GRAY2BGR);
    */

    cv::Mat imgCrCb, imgCr;
    cv::cvtColor(in_image_, imgCrCb, cv::COLOR_BGR2YCrCb);
    std::vector<cv::Mat> _channels;
    cv::split(imgCrCb, _channels);
    imgCr = _channels[2];

    cv::inRange(imgCr, params_config_.filter_range_min, params_config_.filter_range_max, imgCr);
    cv::erode(imgCr, imgCr, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(imgCr, imgCr, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

    cv::dilate(imgCr, imgCr, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::erode(imgCr, imgCr, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

    cv::cvtColor(imgCr, in_image_, cv::COLOR_GRAY2RGB);
}

void BallDetector::houghDetection(const unsigned int imgEncoding)
{
    cv::Mat grayImage;
    cv::vector<cv::Vec3f> circlesCurrent;

    //clear previous circles
    circles_.clear();

    // If input image is RGB, convert it to gray
    if ( imgEncoding == IMG_RGB8 ) cv::cvtColor(in_image_, grayImage, CV_BGR2GRAY);

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
    int _radius;
    size_t _ii;

    //draws results to output Image
    if(params_config_.debug == true) out_image_ = in_image_.clone();

    for( _ii = 0; _ii < circles_.size(); _ii++ )
    {
        if ( circles_[_ii][0] != -1 )
        {
            _center = cv::Point(cvRound(circles_[_ii][0]), cvRound(circles_[_ii][1]));
            _radius = cvRound(circles_[_ii][2]);
            cv::circle( out_image_, _center, 5, cv::Scalar(0,0,255), -1, 8, 0 );// circle center in green
            cv::circle( out_image_, _center, _radius, cv::Scalar(0,0,255), 3, 8, 0 );// circle outline in red
        }
    }

}

}       // namespace robotis_op
