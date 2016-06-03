#include <yaml-cpp/yaml.h>
#include <fstream>

#include "ball_detector/BallDetector.h"

namespace robotis_op {

BallDetector::BallDetector() : nh(ros::this_node::getName()) , it(this->nh), params_config(), init_param(false)
{
    has_path = nh.getParam("yaml_path", param_path);

    if(has_path) std::cout << "Path : " << param_path << std::endl;

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
    nh.param<int>("gaussian_blur_size", detCfg.gaussian_blur_size, params_config.gaussian_blur_size);
    if(detCfg.gaussian_blur_size % 2 == 0) detCfg.gaussian_blur_size -= 1;
    if(detCfg.gaussian_blur_size <= 0) detCfg.gaussian_blur_size = 1;
    nh.param<double>("gaussian_blur_sigma", detCfg.gaussian_blur_sigma, params_config.gaussian_blur_sigma);
    nh.param<double>("canny_edge_th", detCfg.canny_edge_th, params_config.canny_edge_th);
    nh.param<double>("hough_accum_resolution", detCfg.hough_accum_resolution, params_config.hough_accum_resolution);
    nh.param<double>("min_circle_dist", detCfg.min_circle_dist, params_config.min_circle_dist);
    nh.param<double>("hough_accum_th", detCfg.hough_accum_th, params_config.hough_accum_th);
    nh.param<int>("min_radius", detCfg.min_radius, params_config.min_radius);
    nh.param<int>("max_radius", detCfg.max_radius, params_config.max_radius);
    nh.param<int>("filter_threshold", detCfg.filter_threshold, params_config.filter_threshold);
    nh.param<bool>("filter_debug", detCfg.debug, params_config.debug);


    //sets publishers
    imagePub = it.advertise("image_out", 100);
    circlesPub = nh.advertise<ball_detector::circleSetStamped>("circle_set", 100);
    cameraInfoPub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 100);

    //sets subscribers
    imageSubs = it.subscribe("image_in", 1, &BallDetector::imageCallback, this);
    cameraInfoSub = nh.subscribe("cameraInfo_in", 100, &BallDetector::cameraInfoCallback, this);

    //initializes newImageFlag
    newImageFlag = false;

    // dynamic_reconfigure
    _callback_fnc = boost::bind(&BallDetector::dynParamCallback, this, _1, _2);
    _param_server.setCallback(_callback_fnc);

    //sets config and prints it
    params_config = detCfg;
    init_param = true;
    printConfig();
}

BallDetector::~BallDetector()
{

}

bool BallDetector::newImage()
{
    if ( newImageFlag )
    {
        newImageFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}

void BallDetector::process()
{
    if ( cvImgPtrSubs!=NULL )
    {
        //sets input image
        setInputImage(cvImgPtrSubs->image);

        // test image filtering
        filterImage();

        //detect circles
        houghDetection(this->imgEncoding);
    }
}

void BallDetector::publishImage()
{
    //image_raw topic
    cvImgPub.header.seq ++;
    cvImgPub.header.stamp = sub_time;
    cvImgPub.header.frame_id = image_frame_id;
    switch(imgEncoding)
    {
        case IMG_RGB8: cvImgPub.encoding = sensor_msgs::image_encodings::RGB8; break;
        case IMG_MONO: cvImgPub.encoding = sensor_msgs::image_encodings::MONO8; break;
        default: cvImgPub.encoding = sensor_msgs::image_encodings::MONO8; break;
    }
    getOutputImage(cvImgPub.image);
    imagePub.publish(cvImgPub.toImageMsg());
    cameraInfoPub.publish(cameraInfoMsg);
}

void BallDetector::publishCircles()
{
    if(circles.size() == 0) return;

    //clears and resize the message
    circlesMsg.circles.clear();
    circlesMsg.circles.resize(circles.size());

    //fill header
    circlesMsg.header.seq ++;
    circlesMsg.header.stamp = sub_time;
    circlesMsg.header.frame_id = "detector"; //To do: get frame_id from input image

    //fill circle data
    for(int idx = 0; idx < circles.size(); idx++ )
    {
        circlesMsg.circles[idx].x = circles[idx][0];    // x
        circlesMsg.circles[idx].y = circles[idx][1];    // y
        circlesMsg.circles[idx].z = circles[idx][2];    // radius
    }

    //publish message
    circlesPub.publish(circlesMsg);
}

void BallDetector::imageCallback(const sensor_msgs::ImageConstPtr & msg)
{
    try
    {
        if ( msg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ) this->imgEncoding = IMG_MONO;
        if ( msg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0 ) this->imgEncoding = IMG_RGB8;
        this->cvImgPtrSubs = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //indicates a new image is available
    this->sub_time = msg->header.stamp;
    this->image_frame_id = msg->header.frame_id;
    this->newImageFlag = true;
    return;
}

void BallDetector::dynParamCallback(ball_detector::detectorParamsConfig &config, uint32_t level)
{
    params_config.gaussian_blur_size       = config.gaussian_blur_size;
    params_config.gaussian_blur_sigma      = config.gaussian_blur_sigma;
    params_config.canny_edge_th            = config.canny_edge_th;
    params_config.hough_accum_resolution   = config.hough_accum_resolution;
    params_config.min_circle_dist          = config.min_circle_dist;
    params_config.hough_accum_th           = config.hough_accum_th;
    params_config.min_radius               = config.min_radius;
    params_config.max_radius               = config.max_radius;
    params_config.filter_threshold         = config.filter_threshold;
    params_config.debug                    = config.debug_image;

    // gaussian_blur has to be odd number.
    if(params_config.gaussian_blur_size % 2 == 0) params_config.gaussian_blur_size -= 1;
    if(params_config.gaussian_blur_size <= 0) params_config.gaussian_blur_size = 1;

    printConfig();
    saveConfig();
}


void BallDetector::cameraInfoCallback(const sensor_msgs::CameraInfo & msg)
{
    cameraInfoMsg = msg;
}


void BallDetector::printConfig()
{
    if(init_param == false) return;

    std::cout << "Detetctor Configuration:" << std::endl
              << "    gaussian_blur_size: " << params_config.gaussian_blur_size << std::endl
              << "    gaussian_blur_sigma: " << params_config.gaussian_blur_sigma << std::endl
              << "    canny_edge_th: " << params_config.canny_edge_th << std::endl
              << "    hough_accum_resolution: " << params_config.hough_accum_resolution << std::endl
              << "    min_circle_dist: " << params_config.min_circle_dist << std::endl
              << "    hough_accum_th: " << params_config.hough_accum_th << std::endl
              << "    min_radius: " << params_config.min_radius << std::endl
              << "    max_radius: " << params_config.max_radius << std::endl
              << "    filter_threshold: " << params_config.filter_threshold << std::endl
              << "    filter_image_to_debug: " << params_config.debug << std::endl << std::endl;
}

void BallDetector::saveConfig()
{
    if(has_path == false) return;

    YAML::Emitter _out;

    _out << YAML::BeginMap;
    _out << YAML::Key << "gaussian_blur_size"       << YAML::Value << params_config.gaussian_blur_size;
    _out << YAML::Key << "gaussian_blur_sigma"      << YAML::Value << params_config.gaussian_blur_sigma;
    _out << YAML::Key << "canny_edge_th"            << YAML::Value << params_config.canny_edge_th;
    _out << YAML::Key << "hough_accum_resolution"   << YAML::Value << params_config.hough_accum_resolution;
    _out << YAML::Key << "min_circle_dist"          << YAML::Value << params_config.min_circle_dist;
    _out << YAML::Key << "hough_accum_th"           << YAML::Value << params_config.hough_accum_th;
    _out << YAML::Key << "min_radius"               << YAML::Value << params_config.min_radius;
    _out << YAML::Key << "max_radius"               << YAML::Value << params_config.max_radius;
    _out << YAML::Key << "filter_threshold"         << YAML::Value << params_config.filter_threshold;
    _out << YAML::Key << "filter_debug"             << YAML::Value << params_config.debug;
    _out << YAML::EndMap;

    // output to file
    std::ofstream fout(param_path.c_str());
    fout << _out.c_str();
}

void BallDetector::setInputImage(const cv::Mat & inIm)
{
    inImage = inIm.clone();

    if(params_config.debug == false) outImage = inImage.clone();
}

void BallDetector::getOutputImage(cv::Mat & outIm)
{
    this->drawOutputImage();
    outIm = outImage.clone();
}

void BallDetector::filterImage()
{
    if(!inImage.data) return;

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
    cv::cvtColor(inImage, imgCrCb, cv::COLOR_BGR2YCrCb);
    std::vector<cv::Mat> _channels;
    cv::split(imgCrCb, _channels);
    imgCr = _channels[2];

    cv::inRange(imgCr, params_config.filter_threshold, 255, imgCr);
    cv::erode(imgCr, imgCr, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(imgCr, imgCr, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

    cv::dilate(imgCr, imgCr, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::erode(imgCr, imgCr, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

    cv::cvtColor(imgCr, inImage, cv::COLOR_GRAY2RGB);
}

void BallDetector::houghDetection(const unsigned int imgEncoding)
{
    cv::Mat grayImage;
    cv::vector<cv::Vec3f> circlesCurrent;

    //clear previous circles
    circles.clear();

    // If input image is RGB, convert it to gray
    if ( imgEncoding == IMG_RGB8 ) cv::cvtColor(inImage, grayImage, CV_BGR2GRAY);

    //Reduce the noise so we avoid false circle detection
    cv::GaussianBlur( grayImage, grayImage, cv::Size(params_config.gaussian_blur_size, params_config.gaussian_blur_size), params_config.gaussian_blur_sigma );

    //Apply the Hough Transform to find the circles
    cv::HoughCircles( grayImage, circlesCurrent, CV_HOUGH_GRADIENT, params_config.hough_accum_resolution, params_config.min_circle_dist, params_config.canny_edge_th, params_config.hough_accum_th, params_config.min_radius, params_config.max_radius );

    //set found circles to circles set. Apply some condition if desired.
    unsigned int ii;
    for (ii = 0; ii < circlesCurrent.size(); ii++ )
    {
        circles.push_back(circlesCurrent.at(ii));
        // std::cout << "circle " << ii << ": (" << circles.at(ii)[0] << "," << circles.at(ii)[1] << ")" << std::endl;
    }
}

void BallDetector::drawOutputImage()
{
    cv::Point _center;
    int _radius;
    size_t _ii;

    //draws results to output Image
    if(params_config.debug == true) outImage = inImage.clone();

    for( _ii = 0; _ii < circles.size(); _ii++ )
    {
        if ( circles[_ii][0] != -1 )
        {
            _center = cv::Point(cvRound(circles[_ii][0]), cvRound(circles[_ii][1]));
            _radius = cvRound(circles[_ii][2]);
            cv::circle( outImage, _center, 5, cv::Scalar(0,0,255), -1, 8, 0 );// circle center in green
            cv::circle( outImage, _center, _radius, cv::Scalar(0,0,255), 3, 8, 0 );// circle outline in red
        }
    }

}

}       // namespace robotis_op
