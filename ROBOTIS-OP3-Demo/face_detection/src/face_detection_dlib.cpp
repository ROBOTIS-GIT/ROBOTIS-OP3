//##########################################################################
// DO NOT MODIFY
//
//This project was created within an academic research setting, and thus should
//be considered as EXPERIMENTAL code. There may be bugs and deficiencies in the
//code, so please adjust expectations accordingly. With that said, we are
//intrinsically motivated to ensure its correctness (and often its performance).
//Please use the corresponding web repository tool (e.g. github, bitbucket, etc)
//to file bugs, suggestions, pull requests; we will do our best to address them
//in a timely manner.
//
// SOFTWARE LICENSE AGREEMENT (BSD LICENSE):
//
//
//Copyright (c) 2015, Philippe Ludivig
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions are met:
//
//* Redistributions of source code must retain the above copyright notice, this
//  list of conditions and the following disclaimer.
//
//* Redistributions in binary form must reproduce the above copyright notice,
//  this list of conditions and the following disclaimer in the documentation
//  and/or other materials provided with the distribution.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//##########################################################################


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <dynamic_reconfigure/server.h>
#include <face_detection/face_det_dlibConfig.h>



#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>


#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

#include <iostream>
#include <queue>
#include <stdio.h>
#include <sstream>
#include <string>
#include <time.h>


using namespace std;
using namespace cv;
//using namespace dlib;

// OpenCV publishing windows
static const std::string OPENCV_WINDOW = "Image window";



//####################################################################
//#                                                                  #
//####################################################################
//###################### Face Detector Class #########################
//####################################################################
//#                                                                  #
//####################################################################
class FaceDetector
{
  ros::NodeHandle nh_;
  ros::NodeHandle n;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher faceCoord_pub;

  // required for the dynamic reconfigure server
  dynamic_reconfigure::Server<face_detection::face_det_dlibConfig> srv;
  dynamic_reconfigure::Server<face_detection::face_det_dlibConfig>::CallbackType f;

  int counter;
  int neighborsValue;
  float scaleValue;
  int minSize;
  int cascadeValue;
  float imgScale;
  int histOnOff;
  int blurFactor;
  int brightnessFactor;
  float contrastFactor;
  int debug;
  int button1;
  int flag;
  int colourValue;
  int inputSkipp;
  int maxSize;
  float totalTime;
  int windowOnOff;
  int pixelSwitch;
  int fpsWindowSize;
  int publish;

  string imageInput = "/camera/image_raw";
  string imageOutput = "/face_det/image_raw";

  struct timeval tinit_time;
  queue <timeval> fpsTimeQueue;

  char myflag;

  cv::CascadeClassifier face_cascade;
  cv::CascadeClassifier face_cascade_0;
  cv::CascadeClassifier face_cascade_1;
  cv::CascadeClassifier face_cascade_2;
  cv::CascadeClassifier face_cascade_3;
  cv::CascadeClassifier face_cascade_4;

  dlib::frontal_face_detector detector;

  std::vector<cv::Rect> faces;
  std::vector<cv::Rect> faces_fromDlib;

  std::vector<dlib::rectangle> faces_dlib;


  int gFps;
  int gCounter;
  Mat gray;
  std::vector<cv::Rect>::const_iterator i;


  // start and end times
  time_t start, end, timeZero, currentTime;
  ros::Time begin;

  // fps calculated using number of frames / seconds
  double fps;
  // frame counter
  int frameCounter;
  int totalFrameCounter;
  // floating point seconds elapsed since start
  double sec;
  int totalDetections;

private:
    //####################################################################
    //############# called every time theres a new image #################
    //####################################################################
    void newImageCallBack(const sensor_msgs::ImageConstPtr& msg)
    {
      // starts time calculations, one of the counters is being reset every once in a while
      struct timeval tstart, tend;
      gettimeofday(&tstart, NULL);
      fpsTimeQueue.push(tstart);


      if (totalFrameCounter == 0) {
          gettimeofday(&tinit_time, NULL);
          begin = ros::Time::now();
      }


      // retrieves the image from the camera driver
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }




      //####################################################################
      //######################## image preprocessing #######################
      //####################################################################
      //imgScale = 1.0/imgScaleValue;
      gCounter += 1;

      // change contrast: 0.5 = half  ; 2.0 = double
      cv_ptr->image.convertTo(gray, -1, contrastFactor, 0);

      Mat imgForDlib = cv_ptr->image.clone();
      resize(imgForDlib, imgForDlib, Size(), imgScale , imgScale);

      // create B&W image
      cvtColor( gray, gray, CV_BGR2GRAY );

      //equalize the histogram
      if(histOnOff == 1){
        equalizeHist( gray, gray );
      }

      //blur image by blurfactor
      if(blurFactor > 0){
        blur( gray, gray, Size( blurFactor, blurFactor) );
      }

      //scale image
      resize(gray, gray, Size(), imgScale , imgScale);



      //####################################################################
      //####################### detection part #############################
      //####################################################################

      // depending on the gFps setting, this part is only executed every couple of frames
      if(gCounter > gFps -1){
        gCounter = 0;

        //dlib detection
        // Detect faces dlib
        dlib::cv_image<dlib::bgr_pixel> cimg(imgForDlib);
        faces_dlib = detector(cimg);
      }

      faces.clear();

      for (unsigned i = 0; i != faces_dlib.size(); ++i) {
          //l = x
          //t = y
          //r = x+w
          //b = y+h


          Rect newFace((faces_dlib[i].left()), //x
                       (faces_dlib[i].top()), //y
                       (faces_dlib[i].right()-faces_dlib[i].left()), //width
                       (faces_dlib[i].bottom()-faces_dlib[i].top())  //height
                     ); // height

          faces.push_back(newFace);

      }


      //keep number of total detections
      totalDetections += faces.size();


      //print faces on top of image
      if(pixelSwitch == 0){
          cv_ptr->image = drawFaces(cv_ptr->image);
      } else {

          //blur faces
          Mat blurMyFace;
          for (i = faces.begin(); i != faces.end(); ++i) {
              Rect cropROI((i->x)/imgScale,(i->y)/imgScale,(i->width)/imgScale, (i->height)/imgScale);
              blurMyFace = cv_ptr->image(cropROI);
              blurMyFace = pixelate(blurMyFace,16);
              blurMyFace.copyTo(cv_ptr->image(cropROI));
          }
      }



      //Display Section, images will only displayed if option is selected
      if(debug != 0){
        if(debug == 1 || debug == 3 ){
          cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        }
        else if(debug == 2){
          for (i = faces.begin(); i != faces.end(); ++i) {
            //if(i->width <250){
              cv::rectangle(
                gray,
                cv::Point(i->x, i->y),
                cv::Point(i->x + i->width, i->y + i->height),
                CV_RGB(0, 255, 0),
                2);
            //}
          }
          cv::imshow(OPENCV_WINDOW, gray);
        }
      }



      //publishing
      if(publish != 0 ){
        // Output modified video stream

        if(publish == 1 || publish == 3){
            image_pub_.publish(cv_ptr->toImageMsg());
        }


        // ### publishing coordinates ###
        if(publish > 1){
          std_msgs::Int32MultiArray myMsg;
          myMsg.data.clear();
          // publish current fps rate
          myMsg.data.push_back(fps);
          // publish number of detected faces
          myMsg.data.push_back(faces.size());
          // width of the image
          myMsg.data.push_back(cv_ptr->image.cols);
          // height of the image
          myMsg.data.push_back(cv_ptr->image.rows);
          //for (i = faces.begin(); i != faces.end(); ++i) {
          for ( unsigned i = 0; i < faces.size(); i++) {
              myMsg.data.push_back(1);
              myMsg.data.push_back(1);
              myMsg.data.push_back(faces[i].x);
              myMsg.data.push_back(faces[i].y);
              myMsg.data.push_back(faces[i].width);
              myMsg.data.push_back(faces[i].height);
          }
          faceCoord_pub.publish(myMsg);
        }

      }



      //measure time in milisec
      long mtime, seconds, useconds;

      //measure fps since beginning
      gettimeofday(&tend, NULL);
      seconds  = tend.tv_sec  - tinit_time.tv_sec;
      useconds = tend.tv_usec - tinit_time.tv_usec;
      mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
      double fpsTotal =  float(totalFrameCounter) / (float(mtime)/1000);

      // measure fps for last 10 frames
      gettimeofday(&tend, NULL);
      seconds  = tend.tv_sec  - fpsTimeQueue.front().tv_sec;
      useconds = tend.tv_usec - fpsTimeQueue.front().tv_usec;
      mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
      fps =  float(10) / (float(mtime)/1000);

      fpsTimeQueue.pop();

      // print out averages
      totalTime = ( ros::Time::now() - begin).toSec();

      printf("time passed: %.2f  #  frames: %i  #  fps: %f  \n#  total number of detections: %i # FPS in last 100 frames : %.2f \n", totalTime, totalFrameCounter, fpsTotal, totalDetections, fps);

      totalFrameCounter += 1;
      cv::waitKey(3);
    }

    // #########################################
    // #### used for pixelising images #########
    // #########################################
    Mat pixelate(Mat myImage, int pixelizationRate){

      cv::Mat result = cv::Mat::zeros(myImage.size(), CV_8UC3);
      for (int i = 0; i < myImage.rows; i += pixelizationRate)
         {
             for (int j = 0; j < myImage.cols; j += pixelizationRate)
             {
                 cv::Rect rect = cv::Rect(j, i, pixelizationRate, pixelizationRate) &
                                 cv::Rect(0, 0, myImage.cols, myImage.rows);

                 cv::Mat sub_dst(result, rect);
                 sub_dst.setTo(cv::mean(myImage(rect)));

             }
         }

      return result;
    }


    //####################################################################
    //##################### drawFaces ####################################
    //####################################################################
    //takes the detected faces and daws them on top of an image
    cv::Mat drawFaces(cv::Mat myImage) {

        for (i = faces.begin(); i != faces.end(); ++i) {
            cv::rectangle(
              myImage,
              cv::Point((i->x)/imgScale, (i->y)/imgScale),
              cv::Point((i->x)/imgScale + (i->width)/imgScale, (i->y)/imgScale + (i->height)/imgScale),
              CV_RGB(50, 255 , 50),
              2);
        }

        if(debug ==3){

          // display fps
          string fpsText = "FPS: " + std::to_string((int)fps);
          cv::putText(myImage, fpsText, cv::Point(25,25), CV_FONT_NORMAL, 0.75, Scalar(255,50,50),1,1);
        }

        return myImage;
    }



public:

  //######################################################################
  //##################### constructor ####################################
  //######################################################################
  FaceDetector(String casc0, String casc1, String casc2, String casc3, String casc4)
    : it_(nh_)
  {
    inputSkipp = 1;


    // Subscrive to input video feed and publish output video feed "/camera/image_raw",
    image_sub_ = it_.subscribe(imageInput, inputSkipp, &FaceDetector::newImageCallBack, this);
    image_pub_ = it_.advertise(imageOutput, inputSkipp);

    //load dlib detector
    detector = dlib::get_frontal_face_detector();



    printf("OpenCV: %s \n", cv::getBuildInformation().c_str());

    counter = 0;
    gFps = 2;
    gCounter = gFps -1;
    neighborsValue = 2;
    scaleValue = 1.2;
    minSize = 13;
    maxSize = 250;
    cascadeValue = 2;
    imgScale = 0.5;
    histOnOff = 0;
    blurFactor = 0;
    brightnessFactor = 0;
    button1 = 0;
    frameCounter = 0;
    contrastFactor = 1.5;
    flag = 2;
    fps = -1;
    debug = 0;
    totalFrameCounter = 0;
    totalTime = 0;
    myflag = CV_HAAR_DO_CANNY_PRUNING;
    windowOnOff = 0;
    totalDetections = 0;
    pixelSwitch = 1;
    fpsWindowSize = 10;

    //init fps Window Queue
    for (int i = 0;i < (fpsWindowSize);i++) {
        struct timeval now;
        fpsTimeQueue.push(now);
    }



    faceCoord_pub = n.advertise<std_msgs::Int32MultiArray>("faceCoord", 1000);

    //setting up the dynamic reconfigure server
    f = boost::bind(&FaceDetector::callback, this, _1, _2);
    srv.setCallback(f);

    //setNumThreads(0);

    //generate windows
    if(debug != 0){
      cv::namedWindow(OPENCV_WINDOW);
    }

  }

  //####################################################################
  //##################### destroyer ####################################
  //####################################################################
  ~FaceDetector()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }


  //########################################################
  //########## reconfigure callback function ###############
  //########################################################
  void callback(face_detection::face_det_dlibConfig &config, uint32_t level)
  {

    ROS_INFO("Reconfigure request");


    if (config.displayed_Image == 0 && debug > 0) {
      cv::destroyWindow(OPENCV_WINDOW);
    }

    gFps = config.skipFrames;

    //scaleValue = config.scaleValue;


    imgScale = config.imgScale;
    histOnOff = config.histOnOff;
    blurFactor = config.blurFactor;
    brightnessFactor = config.brightnessFactor;
    contrastFactor = config.contrastFactor;
    debug = config.displayed_Image;
    inputSkipp = config.inputSkipp;
    pixelSwitch = config.pixelSwitch;
    publish = config.publish;



    if (imageInput != config.imageInput || inputSkipp != config.inputSkipp) {
      imageInput = config.imageInput;
      image_sub_ = it_.subscribe(imageInput, inputSkipp, &FaceDetector::newImageCallBack, this);
    }

    if (imageOutput != config.imageOutput || inputSkipp != config.inputSkipp) {
      imageOutput = config.imageOutput;
      image_pub_ = it_.advertise(imageOutput, inputSkipp);
    }

  }


  // check the reconfigure sever
  void callSrv()
  {
    srv.setCallback(f);
  }

};




//########################################################
//##################### Main #############################
//########################################################
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  printf("\n");
  printf("\n");
  printf("##############################################\n");
  printf("############ ROS Face Detection ##############\n");
  printf("##############################################\n");
  printf("\n");
  if(argc < 6){
    printf("Not Enough arguments, use one of the provided Roslaunch files\n");
    printf("\n");
    printf("Alternatively, arguments are needed as follows:\n");
    printf("01) Detection Cascade file 0\n");
    printf("02) Detection Cascade file 1\n");
    printf("03) Detection Cascade file 2\n");
    printf("04) Detection Cascade file 3\n");
    printf("05) Detection Cascade file 4\n");
    printf("\n");
    printf("\n");
    exit(0);
  }



  ros::init(argc, argv, "face_detection");


  ROS_INFO("Starting to spin...");

  FaceDetector faceDet(argv[1],argv[2],argv[3],argv[4],argv[5]);
  faceDet.callSrv();
  ros::spin();
  return 0;
}
