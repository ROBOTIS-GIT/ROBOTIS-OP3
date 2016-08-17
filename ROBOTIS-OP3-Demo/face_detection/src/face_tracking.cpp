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
#include <opencv2/video/tracking.hpp>
#include <dynamic_reconfigure/server.h>
#include <face_detection/face_trackConfig.h>

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
  dynamic_reconfigure::Server<face_detection::face_trackConfig> srv;
  dynamic_reconfigure::Server<face_detection::face_trackConfig>::CallbackType f;


  // image processing variables
  int brightnessFactor;
  float contrastFactor;
  int blurFactor;
  int histOnOff;
  float imgScale;

  // cascade detector variables
  int cascadeValue;
  int minSize;
  int maxSize;
  char myflag;
  float scaleValue;
  int neighborsValue;

  // tracking variables
  int maxTrackingNum;
  int trackSearchWinSize;
  int maxNumFeatures;
  int initialDetectionNum;

  // other variables
  int debug;
  int pixelSwitch;
  int inputSkipp;
  int publish;
  int fpsWindowSize;
  int IDcounter;
  int counter;
  int gFps;
  int gCounter;
  int totalDetections;

  bool print_log;

  // publish subscribe variables
  string imageInput = "/camera/image_raw";
  string imageOutput = "/face_det/image_raw";


  // variables containing cascades
  cv::CascadeClassifier face_cascade;
  cv::CascadeClassifier face_cascade_0;
  cv::CascadeClassifier face_cascade_1;
  cv::CascadeClassifier face_cascade_2;
  cv::CascadeClassifier face_cascade_3;
  cv::CascadeClassifier face_cascade_4;

  // containers for face details
  std::vector<cv::Rect> faces;
  std::vector<int> lastSeen;
  std::vector<int> faceID;
  std::vector<int> detectionLength;

  std::vector<cv::Rect>::const_iterator i;


  // Image Mats used for tracking
  Mat previousFrame;
  Mat inputImage;
  Mat gray;

  // timing variables
  ros::Time begin;
  int totalFrameCounter;
  double fps;
  double sec;
  float totalTime;
  struct timeval tinit_time;
  queue <timeval> fpsTimeQueue;

private:

    //####################################################################
    //############# called every time theres a new image #################
    //####################################################################
    void newImageCallBack(const sensor_msgs::ImageConstPtr& msg)
    {

      // starts time calculations
      struct timeval tstart, tend;
      gettimeofday(&tstart, NULL);
      fpsTimeQueue.push(tstart);


      // retrieves the image from the camera driver
      // ###############################################
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


      // run only on the first frame
      if (totalFrameCounter == 0) {
          gettimeofday(&tinit_time, NULL);
          begin = ros::Time::now();
          previousFrame = cv_ptr->image;
          cvtColor( previousFrame, previousFrame, CV_BGR2GRAY );
      }

      //####################################################################
      //######################## image preprocessing #######################
      //####################################################################
      gCounter += 1;

      // change contrast: 0.5 = half  ; 2.0 = double
      cv_ptr->image.convertTo(gray, -1, contrastFactor, 0);

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

      // make full copy of the image (no pointer)
      inputImage = gray.clone();

      //scale image
      resize(gray, gray, Size(), imgScale , imgScale);



      //####################################################################
      //####################### detection part #############################
      //####################################################################
      // depending on the gFps setting, this part is only executed every couple of frames

      std::vector<cv::Rect> newFaces;
      if(gCounter > gFps -1){

        face_cascade.detectMultiScale(
          gray,                       // input image (grayscale)
          newFaces,                      // output variable containing face rectangle
          scaleValue,                 // scale factor
          neighborsValue,             // minimum neighbors
          0|myflag,                   // flags
          cv::Size(minSize*imgScale, minSize*imgScale), // minimum size
          cv::Size(maxSize*imgScale, maxSize*imgScale)  // minimum size
        );

        //printf("min %d max %d\n",minSize, maxSize );
        //printf("min_scale %f max_scale %f\n",minSize*imgScale, maxSize*imgScale );

        if(imgScale != 1){
          for(unsigned i = 0; i < newFaces.size(); ++i) {
              newFaces[i].x = newFaces[i].x/imgScale;
              newFaces[i].y = newFaces[i].y/imgScale;
              newFaces[i].width = newFaces[i].width/imgScale;
              newFaces[i].height = newFaces[i].height/imgScale;
          }
        }
      }



      //####################################################################
      //####################### tracking part ##############################
      //####################################################################
      // tracking the featres between the previeous and the current frame
      Mat croppedImage;
      int mx;
      int my;
      int mh;
      int mw;


      // iterates trough all faces from previous cycle
      for (unsigned i = 0; i < faces.size(); i++) {
        mx = faces[i].x;
        my = faces[i].y;
        mh = faces[i].height;
        mw = faces[i].width;

        // averaging variables
        double mvRateX = 0.0;
        double mvRateY = 0.0;

        std::vector<cv::Point2f> features_prev, features_next;
        std::vector<uchar> status;
        std::vector<float> err;

        // crop image and find features within the cropping rectangle
        // ###############################################################
        croppedImage = inputImage(Rect(mx,my, mw,mh));
        cv::goodFeaturesToTrack(croppedImage, // the image
          features_prev,   // the output detected features
          maxNumFeatures,  // the maximum number of features
          0.4,     // quality level
          2     // min distance between two features
        );


        // correct coordinates for cropping operation
        for (unsigned j = 0; j < features_prev.size(); j++) {
            features_prev[j].x = features_prev[j].x + mx;
            features_prev[j].y = features_prev[j].y + my;
            if(pixelSwitch == 0 && debug ==3){
                cv::circle(cv_ptr->image, cv::Point(features_prev[j].x , features_prev[j].y), 1, CV_RGB(255,0,0),CV_FILLED);
            }
        }



        // Optical Flow
        // #############################################################
        cv::Size winSize(trackSearchWinSize,trackSearchWinSize);
        if(features_prev.size() != 0){
            cv::calcOpticalFlowPyrLK(
              previousFrame, inputImage, // 2 consecutive images
              features_prev, // input point positions in first im
              features_next, // output point positions in the 2nd
              status,    // tracking success
              err,      // tracking error
              winSize
            );
        }



        // calc error rate
        // #######################################################
        for (unsigned j = 0; j < features_next.size(); j++) {
            if(pixelSwitch == 0 && debug ==3){
                cv::circle(cv_ptr->image, cv::Point(features_next[j].x , features_next[j].y), 1, CV_RGB(255,255,255),CV_FILLED);
            }
            mvRateX += features_next[j].x - features_prev[j].x;
            mvRateY += features_next[j].y - features_prev[j].y;

        }
        mvRateX = mvRateX /features_next.size();
        mvRateY = mvRateY /features_next.size();

        if(pixelSwitch == 0 && debug ==3){
            cv::circle(cv_ptr->image, cv::Point(mx + mw/2, my + mh/2), 10, CV_RGB(0,255,0));
            cv::circle(cv_ptr->image, cv::Point(mx + mw/2 + mvRateX, my + mh/2 +mvRateY), 10, CV_RGB(255,0,0));
        }


        // update error rate
        faces[i].x = faces[i].x + mvRateX;
        faces[i].y = faces[i].y + mvRateY;


        // when moving outside of the image, remove face from list
        // ##############################################################
        if(faces[i].x < 0 || faces[i].y < 0 || (faces[i].x + faces[i].width) > (cv_ptr->image.cols) || (faces[i].y+ faces[i].height)  > (cv_ptr->image.rows)){
            faces.erase (faces.begin()+i);
            lastSeen.erase (lastSeen.begin()+i);
            faceID.erase (faceID.begin()+i);
            detectionLength.erase (detectionLength.begin()+i);
            i--;
        }

      }


      //####################################################################
      //################### find intersection  #############################
      //####################################################################
      // here we compare the tracked faces against the newly detected faces:
      // if we have an intersection, the tracked faces is udated with a new detection
      // if we have no intersection, the new detection is added to the current faces
      if(gCounter > gFps -1){
          gCounter = 0;

          for (unsigned i = 0; i < newFaces.size(); i++) {
              int duplicatedFaceDetection = 0;
              for ( unsigned j = 0; j < faces.size(); j++) {
                  Rect interSection = faces[j] & newFaces[i];

                  // all values == 0 means no intersection
                  if (interSection.width != 0){
                        // we have intersection
                        duplicatedFaceDetection = 1;
                        faces[j] = newFaces[i];
                        lastSeen[j] = maxTrackingNum;
                        break;
                  }

              }

              // if we have no intersection with any face from the list:
              // we know that a new face has been detected
              if(duplicatedFaceDetection == 0){
                  faces.push_back(newFaces[i]);
                  lastSeen.push_back(initialDetectionNum);
                  faceID.push_back(IDcounter);
                  IDcounter++;
                  detectionLength.push_back(1);

              }
          }


      //####################################################################
      //################### count lastSeen #################################
      //####################################################################
      // update lastSeen list, and remove faces that havent been seen in a while
          for (unsigned i = 0; i < faces.size(); i++) {
              if(lastSeen[i] == 1){
                  // remove face
                  faces.erase (faces.begin()+i);
                  lastSeen.erase (lastSeen.begin()+i);
                  faceID.erase (faceID.begin()+i);
                  detectionLength.erase (detectionLength.begin()+i);
                  i--;
              } else {
                  // otherwise update face
                  lastSeen[i] = lastSeen[i] -1;
                  detectionLength[i] = detectionLength[i] + 1;
              }
          }
      }




      //####################################################################
      //############ last part (printing drawing publishing) ###############
      //####################################################################

      // copy the current image to be used in the next cycle for tracking
      previousFrame = inputImage.clone();

      // keep track of number of total detections
      totalDetections += faces.size();

      // print faces on top of image
      if(pixelSwitch == 0){
          cv_ptr->image = drawFaces(cv_ptr->image);
      } else {
          // blur faces
          Mat blurMyFace;
          for (i = faces.begin(); i != faces.end(); ++i) {
              Rect cropROI((i->x),(i->y),(i->width), (i->height));
              blurMyFace = cv_ptr->image(cropROI);
              blurMyFace = pixelate(blurMyFace,16);
              blurMyFace.copyTo(cv_ptr->image(cropROI));
          }
      }


      // Display Section, images will only displayed if option is selected
      // debug 0 = no display
      if(debug != 0){
        if(debug == 1 || debug == 3){
          cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        }
        // debug 2 = display image used for detection (gray)
        else if(debug == 2){
          for (i = faces.begin(); i != faces.end(); ++i) {
              cv::rectangle(
                gray,
                cv::Point((i->x)*imgScale, (i->y)*imgScale),
                cv::Point((i->x)*imgScale + (i->width)*imgScale, (i->y)*imgScale + (i->height)*imgScale),
                CV_RGB(0, 255, 0),
                2);
          }
          cv::imshow(OPENCV_WINDOW, gray);
        }
      }


      // publishing
      // #############################
      if(publish != 0 ){

          // Output modified video stream
          if(publish == 1 || publish == 3){
              image_pub_.publish(cv_ptr->toImageMsg());
          }


          // publishing coordinates - inside array
          // #####################################################
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

              // publish details for each face
              for ( unsigned i = 0; i < faces.size(); i++) {
                  myMsg.data.push_back(faceID[i]);
                  myMsg.data.push_back(detectionLength[i]);
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

      totalTime = ( ros::Time::now() - begin).toSec();

      // print out monitoring values
      if(print_log)
        printf("time passed: %.2f  #  frames: %i  #  fps: %f  \n#  total number of detections: %i # FPS in last 10 frames : %.2f \n", totalTime, totalFrameCounter, fpsTotal, totalDetections, fps);

      totalFrameCounter += 1;
      cv::waitKey(3);



    }

    // #########################################
    // #### used for pixelising images #########
    // #########################################
    // takes input image (Mat) and returns a pixelisated
    // image at the rate "pixelizationRate", higer value:
    // stronger pixelisation
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
    //takes the detected faces and daws them on top of the input image
    cv::Mat drawFaces(cv::Mat myImage) {


        if(debug ==3){

            //draw min/max detection box size
            cv::rectangle(
              myImage,
              cv::Point(0, 0),
              cv::Point(minSize, minSize),
              CV_RGB(255, 50 , 50),
              2);
            cv::rectangle(
              myImage,
              cv::Point(0, 0),
              cv::Point(maxSize, maxSize),
              CV_RGB(255, 50 , 50),
              2);
            cv::rectangle(
              myImage,
              cv::Point(0, maxSize),
              cv::Point(maxSize+1, maxSize+20),
              CV_RGB(255, 50 , 50),
              CV_FILLED);
            cv::putText(myImage, "min/max size", cv::Point(5, maxSize+15), CV_FONT_NORMAL, 0.5, Scalar(255,255,255),1,1);

            //draw fps numbers
            string fpsText = "FPS: " + std::to_string((int)fps);
            cv::putText(myImage, fpsText, cv::Point(25,25), CV_FONT_NORMAL, 0.75, Scalar(255,50,50),1,1);

        }



        for (unsigned i = 0; i < faces.size(); ++i) {
            int boxSize = 25;
            if(faceID[i] > 99){boxSize = 36;}



            if(lastSeen[i] == (maxTrackingNum -1) ){

                // green = newly detected
                cv::rectangle(
                  myImage,
                  cv::Point((faces[i].x), (faces[i].y)),
                  cv::Point((faces[i].x) + (faces[i].width), (faces[i].y) + (faces[i].height)),
                  CV_RGB(50, 255 , 50),
                  2);

                cv::rectangle(
                  myImage,
                  cv::Point((faces[i].x)-1, (faces[i].y+faces[i].height)),
                  cv::Point((faces[i].x)+boxSize, (faces[i].y+faces[i].height)+20),
                  CV_RGB(50, 255 , 50),
                  CV_FILLED);

            } else if(lastSeen[i] != 1) {

                // blue = tracked from previous detection
                cv::rectangle(
                  myImage,
                  cv::Point((faces[i].x), (faces[i].y)),
                  cv::Point((faces[i].x) + (faces[i].width), (faces[i].y) + (faces[i].height)),
                  CV_RGB(50, 50 , 255),
                  2);
                cv::rectangle(
                  myImage,
                  cv::Point((faces[i].x)-1, (faces[i].y+faces[i].height)),
                  cv::Point((faces[i].x)+boxSize, (faces[i].y+faces[i].height)+20),
                  CV_RGB(50, 50 , 255),
                  CV_FILLED);

            } else {

                // red = about to disappear
                cv::rectangle(
                  myImage,
                  cv::Point((faces[i].x), (faces[i].y)),
                  cv::Point((faces[i].x) + (faces[i].width), (faces[i].y) + (faces[i].height)),
                  CV_RGB(255, 50 , 50),
                  2);

                cv::rectangle(
                  myImage,
                  cv::Point((faces[i].x)-1, (faces[i].y+faces[i].height)),
                  cv::Point((faces[i].x)+boxSize, (faces[i].y+faces[i].height)+20),
                  CV_RGB(255, 50 , 50),
                  CV_FILLED);
            }


            // print the ID of the face
            cv::putText(myImage, std::to_string(faceID[i]), cv::Point(faces[i].x+1,faces[i].y+faces[i].height+15), CV_FONT_NORMAL, 0.5, Scalar(255,255,255),1,1);



        }


        return myImage;
    }


public:



  //######################################################################
  //##################### constructor ####################################
  //######################################################################
  // inputs equal to the names and directories of the different input
  // CascadeClassifier (5 needed)
  FaceDetector(String casc0, String casc1, String casc2, String casc3, String casc4)
    : it_(nh_)
  {


    // ###############################################
    // ########### initializing variables ############
    // ###############################################

    // image processing variables
    imgScale = 1.0;
    histOnOff = 0;
    blurFactor = 0;
    brightnessFactor = 0;
    contrastFactor = 1.5;

    // cascade detector variables
    neighborsValue = 2;
    scaleValue = 1.2;
    minSize = 13;
    maxSize = 250;
    cascadeValue = 2;
    myflag = CV_HAAR_DO_CANNY_PRUNING;

    // tracking variables
    trackSearchWinSize = 100;
    initialDetectionNum = 4;
    maxNumFeatures = 15;
    maxTrackingNum = 60;


    // counting variables
    counter = 0;
    gCounter = gFps -1;
    fps = -1;
    totalFrameCounter = 0;
    totalTime = 0;
    totalDetections = 0;
    IDcounter = 1;
    fpsWindowSize = 10;


    // other variables

    // determines which interface is shown
    debug = 0;

    // pixelise faces ON/OFF
    pixelSwitch = 1;

    // determines what is published
    publish = 1;

    // skips images on detection
    gFps = 2;





    // defines if images are skipped when not all of them can be processed in time;
    inputSkipp = 1;

    // Subscribing to the input images.
    image_sub_ = it_.subscribe(imageInput, inputSkipp, &FaceDetector::newImageCallBack, this);

    // Publishing the output images.
    image_pub_ = it_.advertise(imageOutput, inputSkipp);


    //loads in the different cascade detection files
    printf("################\n" );
    if (face_cascade_0.load(casc0) == false) {
      printf("cascade.load_0() failed...\n");
      printf("The missing cascade file is /include/face_detection/HaarCascades/haarcascade_frontalface_alt.xml\n");
      exit(0);
      }
    if (face_cascade_1.load(casc1) == false) {
      printf("cascade.load_1() failed...\n");
      printf("The missing cascade file is /include/face_detection/HaarCascades/haarcascade_frontalface_alt2.xml\n");
      exit(0);
      }
    if (face_cascade_2.load(casc2) == false) {
      printf("cascade.load_2() failed...\n");
      printf("The missing cascade file is /include/face_detection/HaarCascades/haarcascade_frontalface_alt_tree.xml\n");
      exit(0);
      }
    if (face_cascade_3.load(casc3) == false) {
      printf("cascade.load_3() failed...\n");
      printf("The missing cascade file is /include/face_detection/HaarCascades/haarcascade_frontalface_default.xml\n");
      exit(0);
      }
    if (face_cascade_4.load(casc4) == false) {
      printf("cascade.load_4() failed...\n");
      printf("The missing cascade file is /include/face_detection/lbpCascades/lbpcascade_frontalface.xml\n");
      exit(0);
      }

    // prints out OpenCV build information for debugging
    printf("OpenCV: %s \n", cv::getBuildInformation().c_str());


    //init fps Window Queue
    for (int i = 0;i < (fpsWindowSize);i++) {
        struct timeval now;
        fpsTimeQueue.push(now);
    }


    // should you need to force the program to a specific number of threads
    // setNumThreads(0);


    //setting up the dynamic reconfigure server
    f = boost::bind(&FaceDetector::callback, this, _1, _2);
    srv.setCallback(f);

    // publishing face coordinates
    faceCoord_pub = n.advertise<std_msgs::Int32MultiArray>("faceCoord", 1000);

    //generate OpenCV window
    if(debug != 0){
      cv::namedWindow(OPENCV_WINDOW);
    }

    print_log = false;
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
  // function communicates with the ROS rqt reconfigure interface
  void callback(face_detection::face_trackConfig &config, uint32_t level)
  {

    ROS_INFO("Reconfigure request");

    // destroy opencv window switch from display to no display
    if (config.displayed_Image == 0 && debug > 0) {
      cv::destroyWindow(OPENCV_WINDOW);
    }

    // cascade detector variables
    neighborsValue = config.neighborsValue;
    scaleValue = config.scaleValue;
    minSize = config.minSize;
    maxSize = config.maxSize;
    cascadeValue = config.cascadeValue;

    // image processing variables
    imgScale = config.imgScale;
    histOnOff = config.histOnOff;
    blurFactor = config.blurFactor;
    brightnessFactor = config.brightnessFactor;
    contrastFactor = config.contrastFactor;

    // tracking variables
    maxNumFeatures = config.maxNumFeatures;
    maxTrackingNum = config.maxTrackingNum;
    initialDetectionNum = config.initialDetectionNum;
    trackSearchWinSize = config.trackSearchWinSize;


    // other variables
    // #################################

    // determines which interface is shown
    debug = config.displayed_Image;

    // skips input images
    inputSkipp = config.inputSkipp;

    // pixelise faces ON/OFF
    pixelSwitch = config.pixelSwitch;

    // determines what is published
    publish = config.publish;

    // skips images on detection
    gFps = config.skipFrames;



    // selecting the correct flag for the
    // ######################################
    switch(config.myflag){
      case 0 :
        myflag = CV_HAAR_SCALE_IMAGE;
        break;
      case 1 :
        myflag = CV_HAAR_FIND_BIGGEST_OBJECT;
        break;
      case 2 :
        myflag = CV_HAAR_DO_CANNY_PRUNING;
        break;
      case 3 :
        myflag = CV_HAAR_DO_ROUGH_SEARCH;
        break;
      default:
        myflag = CV_HAAR_SCALE_IMAGE;
        break;
      }

    // selecting the correct cascade for the
    // ######################################
    switch (cascadeValue) {
      case 0:
        face_cascade = face_cascade_0;
        break;
      case 1:
        face_cascade = face_cascade_1;
        break;
      case 2:
        face_cascade = face_cascade_2;
        break;
      case 3:
        face_cascade = face_cascade_3;
        break;
      case 4:
        face_cascade = face_cascade_4;
        break;
      default:
        face_cascade = face_cascade_0;
        break;
    }


    // changes the location of the input images
    // ######################################
    if (imageInput != config.imageInput || inputSkipp != config.inputSkipp) {
      imageInput = config.imageInput;
      image_sub_ = it_.subscribe(imageInput, inputSkipp, &FaceDetector::newImageCallBack, this);
    }

    // changes the publication location
    // ######################################
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

  void setPrint(bool print)
  {
    print_log = print;
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



  ros::init(argc, argv, "face_tracking");

  ROS_INFO("Starting to spin...");

  FaceDetector faceDet(argv[1],argv[2],argv[3],argv[4],argv[5]);

  if(argc == 7 && argv[6] == "print")
    faceDet.setPrint(true);

  faceDet.callSrv();
  ros::spin();
  return 0;
}
