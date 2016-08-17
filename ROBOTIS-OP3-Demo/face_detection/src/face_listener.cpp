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
// This node is a guide on how to implement the face detection data which is
// being published by the face_detection and face_tracking nodes. The data is
// published as follows:

// 0: Detection Speed in Frames Per Second
// 1: Number of faces detected in the Current Frame
// 2: Image Size X
// 3: Image Size Y

// 4: First Face ID (This feature does only work with face_tracking)
// 5: First Face Detection Length (This feature does only work with face_tracking)
// 6: First Face X coordinate (top left Corner)
// 7: First Face Y coordinate (top left Corner)
// 8: First Face Width
// 9: First Face Heigth

// 4: Second Face ID (This feature does only work with face_tracking)
// 5: Second Face Detection Length (This feature does only work with face_tracking)
// 6: Second Face X coordinate (top left Corner)
// 7: Second Face Y coordinate (top left Corner)
// 8: Second Face Width
// 9: Second Face Heigth

// Third Face

// Fourth Face

// ...

#include "ros/ros.h"

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"


int Arr[200];


void chatterCallback(const std_msgs::Int32MultiArray::ConstPtr& myMsg)
{
  //ROS_INFO("I heard: [%d]", myMsg->data.c_str());


  int i = 0;

  //the data is being stored inside an array
  for(std::vector<int>::const_iterator it = myMsg->data.begin(); it != myMsg->data.end(); ++it)
  {
    Arr[i] = *it;
    i++;
  }

  int counter = 4;
  printf("### Face_Data: fps[%d] numFaces[%d] ImgX[%d] ImgY[%d] ###########\n", Arr[0], Arr[1], Arr[2], Arr[3]);
  for (int i = 0; i < Arr[1]; i++) {
      printf("Face: %d # detected for [%d] # X[%d] Y[%d]\n", Arr[counter], Arr[counter+1], Arr[counter+2], Arr[counter+3]);
      counter += 6;
  }


  //ROS_INFO("I heard: ");
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("faceCoord", 1000, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
