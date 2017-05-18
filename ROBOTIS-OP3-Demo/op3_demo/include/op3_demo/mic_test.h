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

#ifndef MIC_TEST_H_
#define MIC_TEST_H_

#include <signal.h>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>

#include "op3_demo/op_demo.h"

#include "robotis_controller_msgs/SyncWriteItem.h"

namespace robotis_op
{

class MicTest : public OPDemo
{
 public:
  enum Mic_Test_Status
  {
    Ready = 0,
    AnnounceRecording = 1,
    MicRecording = 2,
    PlayingSound = 3,
    DeleteTempFile = 4,
    DemoCount = 5
  };

  MicTest();
  ~MicTest();

  void setDemoEnable();
  void setDemoDisable();

 protected:
  const int SPIN_RATE;

  void processThread();
  void callbackThread();

  void process();

  void announceTest();
  void recordSound(int recording_time);
  void recordSound();
  void playTestSound(const std::string &path);
  void playSound(const std::string &file_path);
  void deleteSoundFile(const std::string &file_path);

  void startTimer(double wait_time);
  void finishTimer();

  void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);

  std::string recording_file_name_;
  std::string default_mp3_path_;

  ros::Publisher play_sound_pub_;
  ros::Subscriber buttuon_sub_;

  ros::Time start_time_;
  double wait_time_;
  bool is_wait_;
  int record_pid_;
  int play_pid_;
  int test_status_;
};

} /* namespace robotis_op */

#endif /* MIC_TEST_H_ */
