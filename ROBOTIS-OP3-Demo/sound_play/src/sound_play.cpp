/*
 * sound_play.cpp
 *
 *  Created on: 2016. 3. 24.
 *      Author: zerom
 */

#include <signal.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

pid_t play_pid = -1;
std::string sound_file_path = "";

void play_sound_callback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "")
  {
    if (play_pid != -1)
      kill(play_pid, SIGKILL);

    return;
  }

  if (play_pid != -1)
    kill(play_pid, SIGKILL);

  play_pid = fork();

  switch (play_pid)
  {
    case -1:
      fprintf(stderr, "Fork Failed!! \n");
      break;
    case 0:
      execl("/usr/bin/mpg321", "mpg321", (sound_file_path + msg->data).c_str(), "-q", (char*) 0);
      break;
    default:
      break;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sound_play");
  ros::NodeHandle _nh;

  sound_file_path = _nh.param<std::string>("sound_file_path", "");
  if (sound_file_path != "" && sound_file_path.compare(sound_file_path.size() - 1, 1, "/") != 0)
    sound_file_path += "/";

  ros::Subscriber _play_mp3_sub = _nh.subscribe("/play_sound_file", 10, &play_sound_callback);

  ros::spin();
  return 0;
}

