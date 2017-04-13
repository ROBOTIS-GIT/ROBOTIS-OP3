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

/* Author: Kayman Jung*/

#include <ros/ros.h>

#include "op3_camera_setting_tool/op3_camera_setting_tool.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "op3_camera_setting_tool");

  ros::NodeHandle nh;

  // get param to use
  nh.param("video_device", g_device_name, std::string("/dev/video0"));
  nh.param("camera_node_name", g_camera_node_name, std::string("/usb_cam_node"));

  // v4l param name, ros param name
  g_param_list["brightness"] = "brightness";
  g_param_list["contrast"] = "contrast";
  g_param_list["saturation"] = "saturation";
  g_param_list["sharpness"] = "sharpness";

  g_param_list["focus_auto"] = "autofocus";
  g_param_list["focus_absolute"] = "focus";

  g_param_list["exposure_auto"] = "autoexposure";
  g_param_list["exposure_absolute"] = "exposure";
  g_param_list["gain"] = "gain";

  g_param_list["white_balance_temperature_auto"] = "auto_white_balance";
  g_param_list["white_balance_temperature"] = "white_balance";

  // get param to set camera
  getROSParam();

  ros::Subscriber camera_param_sub = nh.subscribe("/op3_camera/set_param", 1, &setCameraParameterCallback);
  ros::Subscriber camera_params_sub = nh.subscribe("/op3_camera/set_params", 1, &setCameraParametersCallback);

  ROS_INFO("start camera setting tool!");

  ros::spin();

  return 0;
}

void setCameraParameterCallback(const op3_camera_setting_tool::V4lParameter::ConstPtr &msg)
{
  setV4lParameter(msg->name, msg->value);
}

void setCameraParametersCallback(const op3_camera_setting_tool::V4lParameters::ConstPtr &msg)
{
  for (int ix = 0; ix < msg->video_parameter.size(); ix++)
    setV4lParameter(msg->video_parameter[ix].name, msg->video_parameter[ix].value);
}

void setV4lParameter(const std::string& param, const std::string& value)
{
  // build the command
  std::stringstream ss;
  ss << "v4l2-ctl --device=" << g_device_name << " -c " << param << "=" << value << " 2>&1";
  std::string cmd = ss.str();

  setV4lParameter(cmd);

  setROSParam(param, boost::lexical_cast<int>(value));
}

void setV4lParameter(const std::string& param, const int& value)
{
  // build the command
  std::stringstream ss;
  ss << "v4l2-ctl --device=" << g_device_name << " -c " << param << "=" << value << " 2>&1";
  std::string cmd = ss.str();

  ROS_INFO("%s", cmd.c_str());
  setV4lParameter(cmd);

  setROSParam(param, value);
}

void setV4lParameter(const std::string& cmd)
{
  // capture the output
  std::string output;
  int buffer_size = 256;
  char buffer[buffer_size];
  FILE *stream = popen(cmd.c_str(), "r");
  if (stream)
  {
    while (!feof(stream))
      if (fgets(buffer, buffer_size, stream) != NULL)
        output.append(buffer);
    pclose(stream);
    // any output should be an error
    if (output.length() > 0)
      ROS_WARN("%s", output.c_str());
  }
  else
    ROS_WARN("usb_cam_node could not run '%s'", cmd.c_str());

}

void getROSParam()
{
  ros::NodeHandle nh;

  int param_int_value;
  bool param_bool_value;
  bool exist_param;
  std::string prefix = (g_camera_node_name == "") ? g_camera_node_name : g_camera_node_name + "/";

  exist_param = nh.getParam(prefix + "brightness", param_int_value);  //0-255
  if (exist_param == true)
    setV4lParameter("brightness", param_int_value);

  exist_param = nh.getParam(prefix + "contrast", param_int_value);  //0-255
  if (exist_param == true)
    setV4lParameter("contrast", param_int_value);

  exist_param = nh.getParam(prefix + "saturation", param_int_value);  //0-255
  if (exist_param == true)
    setV4lParameter("saturation", param_int_value);

  exist_param = nh.getParam(prefix + "sharpness", param_int_value);  //0-255
  if (exist_param == true)
    setV4lParameter("sharpness", param_int_value);

  exist_param = nh.getParam(prefix + "gain", param_int_value);  //0-255
  if (exist_param == true)
    setV4lParameter("gain", param_int_value);

  // focus
  exist_param = nh.getParam(prefix + "autofocus", param_bool_value);
  if (exist_param == true)
  {
    setV4lParameter("focus_auto", param_bool_value);

    if (param_bool_value == false)
    {
      int focus_absolute;
      nh.param(prefix + "focus", focus_absolute, -1);  //0-255, -1 "leave alone"
      setV4lParameter("focus_absolute", focus_absolute);
    }
  }

  // exposure
  exist_param = nh.getParam(prefix + "autoexposure", param_bool_value);
  if (exist_param == true)
  {
    // turn down exposure control (from max of 3)
    setV4lParameter("exposure_auto", 1);

    if (param_bool_value == false)
    {
      int exposure;
      nh.param(prefix + "exposure", exposure, 100);
      setV4lParameter("exposure_absolute", exposure);
    }
  }

  // white balance
  exist_param = nh.getParam(prefix + "auto_white_balance", param_bool_value);
  if (exist_param == true)
  {
    setV4lParameter("white_balance_temperature_auto", param_bool_value);

    if (param_bool_value == false)
    {
      int white_balance_temperature;
      nh.param(prefix + "white_balance", white_balance_temperature, 4000);
      setV4lParameter("white_balance_temperature", white_balance_temperature);
    }
  }
}

void setROSParam(const std::string& param, const int& value)
{
  std::map<std::string, std::string>::iterator param_iter;

  param_iter = g_param_list.find(param);

  if (param_iter == g_param_list.end())
    return;

  ros::NodeHandle nh;
  std::string prefix = (g_camera_node_name == "") ? g_camera_node_name : g_camera_node_name + "/";

  if(param == "exposure_auto")
  {
    nh.setParam(prefix + param_iter->second, (value == 1) ? false : true);
    ROS_INFO("Set param to parameter server : %s = %s", param.c_str(), (value == 1) ? "false" : "true");
  }
  else
  {
    nh.setParam(prefix + param_iter->second, value);
    ROS_INFO("Set param to parameter server : %s = %d", param.c_str(), value);
  }
}
