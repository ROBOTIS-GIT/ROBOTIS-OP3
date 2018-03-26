/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Kayman, Jay Song */

#ifndef ACTION_FILE_DEFINE_H_
#define ACTION_FILE_DEFINE_H_

namespace robotis_op
{
namespace action_file_define
{

const int MAXNUM_PAGE   = 256;
const int MAXNUM_STEP   = 7;
const int MAXNUM_NAME   = 13;
const int MAXNUM_JOINTS = 31;

const int SPEED_BASE_SCHEDULE = 0;
const int TIME_BASE_SCHEDULE  = 0x0a;

const int INVALID_BIT_MASK    = 0x4000;
const int TORQUE_OFF_BIT_MASK = 0x2000;

typedef struct // Header Structure (total 64unsigned char)
{
  unsigned char name[MAXNUM_NAME+1];  // Name             0~13
  unsigned char reserved1;            // Reserved1        14
  unsigned char repeat;               // Repeat count     15
  unsigned char schedule;             // schedule         16
  unsigned char reserved2[3];         // reserved2        17~19
  unsigned char stepnum;              // Number of step   20
  unsigned char reserved3;            // reserved3        21
  unsigned char speed;                // Speed            22
  unsigned char reserved4;            // reserved4        23
  unsigned char accel;                // Acceleration time 24
  unsigned char next;                 // Link to next     25
  unsigned char exit;                 // Link to exit     26
  unsigned char reserved5[4];         // reserved5        27~30
  unsigned char checksum;             // checksum         31
  unsigned char pgain[MAXNUM_JOINTS]; // pgain            32~62
  unsigned char reserved6;            // reserved6        63
} PageHeader;

typedef struct // Step Structure (total 64unsigned char)
{
  unsigned short position[MAXNUM_JOINTS]; // Joint position   0~61
  unsigned char pause;                    // Pause time       62
  unsigned char time;                     // Time             63
} Step;

typedef struct // Page Structure (total 512unsigned char)
{
  PageHeader header;             // Page header  0~63
  Step       step[MAXNUM_STEP];	 // Page step    61~501
} Page;

}
}

#endif /* ACTION_FILE_DEFINE_H_ */
