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

/* Author: kayman Jung, Jay Song */

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
