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


/* Author: Jay Song */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <term.h>
#include <ncurses.h>
#include <libgen.h>
#include <signal.h>

#include <ros/ros.h>
#include "op3_action_editor/cmd_process.h"

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

void sighandler(int sig)
{
    struct termios term;
    tcgetattr( STDIN_FILENO, &term );
    term.c_lflag |= ICANON | ECHO;
    tcsetattr( STDIN_FILENO, TCSANOW, &term );

    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "OP3 Test Action Editor");
    ros::NodeHandle _nh;

    std::string         _offset_file    = _nh.param<std::string>("offset_table", "");
    std::string         _robot_file     = _nh.param<std::string>("robot_file_path", "");

    std::string         _init_file      = _nh.param<std::string>("init_file_path", "");


    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);



    int ch;


    if(InitializeActionEditor(_robot_file, _init_file, _offset_file) == false) {
    	ROS_ERROR("Failed to Initialize");
    	return 0;
    }



    DrawIntro();

    while(1)
    {
        ch = _getch();

        if(ch == 0x1b)
        {
            ch = _getch();
            if(ch == 0x5b)
            {
                ch = _getch();
                if(ch == 0x41)      // Up arrow key
                    MoveUpCursor();
                else if(ch == 0x42) // Down arrow key
                    MoveDownCursor();
                else if(ch == 0x44) // Left arrow key
                    MoveLeftCursor();
                else if(ch == 0x43) // Right arrow key
                    MoveRightCursor();
            }
        }
        else if( ch == '[' )
            UpDownValue( -1);
        else if( ch == ']' )
            UpDownValue( 1);
        else if( ch == '{' )
            UpDownValue( -10);
        else if( ch == '}' )
            UpDownValue( 10);
        else if( ch == ' ' )
            ToggleTorque();
//        else if( ch == ';') {
//        	UnFoldRightHand();
//        }
//        else if( ch == '\'') {
//        	FoldRightHand();
//        }
//        else if( ch == ',') {
//        	UnFoldLeftHand();
//        }
//        else if( ch == '.') {
//        	FoldLeftHand();
//        }
        else if( ch >= 'A' && ch <= 'z' )
        {
            char input[128] = {0,};
            char *token;
            int input_len;
            char cmd[80];
            int num_param;
            int iparam[30];

            int idx = 0;

            BeginCommandMode();

            printf("%c", ch);
            input[idx++] = (char)ch;

            while(1)
            {
                ch = _getch();
                if( ch == 0x0A )
                    break;
                else if( ch == 0x7F )
                {
                    if(idx > 0)
                    {
                        ch = 0x08;
                        printf("%c", ch);
                        ch = ' ';
                        printf("%c", ch);
                        ch = 0x08;
                        printf("%c", ch);
                        input[--idx] = 0;
                    }
                }
                else if( ( ch >= 'A' && ch <= 'z' ) || ch == ' ' || ( ch >= '0' && ch <= '9'))
                {
                    if(idx < 127)
                    {
                        printf("%c", ch);
                        input[idx++] = (char)ch;
                    }
                }
            }

            fflush(stdin);
            input_len = strlen(input);
            if(input_len > 0)
            {
                token = strtok( input, " " );
                if(token != 0)
                {
                    strcpy( cmd, token );
                    token = strtok( 0, " " );
                    num_param = 0;
                    while(token != 0)
                    {
                        iparam[num_param++] = atoi(token);
                        token = strtok( 0, " " );
                    }

                    if(strcmp(cmd, "exit") == 0)
                    {
                        if(AskSave() == false)
                            break;
                    }
                    else if(strcmp(cmd, "re") == 0)
                        DrawPage();
                    else if(strcmp(cmd, "help") == 0)
                        HelpCmd();
                    else if(strcmp(cmd, "n") == 0)
                        NextCmd();
                    else if(strcmp(cmd, "b") == 0)
                        PrevCmd();
                    else if(strcmp(cmd, "time") == 0)
                        TimeCmd();
                    else if(strcmp(cmd, "speed") == 0)
                        SpeedCmd();
                    else if(strcmp(cmd, "page") == 0)
                    {
                        if(num_param > 0)
                            PageCmd(iparam[0]);
                        else
                            PrintCmd("Need parameter");
                    }
                    else if(strcmp(cmd, "play") == 0)
                    {
                        PlayCmd();
                    }
//                    else if(strcmp(cmd, "playwith") == 0)
//                    {
//                    	PlayWithCmd(manager);
//                    }
                    else if(strcmp(cmd, "set") == 0)
                    {
                        if(num_param > 0)
                            SetValue( iparam[0]);
                        else
                            PrintCmd("Need parameter");
                    }
                    else if(strcmp(cmd, "list") == 0)
                        ListCmd();
                    else if(strcmp(cmd, "on") == 0)
                        OnOffCmd( true, num_param, iparam);
                    else if(strcmp(cmd, "off") == 0)
                        OnOffCmd( false, num_param, iparam);
                    else if(strcmp(cmd, "w") == 0)
                    {
                        if(num_param > 0)
                            WriteStepCmd(iparam[0]);
                        else
                            PrintCmd("Need parameter");
                    }
                    else if(strcmp(cmd, "d") == 0)
                    {
                        if(num_param > 0)
                            DeleteStepCmd(iparam[0]);
                        else
                            PrintCmd("Need parameter");
                    }
                    else if(strcmp(cmd, "i") == 0)
                    {
                        if(num_param == 0)
                            InsertStepCmd(0);
                        else
                            InsertStepCmd(iparam[0]);
                    }
//                    else if(strcmp(cmd, "m") == 0)
//                    {
//                        if(num_param > 1)
//                            MoveStepCmd(iparam[0], iparam[1]);
//                        else
//                            PrintCmd("Need parameter");
//                    }
                    else if(strcmp(cmd, "mrl")==0)
                    {
                    	if(num_param >= 1) {
                    		MirrorRight2LeftCmd(num_param, iparam);
                    	}
                    	else
                    		PrintCmd("Need parameter");
                    }
                    else if(strcmp(cmd, "mlr") == 0)
                    {
                    	if(num_param >= 1) {
                    		MirrorLeft2RightCmd(num_param, iparam);
                    	}
                    	else
                    		PrintCmd("Need parameter");
                    }
                    else if(strcmp(cmd, "mm") == 0)
                    {
                    	if(num_param >= 1) {
                    		MirrorCmd(num_param, iparam);
                    	}
                    	else
                    		PrintCmd("Need parameter");
                    }
//                    else if(strcmp(cmd, "cs") == 0)
//                    {
//                    	if(num_param >= 2) {
//                    		StepCopyCmd(iparam[0], iparam[1]);
//                    	}
//                    	else
//                    		PrintCmd("Need parameter");
//                    }
                    else if(strcmp(cmd, "copy") == 0)
                    {
                        if(num_param > 0)
                            CopyCmd(iparam[0]);
                        else
                            PrintCmd("Need parameter");
                    }
                    else if(strcmp(cmd, "new") == 0)
                        NewCmd();
                    else if(strcmp(cmd, "g") == 0)
                    {
                        if(num_param > 0)
                            GoCmd(iparam[0]);
                        else
                            PrintCmd("Need parameter");
                    }
                    else if(strcmp(cmd, "save") == 0)
                        SaveCmd();
                    else if(strcmp(cmd, "name") == 0)
                        NameCmd();
                    else
                        PrintCmd("Bad command! please input 'help'");
                }
            }

            EndCommandMode();
        }
    }

    DrawEnding();

    return 0;
}






