#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <term.h>
#include <ncurses.h>
#include <signal.h>
#include <libgen.h>

#include <ros/ros.h>

#include "op3_walking_tuner/cmd_process.h"


//#define INI_FILE_PATH       "../../../Data/config.ini"
//
//using namespace Robot;
//
//LinuxCM730 linux_cm730("/dev/ttyUSB0");
//CM730 cm730(&linux_cm730);



void sighandler(int sig)
{
    struct termios term;
    tcgetattr( STDIN_FILENO, &term );
    term.c_lflag |= ICANON | ECHO;
    tcsetattr( STDIN_FILENO, TCSANOW, &term );

    exit(0);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "OP3 Walking Tuner");
    ros::NodeHandle _nh;

    std::string         _offset_file    = _nh.param<std::string>("offset_table", "");
    std::string         _robot_file     = _nh.param<std::string>("robot_file_path", "");

    std::string         _init_file      = _nh.param<std::string>("init_file_path", "");

    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);



    if(InitializeWalkingTuner(_robot_file, _init_file, _offset_file) == false) {
        ROS_ERROR("Failed to Initialize");
        return 0;
    }

    if(DrawIntro() == false) {
        ROS_ERROR("Failed to Get Current Walking Param");
        return 0;
    }

    while(1)
    {
        int ch = _getch();
        if(ch == 0x1b)
        {
            ch = _getch();
            if(ch == 0x5b)
            {
                ch = _getch();
                if(ch == 0x41) // Up arrow key
                    MoveUpCursor();
                else if(ch == 0x42) // Down arrow key
                    MoveDownCursor();
            }
        }
        else if( ch == '[' )
            UpdateValue(false, -1.0);
        else if( ch == ']' )
            UpdateValue(false, 1.0);
        else if( ch == '{' )
            UpdateValue(true, -1.0);
        else if( ch == '}' )
            UpdateValue(true, 1.0);
        else if( ch >= 'A' && ch <= 'z' )
        {
            char input[128] = {0,};
            char *token;
            int input_len;
            char cmd[80];
            char strParam[20][30];
            int num_param;

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
                else if( ch >= 'A' && ch <= 'z' )
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
                        strcpy(strParam[num_param++], token);
                        token = strtok( 0, " " );
                    }

                    if(strcmp(cmd, "exit") == 0)
                    {
                        if(AskSave() == false)
                            break;
                    }
                    if(strcmp(cmd, "re") == 0)
                        DrawScreen();
                    else if(strcmp(cmd, "save") == 0)
                    {
                        SaveCmd();
                    }
                    else if(strcmp(cmd, "mon") == 0)
                    {
                        MonitorCmd();
                    }
                    else if(strcmp(cmd, "help") == 0)
                        HelpCmd();
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
