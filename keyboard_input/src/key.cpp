/*
 *  keyInput.cpp
 *
 *
 *  Created on: Oct 1, 2017
 *  Authors:   Enyu Cao
 *            caoe <at> kth.se
 */
//This program reads keystrokes and publishes velocity reference to the controller
// Node Rate: Unknown

#include <ros/ros.h> //This header defines the classic ROS classes
#include <signal.h>
#include <termios.h>
#include <stdio.h>
//------------------Need Change-------------------------------
//------------------Need Change-------------------------------
#include<geometry_msgs/Twist.h>      //For the message to be published
//------------------Need Change-------------------------------
//------------------Need Change-------------------------------

#define KEYCODE_R 0x43  //Right
#define KEYCODE_L 0x44  //Left
#define KEYCODE_U 0x41  //Up
#define KEYCODE_D 0x42  //Down
#define KEYCODE_Reset 0x72  //Reset :stop
#define KEYCODE_Q 0x71  //Character Q for quit

double linear, angular, l_scale=0.05, a_scale=0.2;
// linear_velocity m/s  // angular_velocity rad/s
int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}
//void keyLoop()


int main(int argc, char** argv)
{
    linear = 0;
    angular = 0;
    ros::init(argc, argv, "keyInput");//Initialize the node, and set default name
    ros::NodeHandle nh;//Establish this node with Master(roscore)
    signal(SIGINT,quit); //To monitor Ctrl+C

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
    //Create publisher: pub_pwm	// msg_type(use :: for /) //topic_name //buffer_size

    //keyLoop();
    char c;
    bool dirty=false;


    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the robot.");


    while (ros::ok() )
    {
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0) {
            puts("Error");
            perror("read():");
            exit(-1);
        }

        ROS_DEBUG("value: 0x%02X\n", c);  // Debug message will not show in Terminal. But you can use rxconsole, maybe rqt_console works too.(See Tutorial Basic-8)

        switch(c)    {
        case KEYCODE_L:
            ROS_DEBUG("LEFT");
            angular = angular+a_scale;
            puts("LEFT");
            dirty = true;
            break;
        case KEYCODE_R:
            ROS_DEBUG("RIGHT");
            angular = angular -a_scale;
            puts("Right");
            dirty = true;
            break;
        case KEYCODE_U:
            ROS_DEBUG("UP");
            linear = linear+l_scale;
            puts("UP");
            dirty = true;
            break;
        case KEYCODE_D:
            ROS_DEBUG("DOWN");
            puts("Down");
            linear = linear-l_scale;
            dirty = true;
            break;
        case KEYCODE_Reset:
            ROS_DEBUG("Stop");
            puts("Stop");
            linear = 0;
            angular= 0;
            dirty = true;
            break;
        case KEYCODE_Q:
            //ROS_DEBUG("Q");
            puts("Q for quit");
            //linear_ = -1.0;
            //---------USE THIS TO SHUTDOWN---------
            tcsetattr(kfd, TCSANOW, &cooked);
            ros::shutdown();
            exit(0);
            //---------USE ABOVE TO SHUTDOWN---------
            //dirty = true;
            break;
        default:
            printf("Undefiend value: 0x%02X\n", c);
        }
        geometry_msgs::Twist msg;
        //Fill in the message
        msg.linear.x = linear;  //V   m/s
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;

        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = angular;  // omega rad/s
        if(dirty ==true)
        {
            puts("Send keyinput");
            vel_pub.publish(msg);
            dirty=false;
        }


    }
    return(0);
}
/*
void keyLoop()
{


  return;
  

}
*/
