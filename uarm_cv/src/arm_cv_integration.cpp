/*
 *  arm_cv_integration.cpp
 *
 *
 *  Created on: Oct 31, 2017
 *  Authors:   Enyu Cao
 *            caoe <at> kth.se
 */

//This program reads keystrokes(G for Go), object coordinate(topic) and publishes arm command(topic)
// Node Rate: Unknown

#include <ros/ros.h> //This header defines the classic ROS classes
#include <signal.h>
#include <termios.h>
#include <stdio.h>
//------------------Need Change-------------------------------
#include<geometry_msgs/PointStamped.h>  //For the message subscribed: object coordinate
//------------------Need Change-------------------------------
#include<geometry_msgs/Point.h>      //For the message to be published object coordinate
//------------------Need Change-------------------------------
//------------------Need Change-------------------------------

#define KEYCODE_G 0x67  //Go to the object
#define KEYCODE_Q 0x71  //Character Q for quit

//Period
float freq = 1.0;  // 1Hz
float T = 1.0/(float)freq;  // 1s
// position m from camera
float x,y,z;

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}
//Callback function 1: destination message
void objectCoordReceiver( const geometry_msgs::PointStamped & msgObjCoord){

    ROS_INFO_STREAM(">>Reference Message Receive!");

    x = msgObjCoord.point.x;  // unit: meter
    y = msgObjCoord.point.y;
    z = msgObjCoord.point.z;
    //receive_state = 1;

}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "keyInput");//Initialize the node, and set default name
    ros::NodeHandle nh;//Establish this node with Master(roscore)
    signal(SIGINT,quit); //To monitor Ctrl+C

    ros::Rate loop_rate(freq); //f=10Hz, T=100ms
    ros::Publisher obj_pub = nh.advertise<geometry_msgs::Point>("/Coord_topic", 1);
    //Create publisher: pub_pwm	// msg_type(use :: for /) //topic_name //buffer_size

    //Type:geometry_msgs/PointStamped
    ros::Subscriber sub_obj = nh.subscribe("/camera/object_candidates",1, &objectCoordReceiver);
    //ros::Subscriber sub_reference = nh.subscribe("/mobile_base/commands/velocity",10, &refMessageReceiver); //Testing

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
    puts("Press G to move the arm.");


    while (ros::ok() )
    {
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0) {
            puts("Error");
            perror("read():");
            exit(-1);
        }

        ROS_DEBUG("value: 0x%02X\n", c);  // Debug message will not show in Terminal. 
        
        switch(c)    {
        case KEYCODE_G:
            ROS_DEBUG("GO");
            puts("Go");
            dirty = true;
            break;
        case KEYCODE_Q:
            //ROS_DEBUG("Q");
            puts("Q for quit");
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
        geometry_msgs::Point msg;
        //Fill in the message
        msg.x = x*100.0+12.5;  //Add offset 12 cm
        msg.y = y*100.0+0.0;  //Add offset 0
        msg.z = z*100.0+0.0;  //Add offset 0

        if(dirty ==true)
        {
            puts("Send Arm Command");
            obj_pub.publish(msg);
            dirty=false;
        }
        ros::spinOnce();


        //------------------------------------------------------------------------------------------------------
        //ROS_INFO_STREAM("="<<var );
        //------------------------------------------------------------------------------------------------------

        loop_rate.sleep(); //sleep
    }



    return(0);
}
