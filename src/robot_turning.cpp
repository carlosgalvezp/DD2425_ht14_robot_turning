#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include <geometry_msgs/Twist.h>

#include <cmath>

#define PUBLISH_RATE 10 // Hz
#define QUEUE_SIZE 1000


class Robot_turning
{
public:

    Robot_turning(const ros::NodeHandle& n);
    void run( double turning_angle );

private:

    // TODO: obtained from imu or odometry.. or both?
    double current_angle;

    // TODO: make it as param in launch file
    double turning_speed;

    // angle obtained from odometry first time
    double base_angle;
    bool first_time_called;

    ros::NodeHandle n_;

    ros::Subscriber pose2d_sub_;
    ros::Publisher twist_pub_;

    // Callback func when pose data received
    void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
};

int main (int argc, char* argv[])
{  
    // ** Init node
    ros::init(argc, argv, "robot_turning");
    ros::NodeHandle n;

    // ** Create object
    Robot_turning turn(n);

    // ** turn 90 degrees (to the right?????)
    turn.run( M_PI / 2 );
}

Robot_turning::Robot_turning(const ros::NodeHandle &n)
    : n_(n)
{
    turning_speed = 0.5;
    current_angle = 0;
    first_time_called = true;

    // Publisher
    twist_pub_ = n_.advertise<geometry_msgs::Twist>("/motor_controller/twist", QUEUE_SIZE);
    // Subscriber
    pose2d_sub_ = n_.subscribe("/robot/pose2d", QUEUE_SIZE,  &Robot_turning::poseCallback, this);
}

void Robot_turning::poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    if (first_time_called == true)
    {
        base_angle = msg->theta;
        first_time_called = false;
    }
    else
    {
        current_angle = msg->theta - base_angle;

        std::cout << "update:" << current_angle << " base: " << base_angle << std::endl;
    }
}

void Robot_turning::run( double turning_angle )
{
    ros::Rate loop_rate(PUBLISH_RATE);

    while(ros::ok() && ( ( ( current_angle < turning_angle) && ( turning_angle > 0 ) ) || ( ( current_angle > turning_angle) && ( turning_angle < 0 ) ) ) )
    {
        geometry_msgs::Twist msg;

        std::cout << "current_angle: " << current_angle << " turning_angle: " << turning_angle << std::endl;

        if( turning_angle < 0 )
        {
            // turning left
            msg.angular.z = turning_speed;
        }
        else
        {
            // turning right
            msg.angular.z = -turning_speed;
        }

        twist_pub_.publish(msg);

        // ** Sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    geometry_msgs::Twist msg;
    msg.angular.z = 0;
    twist_pub_.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();

    std::cout << "Exiting...\n";
}

