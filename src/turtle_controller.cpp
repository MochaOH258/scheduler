#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_controller");
    ros::NodeHandle nh("~");

    ros::Publisher cmd_pub =
        nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    double linear_speed = 2.0;
    double angular_speed = 1.0;
    nh.param("linear_speed", linear_speed, linear_speed);
    nh.param("angular_speed", angular_speed, angular_speed);

    ros::Rate rate(10.0);
    while (ros::ok())
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = linear_speed;
        cmd.angular.z = angular_speed;
        cmd_pub.publish(cmd);
        rate.sleep();
    }

    return 0;
}
