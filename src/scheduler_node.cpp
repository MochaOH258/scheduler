#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

class SchedulerNode
{
private:
    ros::NodeHandle nh_;

    ros::Subscriber state_sub_;
    ros::Publisher vel_pub_;

    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    mavros_msgs::State current_state_;


public:
    SchedulerNode()
    {
        state_sub_ = nh_.subscribe<mavros_msgs::State>(
            "/mavros/state", 10, &SchedulerNode::stateCb, this);

        vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
            "/mavros/setpoint_velocity/cmd_vel", 10);

        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
            "/mavros/cmd/arming");

        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
            "/mavros/set_mode");

        ROS_INFO("Scheduler");
    }

    void stateCb(const mavros_msgs::State::ConstPtr& msg)
    {
        current_state_ = *msg;
    }

    bool waitForConnection()
    {
        ros::Rate rate(10.0);
        while (ros::ok() && !current_state_.connected)
        {
            ros::spinOnce();
            rate.sleep();
        }
        return current_state_.connected;
    }

    bool setMode(const std::string& mode_name)
    {
        mavros_msgs::SetMode srv;
        srv.request.base_mode = 0;
        srv.request.custom_mode = mode_name;

        if (set_mode_client_.call(srv))
        {
            return srv.response.mode_sent;
        }
        return false;
    }

    bool arm(bool value)
    {
        mavros_msgs::CommandBool srv;
        srv.request.value = value;

        if (arming_client_.call(srv))
        {
            return srv.response.success;
        }
        return false;
    }

    void publishVelocity(double vx, double vy, double vz, double yaw_rate)
    {
        geometry_msgs::TwistStamped cmd;
        cmd.header.stamp = ros::Time::now();

        cmd.twist.linear.x = vx;
        cmd.twist.linear.y = vy;
        cmd.twist.linear.z = vz;
        cmd.twist.angular.z = yaw_rate;

        vel_pub_.publish(cmd);
    }

    void run()
    {
        ros::Rate rate(20.0);

        waitForConnection();

        setMode("GUIDED");

        arm(true);

        while (ros::ok())
        {
            ros::spinOnce();

            double vx = 0.011;
            double vy = 0.045;
            double vz = 0.014;
            double yaw_rate = 0.01919;

            publishVelocity(vx, vy, vz, yaw_rate);

            rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scheduler_node");
    SchedulerNode node;
    node.run();
    return 0;
}