#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <scheduler/ControlModule.h>

#include <string>

class SchedulerNode
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_{"~"};

    ros::Subscriber state_sub_;
    ros::Publisher vel_pub_;

    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient module_control_client_;

    mavros_msgs::State current_state_;

    std::string flight_mode_;
    std::string autostart_module_name_;
    bool module_started_{false};
    ros::Time last_module_request_time_;

    double vx_{0.011};
    double vy_{0.045};
    double vz_{0.014};
    double yaw_rate_{0.01919};


public:
    SchedulerNode()
    {
        pnh_.param<std::string>("flight_mode", flight_mode_, "GUIDED");
        pnh_.param<std::string>("autostart_module", autostart_module_name_, "turtle");
        pnh_.param("vx", vx_, vx_);
        pnh_.param("vy", vy_, vy_);
        pnh_.param("vz", vz_, vz_);
        pnh_.param("yaw_rate", yaw_rate_, yaw_rate_);

        state_sub_ = nh_.subscribe<mavros_msgs::State>(
            "/mavros/state", 10, &SchedulerNode::stateCb, this);

        vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
            "/mavros/setpoint_velocity/cmd_vel", 10);

        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
            "/mavros/cmd/arming");

        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
            "/mavros/set_mode");

        module_control_client_ = nh_.serviceClient<scheduler::ControlModule>(
            "/launch_manager_node/control_module");

        ROS_INFO("Scheduler node started");
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

    void tryStartModule()
    {
        if (module_started_ || autostart_module_name_.empty())
        {
            return;
        }

        const ros::Time now = ros::Time::now();
        if (!last_module_request_time_.isZero() &&
            (now - last_module_request_time_).toSec() < 1.0)
        {
            return;
        }

        last_module_request_time_ = now;

        if (!module_control_client_.waitForExistence(ros::Duration(0.1)))
        {
            ROS_WARN_THROTTLE(5.0, "Waiting for launch manager service");
            return;
        }

        scheduler::ControlModule srv;
        srv.request.module_name = autostart_module_name_;
        srv.request.command = "start";

        if (!module_control_client_.call(srv))
        {
            ROS_WARN_THROTTLE(5.0, "Failed to call launch manager service");
            return;
        }

        if (srv.response.success)
        {
            module_started_ = (srv.response.state == "RUNNING");
            ROS_INFO("Autostart module [%s]: %s",
                     autostart_module_name_.c_str(),
                     srv.response.message.c_str());
            return;
        }

        ROS_WARN_THROTTLE(5.0,
                          "Autostart module [%s] failed: %s",
                          autostart_module_name_.c_str(),
                          srv.response.message.c_str());
    }

    void run()
    {
        ros::Rate rate(20.0);

        if (!waitForConnection())
        {
            ROS_ERROR("FCU connection was not established");
            return;
        }

        if (!set_mode_client_.waitForExistence(ros::Duration(5.0)))
        {
            ROS_WARN("Set mode service is not available yet");
        }

        if (!arming_client_.waitForExistence(ros::Duration(5.0)))
        {
            ROS_WARN("Arming service is not available yet");
        }

        if (!setMode(flight_mode_))
        {
            ROS_WARN("Failed to set flight mode to %s", flight_mode_.c_str());
        }

        if (!arm(true))
        {
            ROS_WARN("Failed to arm vehicle");
        }

        while (ros::ok())
        {
            ros::spinOnce();
            publishVelocity(vx_, vy_, vz_, yaw_rate_);
            tryStartModule();

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
