#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/Thrust.h>

class Thruster_control
{
    private:
        ros::NodeHandle node;
        
        ros::Subscriber joy;
        
        ros::Publisher pub_velocity;
        geometry_msgs::TwistStamped velocity;
        
        ros::Publisher pub_thrust;
        mavros_msgs::Thrust thrust;
        
        float max_ang_velocity = 5.0;
        float min_ang_velocity = -5.0;

    public:
        Thruster_control();
        void get_joy(const sensor_msgs::Joy::ConstPtr &value);
        float change_range(float value, bool assign);

};

Thruster_control::Thruster_control()
{
    joy             = node.subscribe("/joy", 1, &Thruster_control::get_joy, this);
    pub_velocity    = node.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_attitude/cmd_vel", 1);
    pub_thrust      = node.advertise<mavros_msgs::Thrust>("/mavros/setpoint_attitude/thrust", 1);
}

void Thruster_control::get_joy(const sensor_msgs::Joy::ConstPtr &value)
{
    ROS_INFO("Thrust Value: %f", value->axes[1]);
    ROS_INFO("Steering Value: %f", value->axes[0]);
    thrust.thrust = value->axes[1];
    float steering_val_portion = (value->axes[0] - -1.0)*(max_ang_velocity-min_ang_velocity)/(1.0 - -1.0);
    float steering_val = max_ang_velocity - steering_val_portion;
    velocity.twist.angular.z = value->axes[0];
    pub_thrust.publish(thrust);
    pub_velocity.publish(velocity);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thruster_control");
    Thruster_control obj;
    ros::spin();
    return 0;
}