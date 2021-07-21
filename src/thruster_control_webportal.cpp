#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/RCIn.h>

class Thruster_control
{
    private:
        ros::NodeHandle node;
        
        ros::Subscriber joy;
        ros::Subscriber rc_in;
        ros::Publisher send_rc;
        
        mavros_msgs::OverrideRCIn override_rc;
        
        float min_thrust;
        float max_thrust;

        float min_steering;
        float max_steering;

        unsigned short thruster_val_portion;
        unsigned short steering_val_portion;
        unsigned short thruster_val;
        unsigned short steering_val;

        bool override_flag = false;

    public:
        Thruster_control();
        void get_joy(const sensor_msgs::Joy::ConstPtr &value);
        float change_range(float value, bool assign);
        void get_rc(const mavros_msgs::RCIn::ConstPtr &rc_val);

};

Thruster_control::Thruster_control()
{
    joy     = node.subscribe("/joy", 1, &Thruster_control::get_joy, this);
    send_rc = node.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
    rc_in   = node.subscribe("/mavros/rc/in", 1, &Thruster_control::get_rc, this);

    thruster_val_portion    = 0;       
    steering_val_portion    = 0;
    thruster_val            = 0;            
    steering_val            = 0;

    min_thrust              = 1100.0;
    max_thrust              = 1900.0;
    min_steering            = 1100.0;
    max_steering            = 1900.0;
}

void Thruster_control::get_rc(const mavros_msgs::RCIn::ConstPtr &rc_val)
{
    if(rc_val->channels[10] == 1000)
    {
        if(override_flag)
        {
            override_flag = false;
        }
        else
        {
            override_flag = true;
        }
    }
}

void Thruster_control::get_joy(const sensor_msgs::Joy::ConstPtr &value)
{
    if(override_flag)
    {
        thruster_val_portion = (value->axes[1] - -1.0)*(max_thrust-min_thrust)/(1.0 - -1.0);
        steering_val_portion = (value->axes[0] - -1.0)*(max_steering-min_steering)/(1.0 - -1.0);
        thruster_val = thruster_val_portion + min_thrust;
        steering_val = max_steering - steering_val_portion;
        ROS_INFO("Thrust Value: %d", thruster_val);
        ROS_INFO("Steering Value: %d", steering_val);
        override_rc.channels = {steering_val, 0, thruster_val, 0, 0 , 0, 0, 0};
        send_rc.publish(override_rc);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thruster_control");
    Thruster_control obj;
    ros::spin();
    return 0;
}