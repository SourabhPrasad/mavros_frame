#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/OverrideRCIn.h>

class Thruster_control
{
    private:
        ros::NodeHandle node;
        
        ros::Subscriber joy;
        ros::Publisher send_rc;
        mavros_msgs::OverrideRCIn override_rc;
        
        float min_thrust = 1100.0;
        float max_thrust = 1900.0;

        float min_steering = 1100.0;
        float max_steering = 1900.0;

    public:
        Thruster_control();
        void get_joy(const sensor_msgs::Joy::ConstPtr &value);
        float change_range(float value, bool assign);

};

Thruster_control::Thruster_control()
{
    joy = node.subscribe("/joy", 1, &Thruster_control::get_joy, this);
    send_rc = node.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
}

void Thruster_control::get_joy(const sensor_msgs::Joy::ConstPtr &value)
{
    unsigned short thruster_val_portion = (value->axes[1] - -1.0)*(max_thrust-min_thrust)/(1.0 - -1.0);
    unsigned short steering_val_portion = (value->axes[0] - -1.0)*(max_steering-min_steering)/(1.0 - -1.0);
    unsigned short thruster_val = thruster_val_portion + min_thrust;
    unsigned short steering_val = max_steering - steering_val_portion;
    ROS_INFO("Thrust Value: %d", thruster_val);
    ROS_INFO("Steering Value: %d", steering_val);
    override_rc.channels = {steering_val, 0, thruster_val, 0, 0 , 0, 0, 0};
    send_rc.publish(override_rc);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thruster_control");
    Thruster_control obj;
    ros::spin();
    return 0;
}