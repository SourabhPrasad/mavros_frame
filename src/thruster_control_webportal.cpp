#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/RCIn.h>
#include <std_msgs/Int32.h>

class Thruster_control
{
    private:
        ros::NodeHandle node;
        
        ros::Subscriber get_right;
        ros::Subscriber get_left;
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

        bool override_flag;

        float right_val;
        float left_val;


    public:
        Thruster_control();
        void get_thruster_right(const std_msgs::Int32::ConstPtr &r_val);
        void get_thruster_left(const std_msgs::Int32::ConstPtr &l_val);
        void get_rc(const mavros_msgs::RCIn::ConstPtr &rc_val);
        void pub_override_right();
        void pub_override_left();

};

Thruster_control::Thruster_control()
{
    get_right   = node.subscribe("/X_Thruster/speedValues", 1, &Thruster_control::get_thruster_right, this);
    get_left    = node.subscribe("/Y_Thruster/speedValues", 1, &Thruster_control::get_thruster_left, this);
    send_rc     = node.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
    rc_in       = node.subscribe("/mavros/rc/in", 1, &Thruster_control::get_rc, this);

    thruster_val_portion    = 0;       
    steering_val_portion    = 0;
    thruster_val            = 0;            
    steering_val            = 0;

    min_thrust              = 1100.0;
    max_thrust              = 1900.0;
    min_steering            = 1100.0;
    max_steering            = 1900.0;

    override_flag = true;
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

void Thruster_control::get_thruster_left(const std_msgs::Int32::ConstPtr &l_v)
{
    left_val = l_v->data;
    ROS_INFO("Value: %f", left_val);
    pub_override_right();

}

void Thruster_control::get_thruster_right(const std_msgs::Int32::ConstPtr &r_v)
{
    right_val = r_v->data;
    ROS_INFO("right val: %f", right_val);
    pub_override_left();
}

void Thruster_control::pub_override_right()
{
    if(override_flag)
    {
        thruster_val_portion = (left_val - -40.0)*(max_thrust-min_thrust)/(40.0 - -40.0);
        thruster_val = thruster_val_portion + min_thrust;
        ROS_INFO("Thrust Value: %d", thruster_val);
        override_rc.channels = {1500, 0, thruster_val, 0, 0 , 0, 0, 0};
        send_rc.publish(override_rc);
    }
}
void Thruster_control::pub_override_left()
{
    if(override_flag)
    {
        steering_val_portion = (right_val - -40.0)*(max_steering-min_steering)/(40.0 - -40.0);
        steering_val = steering_val_portion + min_steering;
        ROS_INFO("Steering Value: %d", steering_val);
        override_rc.channels = {steering_val, 0, 1600, 0, 0 , 0, 0, 0};
        if(right_val == 0)
        {
            override_rc.channels = {steering_val, 0, 1500, 0, 0 , 0, 0, 0};
        }
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