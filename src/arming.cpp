#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

class arm_auth
{
    private:
        ros::NodeHandle node;

        ros::ServiceClient arm_client;
        mavros_msgs::CommandBool arm_cmd; 

        std_msgs::Bool auth_val;

        ros::Subscriber get_aws;

        ros::Publisher auth;

        bool auth_flag;
    public:
    arm_auth();
    void aws_sub(const std_msgs::String::ConstPtr &info);
    void pub_auth(bool value);

};

arm_auth::arm_auth()
{
    arm_client  = node.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    get_aws     = node.subscribe("/aws/iot/available", 1, &arm_auth::aws_sub, this);
    auth        = node.advertise<std_msgs::Bool>("/machine/available", 1);

    auth_flag   = false;    
}

void arm_auth::aws_sub(const std_msgs::String::ConstPtr &info)
{
    if(info->data == "Authentication_success")
    {
        pub_auth(true);
        auth_flag = true;
    }
    else if(info->data == "User_logout")
    {
        pub_auth(false);
        auth_flag = false;
        arm_cmd.request.value = false;
        arm_client.call(arm_cmd);
    }
    else if(info->data == "arm" && auth_flag)
    {
        arm_cmd.request.value = true;
        arm_client.call(arm_cmd);
    }
    else if(info->data == "disarm" && auth_flag)
    {
        arm_cmd.request.value = false;
        arm_client.call(arm_cmd);
    }
}

void arm_auth::pub_auth(bool value)
{
    auth_val.data = value;
    auth.publish(auth_val);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arming_client");
    arm_auth obj;
    ros::spin();
    return 0;
}


