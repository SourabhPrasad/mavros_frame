#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/SetMode.h> 
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <string>

class ModeSet
{
    private:
        ros::NodeHandle node;
        
        ros::ServiceClient mode_client;
        mavros_msgs::SetMode new_mode;   

        ros::ServiceClient arm_client;
        mavros_msgs::CommandBool arm_cmd; 

        ros::Subscriber get_button;         
        
        //ros::Subscriber get_state;    //for verification
        //mavros_msgs::State mode;
        
        //std::string current_mode;     //for verification
        //int key;

    
    public:
        ModeSet();

        void send_request();
        void keyCallback(const sensor_msgs::Joy::ConstPtr &joy_val);
        void send_arm_request();

        //void check_mode(const mavros_msgs::State::ConstPtr &state);
        //void verify(); member function for verification
};

//Constructor - Initializing values
ModeSet::ModeSet()
{
    //key = 0;

    mode_client = node.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    get_button  = node.subscribe("/joy", 1, &ModeSet::keyCallback, this);
    arm_client  = node.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    //get_state   = node.subscribe("/mavros/state", 1, &ModeSet::check_mode, this);
}

//Get the key input from the joystick(Change to websockt later)
//Current mapped as per Dualshock 4 controller
//and then set the new mode as per the keypress on the new variable "new_mode"
void ModeSet::keyCallback(const sensor_msgs::Joy::ConstPtr &joy_val)
{
    if(joy_val->buttons[8] == 1 && joy_val->buttons[9] == 1)                     //up arrow key
    {                           
        arm_cmd.request.value = true;
        send_arm_request();
    }
    else if (joy_val->buttons[10] == 1)                                         //down arrow key
    {
        arm_cmd.request.value = false;
        send_arm_request();
    }
    else if (joy_val->buttons[0] == 1)                                         //X
    {
        new_mode.request.custom_mode = "MANUAL";
        send_request();
    }
    else if (joy_val->buttons[1] == 1)                                         //Circle
    {
        new_mode.request.custom_mode = "LOITER";
        send_request();
    }
    else if (joy_val->buttons[2] == 1)                                         //Triangle
    {
        new_mode.request.custom_mode = "HOLD";
        send_request();
    }
    else if (joy_val->buttons[3] == 1)                                         //Square
    {
        new_mode.request.custom_mode = "GUIDED";
        send_request();
    }
}

//Get the current state of the FCU.
//For verification if the mode has actually changed(not implemented)
// void ModeSet::check_mode(const mavros_msgs::State::ConstPtr &state)
// {
//     current_mode = state->mode;
// }

//Sends request to FCU to change the mode
//Small bug where the mode dosent change with a single button press.
//Need to test to with websocket to see if the problem persists.
//If it persists then implement a verification logic.
void ModeSet::send_request()
{
    if(mode_client.call(new_mode))
    {
        ROS_INFO("Mode change request:%d", new_mode.response.mode_sent);
    }
    else
    {
        ROS_INFO("Cannot communicate with mode change server");
    }
}

void ModeSet::send_arm_request()
{
    if(arm_client.call(arm_cmd))
    {
        ROS_INFO("Arm Status:%d", arm_cmd.response.result);
    }
    else
    {
        ROS_INFO("Could not communicated with arm server");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mode_change");
    ModeSet obj;
    ros::spin();
    return 0;
}