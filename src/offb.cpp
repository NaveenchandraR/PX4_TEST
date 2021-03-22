#include <ros/ros.h>
#include <ros/console.h>

#include <string>
#include <boost/thread.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <mission_planner/pose.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped pos, leader_pos;

void PoseCallBack(const geometry_msgs::PoseStampedConstPtr& msg)
{
    pos.pose = msg->pose;
}

void SetPos(const geometry_msgs::PoseStamped setpose)
{
    leader_pos.pose = setpose.pose;
}

bool Leader(mission_planner::pose::Request& req,
            mission_planner::pose::Response& res)
{
    geometry_msgs::PoseStamped setpose;
    setpose.pose.position.x = req.pos_x;
    setpose.pose.position.y = req.pos_y;
    setpose.pose.position.z = req.pos_z;
    SetPos(setpose);
    // local_pos_pub.publish(setpose);
    res.success = true;
    return res.success;
}

int main (int argc, char** argv)
{
    std::string host_name = getenv("HOST");
    std::string vehicle_id;
    std::stringstream ss(host_name);
    while(getline(ss, vehicle_id, '_')) {}

    ros::init(argc, argv, host_name + std::string("_test_node"));
    ros::NodeHandle nh(host_name);

    int leader_id;

    while(ros::ok() and (not nh.getParam("leader_id", leader_id)))
    {
        ROS_INFO("Node waiting for leader id");
    }

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/" + host_name + "/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/" + host_name + "/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/" + host_name + "/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/" + host_name + "/mavros/set_mode");
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav_" + std::to_string(leader_id) + "/mavros/local_position/pose", 10, &PoseCallBack);
    ros::ServiceServer leader = nh.advertiseService
            ("/leader_pose", &Leader);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::Time::now() - last_request <= ros::Duration(10.0)){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    leader_pos.pose.position.x = 0;
    leader_pos.pose.position.y = 0;
    leader_pos.pose.position.z = 5;

    last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if(std::stoi(vehicle_id) == leader_id)
        {
            nh.setParam("leader_publishing", leader_pos.pose.position.z);
            local_pos_pub.publish(leader_pos);
        }
        else
        {
            local_pos_pub.publish(pos);
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}