#include <string>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <px4_test/state.h>

px4_test::state out_msg;

void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& local_msg){
    out_msg.pos_x = local_msg->pose.position.x;
    out_msg.pos_y = local_msg->pose.position.y;
    out_msg.pos_z = local_msg->pose.position.z;

    out_msg.quat_x = local_msg->pose.orientation.x;
    out_msg.quat_y = local_msg->pose.orientation.y;
    out_msg.quat_z = local_msg->pose.orientation.z;
    out_msg.quat_w = local_msg->pose.orientation.w;
}

int main(int argc, char **argv)
{
std::string vehicle_namespace, node_name;

vehicle_namespace = getenv("HOST");
node_name = vehicle_namespace + std::string("_swarm_msg");

ros::init(argc, argv, node_name);
ros::NodeHandle nh;
ROS_INFO("Vehicle namespace set to : %s", vehicle_namespace.c_str());

std::string s1, s2, s3, s4;
s2 = std::string("/") + vehicle_namespace + std::string("/mavros/local_position/pose");
s4 = std::string("/") + vehicle_namespace + std::string("/combined");

ros::Subscriber lpos_sub = nh.subscribe<geometry_msgs::PoseStamped>
        (s2, 10, local_position_cb);

   
ros::Publisher msg_pub = nh.advertise<px4_test::state>
            (s4, 100);

ros::Rate rate(5.0);

while(ros::ok()){

    out_msg.header.stamp = ros::Time::now();
    msg_pub.publish(out_msg);

    ros::spinOnce();
    rate.sleep();
}

    return 0;
}