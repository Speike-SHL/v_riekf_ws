#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <fstream>

using namespace std;

void ground_truth_callback(const nav_msgs::Odometry& msg);
void riekf_esti_callback(const nav_msgs::Odometry& msg);

ofstream ground_truth_file;
ofstream riekf_esti_file;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "final_data_node");
    ros::NodeHandle nh;

    string Ground_truth_savePath = ros::param::param("/final_data_node/Ground_truth_savePath", string("/home/speike/v_riekf_ws/src/V-RIEKF/doc/result/ground_truth.txt"));
    string Riekf_esti_savePath = ros::param::param("/final_data_node/Riekf_esti_savePath", string("/home/speike/v_riekf_ws/src/V-RIEKF/doc/result/riekf_esti.txt"));

    ground_truth_file.open(Ground_truth_savePath);
    riekf_esti_file.open(Riekf_esti_savePath);

    ros::Subscriber ground_truth_sub = nh.subscribe("/origin/ground_truth_pub", 2000, ground_truth_callback);
    ros::Subscriber riekf_esti_sub = nh.subscribe("/riekf/riekf_esti_pub", 2000, riekf_esti_callback);

    ros::spin();

    ground_truth_file.close();
    riekf_esti_file.close();

    return 0;
}

void ground_truth_callback(const nav_msgs::Odometry& msg) {
    double current_time = msg.header.stamp.toSec()/1000;
    double qx = msg.pose.pose.orientation.x;
    double qy = msg.pose.pose.orientation.y;
    double qz = msg.pose.pose.orientation.z;
    double qw = msg.pose.pose.orientation.w;
    double vx = msg.twist.twist.linear.x;
    double vy = msg.twist.twist.linear.y;
    double vz = msg.twist.twist.linear.z;
    double x = msg.pose.pose.position.x;
    double y = msg.pose.pose.position.y;
    double z = msg.pose.pose.position.z;
    
    ground_truth_file << fixed << setprecision(8) << current_time << " " << qx << " " << qy << " " << qz << " " << qw << " " << vx << " " << vy << " " << vz << " " << x << " " << y << " " << z << endl;
}

void riekf_esti_callback(const nav_msgs::Odometry& msg) {
    double current_time = msg.header.stamp.toSec()/1000;
    double qx = msg.pose.pose.orientation.x;
    double qy = msg.pose.pose.orientation.y;
    double qz = msg.pose.pose.orientation.z;
    double qw = msg.pose.pose.orientation.w;
    double vx = msg.twist.twist.linear.x;
    double vy = msg.twist.twist.linear.y;
    double vz = msg.twist.twist.linear.z;
    double x = msg.pose.pose.position.x;
    double y = msg.pose.pose.position.y;
    double z = msg.pose.pose.position.z;

    riekf_esti_file << fixed << setprecision(8) << current_time << " " << qx << " " << qy << " " << qz << " " << qw << " " << vx << " " << vy << " " << vz << " " << x << " " << y << " " << z << endl;
}
