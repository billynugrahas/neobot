#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <neo_msgs/Vel.h>

class NeobotBase
{
    std::string frame_id_;
    std::string child_frame_id_;
    
    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Subscriber vel_sub_;

    double pos_x_;
    double pos_y_;
    double heading_;

    double vx_;
    double vy_;
    double vth_;

    double vel_dt_;

    ros::Time last_time_;

    void velocityCallback(const neo_msgs::Vel& vel)
    {
        ros::Time current_time = ros::Time::now();

        vx_  = vel.linear_velocity_x;  //m/s
        vy_  = vel.linear_velocity_y;  //m/s
        vth_ = vel.angular_velocity_z; //rad/s

        vel_dt_ = (current_time - last_time_).toSec();
        last_time_ = current_time;

        double delta_heading = vth_ * vel_dt_; //rad = rad/s * s
        double delta_x = (vx_ * cos(heading_) - vy_ * sin(heading_)) * vel_dt_; //m = m/s * s
        double delta_y = (vx_ * sin(heading_) + vy_ * cos(heading_)) * vel_dt_; //m = m/s * s

        pos_x_ += delta_x;
        pos_y_ += delta_y;
        heading_ += delta_heading;
        
        //transform
        tf2::Quaternion odom_quaternion;
        odom_quaternion.setRPY(0, 0, heading_);

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = frame_id_;
        odom_trans.child_frame_id = child_frame_id_;

        odom_trans.transform.translation.x = pos_x_;
        odom_trans.transform.translation.x = pos_y_;
        odom_trans.transform.translation.z = 0.0;

        odom_trans.transform.rotation.x = odom_quaternion.x();
        odom_trans.transform.rotation.y = odom_quaternion.y();
        odom_trans.transform.rotation.z = odom_quaternion.z();
        odom_trans.transform.rotation.w = odom_quaternion.w();
        
        //odom
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = frame_id_;
        odom.child_frame_id = child_frame_id_;

        odom.pose.pose.position.x = pos_x_;
        odom.pose.pose.position.y = pos_y_;
        odom.pose.pose.position.z = 0.0;

        odom.pose.pose.orientation.x = odom_quaternion.x();
        odom.pose.pose.orientation.y = odom_quaternion.y();
        odom.pose.pose.orientation.z = odom_quaternion.z();
        odom.pose.pose.orientation.w = odom_quaternion.w();
        odom.pose.covariance[0] = 0.001;
        odom.pose.covariance[7] = 0.001;
        odom.pose.covariance[35] = 0.001;
        
        odom.twist.twist.linear.x = vx_;
        odom.twist.twist.linear.y = vy_;
        odom.twist.twist.linear.z = 0.0;
        
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;

        odom.twist.twist.angular.z = vth_;
        odom.twist.covariance[0] = 0.0001;
        odom.twist.covariance[7] = 0.0001;
        odom.twist.covariance[35] = 0.0001;
        
        odom_pub_.publish(odom);
    }

    public:
        NeobotBase(std::string frame_id, std::string child_frame_id):
            frame_id_(frame_id),
            child_frame_id_(child_frame_id),
            pos_x_(0),
            pos_y_(0),
            heading_(0),
            vx_(0),
            vy_(0),
            vth_(0),
            last_time_(0),
            vel_dt_(0)
        {
            odom_pub_ = nh_.advertise<nav_msgs::Odometry>("raw_odom", 10);
            vel_sub_  = nh_.subscribe("raw_vel", 10, &NeobotBase::velocityCallback, this);
        }
};

int main(int argc, char** argv )
{
    ros::init(argc, argv, "neobot_base_node");
    NeobotBase neo("odom", "base_link");
    ros::spin();
    return 0;
}