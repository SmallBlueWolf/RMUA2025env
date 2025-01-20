#ifndef _UNIARC_MAIN_HPP_
#define _UNIARC_MAIN_HPP_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <airsim_ros/Takeoff.h>
#include <airsim_ros/Reset.h>
#include <airsim_ros/VelCmd.h>
#include <airsim_ros/RotorPWM.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <string>
#include <nav_msgs/Odometry.h>
#include <memory>

// 定义 EKF 类
class UnionsysEKF {
public:
    UnionsysEKF();
    void predict();
    void update(const Eigen::VectorXd &z);
    Eigen::VectorXd getState() const { return x_; }

private:
    Eigen::VectorXd x_;  // 状态向量
    Eigen::MatrixXd P_, F_, H_, R_, Q_, K_;  // EKF 矩阵
};

// 定义主类 BasicDev
class BasicDev {
public:
    BasicDev(ros::NodeHandle *nh);
    ~BasicDev();

private:
    // ROS 相关组件
    ros::NodeHandle *nh_;
    ros::Subscriber pos_cmd_sub;
    ros::Subscriber initial_pose_sub;
    ros::Subscriber end_goal_sub;
    // ros::Subscriber odom_suber;//状态真值
    ros::Subscriber gps_suber;//gps数据

    image_transport::Subscriber front_left_view_suber;
    image_transport::Subscriber front_right_view_suber;
    image_transport::Publisher front_left_pub;
    image_transport::Publisher front_right_pub;
    ros::Publisher vel_publisher;
    ros::Publisher pwm_publisher;
    ros::Publisher odom_pub_;

    ros::ServiceClient takeoff_client;
    ros::ServiceClient land_client;
    ros::ServiceClient reset_client;

    // 图像处理相关
    std::unique_ptr<image_transport::ImageTransport> it;
    cv::Mat front_left_img;
    cv::Mat front_right_img;
    cv_bridge::CvImagePtr cv_front_left_ptr;
    cv_bridge::CvImagePtr cv_front_right_ptr;

    // EKF 状态估计
    UnionsysEKF ekf;

    // PID 参数
    double Kpx, Kdx, Kix, Kpy, Kdy, Kiy, Kpz, Kdz, Kiz;
    double current_pos_x, current_pos_y, current_pos_z;
    double current_vel_x, current_vel_y, current_vel_z;
    double previous_error_x, previous_error_y, previous_error_z;
    double integral_x, integral_y, integral_z;

    // 状态变量
    geometry_msgs::Pose initial_pose_;
    geometry_msgs::Pose target_pose_;
    bool has_initial_pose_ = false;
    bool has_target_pose_ = false;
    bool planner_started_ = false;

    // ROS 消息
    airsim_ros::Takeoff takeoff;
    airsim_ros::Takeoff land;
    airsim_ros::Reset reset;
    airsim_ros::VelCmd velcmd;
    airsim_ros::RotorPWM pwm_cmd;

    // 回调函数
    // void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void gps_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void posCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
    void initialPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void endGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void front_left_view_cb(const sensor_msgs::ImageConstPtr &msg);
    void front_right_view_cb(const sensor_msgs::ImageConstPtr &msg);

};

// 辅助函数
void startEgoPlanner(const geometry_msgs::Pose &init_pose, const geometry_msgs::Pose &target_pose);

#endif
