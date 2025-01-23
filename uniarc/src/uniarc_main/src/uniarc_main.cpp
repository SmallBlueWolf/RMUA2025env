#include "uniarc_main.hpp"
#include <iostream>
#include <cstdlib>

int main(int argc, char **argv) {
    ros::init(argc, argv, "uniarc_main");
    ros::NodeHandle n;
    BasicDev go(&n);
    ros::spin();
    return 0;
}

void startEgoPlanner(const geometry_msgs::Pose &init_pose, const geometry_msgs::Pose &target_pose) {
    std::string command = std::string("bash -c 'cd /uniarc_main/src && . /uniarc_main/devel/setup.bash && roslaunch ego_planner uniarc.launch ") +
                          "init_x:=" + std::to_string(init_pose.position.x) + " " +
                          "init_y:=" + std::to_string(init_pose.position.y) + " " +
                          "init_z:=" + std::to_string(init_pose.position.z) + " " +
                          "target_x:=" + std::to_string(target_pose.position.x) + " " +
                          "target_y:=" + std::to_string(target_pose.position.y) + " " +
                          "target_z:=" + std::to_string(target_pose.position.z) + " &'";

    std::cout << "Executing: " << command << std::endl;
    int ret = system(command.c_str());
    if (ret == 0) {
        std::cout << "EGO-Planner launched successfully in the background." << std::endl;
    } else {
        std::cerr << "Failed to launch EGO-Planner." << std::endl;
    }
}

BasicDev::BasicDev(ros::NodeHandle *nh)
    : nh_(nh), current_pos_x(0), current_pos_y(0), current_pos_z(0),
      current_vel_x(0), current_vel_y(0), current_vel_z(0),
      previous_error_x(0), previous_error_y(0), previous_error_z(0),
      integral_x(0), integral_y(0), integral_z(0),
      Kpx(1.0), Kdx(0.5), Kix(0.1),
      Kpy(1.0), Kdy(0.5), Kiy(0.1),
      Kpz(1.0), Kdz(0.5), Kiz(0.1) 
{

    it = std::make_unique<image_transport::ImageTransport>(*nh);
    front_left_img = cv::Mat(720, 960, CV_8UC3, cv::Scalar(0));
    front_right_img = cv::Mat(720, 960, CV_8UC3, cv::Scalar(0));

    // odom_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/debug/pose_gt", 1, std::bind(&BasicDev::pose_cb, this, std::placeholders::_1));
    // gps_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/gps", 1, std::bind(&BasicDev::gps_cb, this, std::placeholders::_1));
    // odom_pub_ = nh->advertise<nav_msgs::Odometry>("/uniarc/odom", 1);
    // imu_suber = nh->subscribe<sensor_msgs::Imu>("airsim_node/drone_1/imu/imu", 1, std::bind(&BasicDev::imu_cb, this, std::placeholders::_1));//imu数据

    pos_cmd_sub = nh->subscribe<quadrotor_msgs::PositionCommand>(
        "/planning/pos_cmd", 1, &BasicDev::posCmdCallback, this);

    initial_pose_sub = nh_->subscribe<geometry_msgs::PoseStamped>(
        "/airsim_node/initial_pose", 1, &BasicDev::initialPoseCallback, this);

    end_goal_sub = nh_->subscribe<geometry_msgs::PoseStamped>(
        "/airsim_node/end_goal", 1, &BasicDev::endGoalCallback, this);

    // 使用publisher发布速度指令需要定义 Velcmd , 并赋予相应的值后，将他publish（）出去
    velcmd.twist.angular.z = 0;//z方向角速度(yaw, deg)
    velcmd.twist.linear.x = 0; //x方向线速度(m/s)
    velcmd.twist.linear.y = 0;//y方向线速度(m/s)
    velcmd.twist.linear.z = 0; //z方向线速度(m/s)

    pwm_cmd.rotorPWM0 = 0.1;
    pwm_cmd.rotorPWM1 = 0.1;
    pwm_cmd.rotorPWM2 = 0.1;
    pwm_cmd.rotorPWM3 = 0.1;


    takeoff_client = nh->serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    land_client = nh->serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/land");
    reset_client = nh->serviceClient<airsim_ros::Reset>("/airsim_node/reset");

    vel_publisher = nh->advertise<airsim_ros::VelCmd>(
        "airsim_node/drone_1/vel_cmd_body_frame", 1);
    pwm_publisher = nh->advertise<airsim_ros::RotorPWM>(
        "airsim_node/drone_1/rotor_pwm_cmd", 1);

    takeoff_client.call(takeoff);

    ros::spin();
}

BasicDev::~BasicDev() {}

void BasicDev::posCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg) {
    // double pos_x = msg->position.x;
    // double pos_y = msg->position.y;
    // double pos_z = msg->position.z;

    // double vel_x = msg->velocity.x;
    // double vel_y = msg->velocity.y;
    // double vel_z = msg->velocity.z;

    // double yaw_dot = msg->yaw_dot;

    // velcmd.twist.angular.z = yaw_dot;//z方向角速度(yaw, deg)
    // velcmd.twist.linear.x = vel_x; //x方向线速度(m/s)
    // velcmd.twist.linear.y = vel_y;//y方向线速度(m/s)
    // velcmd.twist.linear.z = vel_z; //z方向线速度(m/s)

    // vel_publisher.publish(velcmd);
    std::cout<<"get cmd";
}

void BasicDev::initialPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    initial_pose_ = msg->pose;
    has_initial_pose_ = true;

    if (has_initial_pose_ && has_target_pose_ && !planner_started_) {
        startEgoPlanner(initial_pose_, target_pose_);
        planner_started_ = true;
    }
}

void BasicDev::endGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    target_pose_ = msg->pose;
    has_target_pose_ = true;

    if (has_initial_pose_ && has_target_pose_ && !planner_started_) {
        startEgoPlanner(initial_pose_, target_pose_);
        planner_started_ = true;
    }
}

UnionsysEKF::UnionsysEKF()
{
    // EKF Param
    Eigen::VectorXd x0(9);  
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(9, 9) * 0; 
    Eigen::MatrixXd F(9, 9);  
    Eigen::MatrixXd H(3, 9); 
    Eigen::MatrixXd R(3,3);
    Eigen::MatrixXd Q(9,9);
    float dt = 0.1;
    float rconstant = 1.2;
    float qconstant = 0.1;
    x0 << 0, 0, 0, 0, 0, 0, 0, 0, 0; 
    F << 1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0, 0,  
         0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0,
         0, 0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt,
         0, 0, 0, 1, 0, 0, dt, 0, 0,
         0, 0, 0, 0, 1, 0, 0, dt, 0,
         0, 0, 0, 0, 0, 1, 0, 0, dt,
         0, 0, 0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 1;  
    H << 1, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0, 0, 0, 
         0, 0, 1, 0, 0, 0, 0, 0, 0; 
    R << rconstant, 0, 0,
         0, rconstant, 0,  
         0, 0, rconstant;
    Q << 10*qconstant, 0, 0, 0, 0, 0, 0, 0, 0,  
         0, 10*qconstant, 0, 0, 0, 0, 0, 0, 0, 
         0, 0, 10*qconstant, 0, 0, 0, 0, 0, 0,  
         0, 0, 0, qconstant, 0, 0, 0, 0, 0,  
         0, 0, 0, 0, qconstant, 0, 0, 0, 0,  
         0, 0, 0, 0, 0, qconstant, 0, 0, 0, 
         0, 0, 0, 0, 0, 0, 0.1*qconstant, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0.1*qconstant, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0.1*qconstant;
     x_= x0;
     P_ = P0;
     F_ = F;
     H_ = H;
     R_ = R;
     Q_ = Q; 
}

void UnionsysEKF::predict()
{
    x_ = F_ * x_;  
    P_ = F_ * P_ * F_.transpose() + Q_;  
}

void UnionsysEKF::update(const Eigen::VectorXd& z)
{
    Eigen::VectorXd y = z - H_ * x_; 
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_; 
    K_ = P_ * H_.transpose() * S.inverse();
    x_ = x_ + K_ * y;  
    Eigen::MatrixXd I_KH = (Eigen::MatrixXd::Identity(P_.rows(), P_.rows()) - K_ * H_);
    P_ = I_KH * P_ * I_KH.transpose() + K_ * R_ * K_.transpose();  
}