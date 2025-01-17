#ifndef _UNIARC_MAIN_CPP_
#define _UNIARC_MAIN_CPP_

#include "uniarc_main.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uniarc_main"); // 初始化ros 节点
    ros::NodeHandle n; // 创建node控制句柄
    BasicDev go(&n);
    ros::spin();
    return 0;
}

BasicDev::BasicDev(ros::NodeHandle *nh)
{  
    // 创建图像传输控制句柄
    it = std::make_unique<image_transport::ImageTransport>(*nh); 
    front_left_img = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0));
    front_right_img = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0));

    takeoff.request.waitOnLastTask = 1;
    land.request.waitOnLastTask = 1;

    // 初始化速度指令
    velcmd.twist.angular.z = 0; // z方向角速度(yaw, deg)
    velcmd.twist.linear.x = 0;  // x方向线速度(m/s)
    velcmd.twist.linear.y = 0;  // y方向线速度(m/s)
    velcmd.twist.linear.z = 0;  // z方向线速度(m/s)

    // 初始化PWM命令
    pwm_cmd.rotorPWM0 = 0.1;
    pwm_cmd.rotorPWM1 = 0.1;
    pwm_cmd.rotorPWM2 = 0.1;
    pwm_cmd.rotorPWM3 = 0.1;

    // 无人机信息订阅
    vins_suber = nh->subscribe<nav_msgs::Odometry>(
        "/vins_estimator/odometry", 1000, 
        std::bind(&BasicDev::odometry_cb, this, std::placeholders::_1)
    );

    odom_suber = nh->subscribe<geometry_msgs::PoseStamped>(
        "/airsim_node/drone_1/debug/pose_gt", 1, 
        std::bind(&BasicDev::pose_cb, this, std::placeholders::_1)
    ); // 状态真值，用于赛道一
    
    gps_suber = nh->subscribe<geometry_msgs::PoseStamped>(
        "/airsim_node/drone_1/gps", 1, 
        std::bind(&BasicDev::gps_cb, this, std::placeholders::_1)
    ); // 状态真值，用于赛道一
    
    imu_suber = nh->subscribe<sensor_msgs::Imu>(
        "airsim_node/drone_1/imu/imu", 1, 
        std::bind(&BasicDev::imu_cb, this, std::placeholders::_1)
    ); // imu数据
    
    lidar_suber = nh->subscribe<sensor_msgs::PointCloud2>(
        "airsim_node/drone_1/lidar", 1, 
        std::bind(&BasicDev::lidar_cb, this, std::placeholders::_1)
    ); // lidar数据

    front_left_view_suber = it->subscribe(
        "airsim_node/drone_1/front_left/Scene", 1, 
        std::bind(&BasicDev::front_left_view_cb, this,  std::placeholders::_1)
    );
    
    front_right_view_suber = it->subscribe(
        "airsim_node/drone_1/front_right/Scene", 1, 
        std::bind(&BasicDev::front_right_view_cb, this,  std::placeholders::_1)
    );

    // 通过这两个服务可以调用模拟器中的无人机起飞和降落命令
    takeoff_client = nh->serviceClient<airsim_ros::Takeoff>(
        "/airsim_node/drone_1/takeoff"
    );
    land_client = nh->serviceClient<airsim_ros::Takeoff>(
        "/airsim_node/drone_1/land"
    );
    reset_client = nh->serviceClient<airsim_ros::Reset>(
        "/airsim_node/reset"
    );

    // 通过publisher实现对无人机的控制

    front_left_pub = it->advertise("/uniarc/leftimg", 1);
    front_right_pub = it->advertise("/uniarc/rightimg", 1);

    vel_publisher = nh->advertise<airsim_ros::VelCmd>(
        "airsim_node/drone_1/vel_cmd_body_frame", 1
    );
    pwm_publisher = nh->advertise<airsim_ros::RotorPWM>(
        "airsim_node/drone_1/rotor_pwm_cmd", 1
    );

    // takeoff_client.call(takeoff); // 起飞
    // land_client.call(land); // 降落
    // reset_client.call(reset); // 重置
    takeoff_client.call(takeoff);
    ros::spin();
}

BasicDev::~BasicDev()
{
}

void BasicDev::odometry_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 输出收到的位姿信息
    ROS_INFO("Received Odometry data:");
    ROS_INFO("Position -> x: %f, y: %f, z: %f", 
        msg->pose.pose.position.x, 
        msg->pose.pose.position.y, 
        msg->pose.pose.position.z);
    ROS_INFO("Orientation -> x: %f, y: %f, z: %f, w: %f", 
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
}

void BasicDev::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d eulerAngle = q.matrix().eulerAngles(2,1,0);
    // ROS_INFO("Get pose data. time: %f, eulerangle: %f, %f, %f, posi: %f, %f, %f\n", 
    //     msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
    //     eulerAngle[0], eulerAngle[1], eulerAngle[2], 
    //     msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void BasicDev::gps_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d eulerAngle = q.matrix().eulerAngles(2,1,0);
    // ROS_INFO("Get gps data. time: %f, eulerangle: %f, %f, %f, posi: %f, %f, %f\n", 
    //     msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
    //     eulerAngle[0], eulerAngle[1], eulerAngle[2], 
    //     msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void BasicDev::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    // ROS_INFO("Get imu data. time: %f", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9);
}

void BasicDev::front_left_view_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_front_left_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    if (!cv_front_left_ptr->image.empty())
    {
        ROS_INFO("Received front left image at time: %f", 
                 msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9);
        
        // **创建ROS图像消息并发布**
        sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(msg->header, "bgr8", cv_front_left_ptr->image).toImageMsg();
        front_left_pub.publish(out_msg);
    }
}

void BasicDev::front_right_view_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_front_right_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    if (!cv_front_right_ptr->image.empty())
    {
        ROS_INFO("Received front right image at time: %f", 
                 msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9);
        
        // **创建ROS图像消息并发布**
        sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(msg->header, "bgr8", cv_front_right_ptr->image).toImageMsg();
        front_right_pub.publish(out_msg);
    }
}

void BasicDev::lidar_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pts(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pts);
    // ROS_INFO("Get lidar data. time: %f, size: %ld", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9, pts->size());
}

#endif
