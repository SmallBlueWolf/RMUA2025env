#include "controllerTest.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_test");
    ros::NodeHandle n; // 创建node控制句柄
    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    g_triggerport_client = n.serviceClient<airsim_ros::TriggerPort>("/airsim_node/drone_1/trigger_port");
    g_takeoff_client = n.serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    g_pwm_publisher = n.advertise<airsim_ros::RotorPWM>("/airsim_node/drone_1/rotor_pwm_cmd", 1);
    ros::Subscriber odom_suber = n.subscribe<nav_msgs::Odometry>("/eskf_odom", 1, odom_cb);
    // ros::Subscriber gt_suber = n.subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/debug/pose_gt", 1, gt_cb);
    ros::Subscriber pos_cmd_sub = n.subscribe<quadrotor_msgs::PositionCommand>("/drone_0_planning/pos_cmd", 1, posCmdCallback);
    ros::Subscriber init_pose_suber = n.subscribe<geometry_msgs::PoseStamped>("/airsim_node/initial_pose", 1, init_pose_cb);
    ros::Subscriber end_pose_suber = n.subscribe<geometry_msgs::PoseStamped>("/airsim_node/end_goal", 1, end_position_cb);
    ros::Timer timer = n.createTimer(ros::Duration(1.0), timeCB);
    airsim_ros::Takeoff  tf_cmd;
    ros::spinOnce();
    startEgoPlanner();
    tf_cmd.request.waitOnLastTask = 1;
    g_takeoff_client.call(tf_cmd);
    ros::Rate loop_rate(200);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void startEgoPlanner() {

    while(!get_init_pose || !get_end_goal)
    {
        ros::spinOnce();  // 确保回调函数得到处理
        ros::Duration(0.1).sleep();  // 等待一定时间，确保数据接收
        std::cout << "Waiting for initial and end pose..." << std::endl;
        std::cout << init_end_poses[0] << " " <<
        init_end_poses[1] << " " <<
        init_end_poses[2] << " " <<
        init_end_poses[3] << " " <<
        init_end_poses[4] << " " <<
        init_end_poses[5] << " " << std::endl;
    }


    std::string command = std::string("bash -c 'cd /uniarc_main/src && . /uniarc_main/devel/setup.bash && roslaunch ego_planner uniarc.launch ") +
                          "init_x:=" + std::to_string(init_end_poses[0]) + " " +
                          "init_y:=" + std::to_string(init_end_poses[1]) + " " +
                          "init_z:=" + std::to_string(init_end_poses[2]) + " " +
                          "target_x:=" + std::to_string(init_end_poses[3]) + " " +
                          "target_y:=" + std::to_string(init_end_poses[4]) + " " +
                          "target_z:=" + std::to_string(init_end_poses[5]) + " &'";

    std::cout << "Executing: " << command << std::endl;
    int ret = system(command.c_str());
    if (ret == 0) {
        std::cout << "EGO-Planner launched successfully in the background." << std::endl;
    } else {
        std::cerr << "Failed to launch EGO-Planner." << std::endl;
    }
}

void posCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    Eigen::Vector3d pos(msg->position.x, msg->position.y, msg->position.z);
    Eigen::Vector3d vel(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    Eigen::Vector3d acc(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
    double yaw = msg->yaw;
    double yaw_dot = msg->yaw_dot;
    pos_received[0] = pos(0);
    pos_received[1] = pos(1);
    pos_received[2] = pos(2);
    pos_received[3] = vel(0);
    pos_received[4] = vel(1);
    pos_received[5] = vel(2);
    pos_received[6] = acc(0);
    pos_received[7] = acc(1);
    pos_received[8] = acc(2);
    pos_received[9] = yaw;
    pos_received[10] = yaw_dot;
    isgetpos = true;
}

void timeCB(const ros::TimerEvent& event)
{
    if(cb_cnt / 100 < 11)return;
    airsim_ros::TriggerPort cmd;
    cmd.request.port = trigger_port;
    cmd.request.enter = 1;
    g_triggerport_client.call(cmd);
    trigger_port += 1;
}

void init_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Quaternion Q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Matrix3d rotationM = Q.normalized().toRotationMatrix();
    // std::cout<<"initial pose: \n"<<rotationM<<std::endl;
    Eigen::Vector3d pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    // std::cout<<"initial pos: "<<pos.transpose()<<std::endl;
    Tw0 = Eigen::Matrix4d::Identity();
    Tw0.block(0, 0, 3, 3)=rotationM;
    Tw0.block(0, 3, 3 ,1) = pos;
    Twb_last = Tw0;
    // std::cout<<"Tw0:\n"<<Tw0<<std::endl;

    init_end_poses[0] = msg->pose.position.x;
    init_end_poses[1] = msg->pose.position.y;
    init_end_poses[2] = msg->pose.position.z;
    get_init_pose = true;
}

void end_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Pwend = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    // std::cout<<"end pos: "<<Pwend.transpose()<<std::endl;
    if(get_init_pose && !get_end_goal)
    {
        for(auto ps: globalPaths)
        {
            if((ps[0]-Tw0.block(0, 3, 3, 1)).norm() < 10)
            {
                for(int i = 0; i < ps.size(); i++)
                {
                    globalPath.emplace_back(ps[i]);
                }
                break;
            }
        }
        for(auto ps: globalPaths)
        {
            if((ps[0]-Pwend).norm() < 10)
            {
                for(int i = 0; i < ps.size(); i++)
                {
                    globalPath.emplace_back(ps[ps.size()-i]);
                }
                break;
            }
        }
        init_end_poses[3] = msg->pose.position.x;
        init_end_poses[4] = msg->pose.position.y;
        init_end_poses[5] = msg->pose.position.z;
        get_end_goal = true;
    }
}

// void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
// {
//     cb_cnt ++;
//     if(cb_cnt / 100 < 10)return;
//     if(! get_init_pose)return;
//     Eigen::Quaternion Q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
//     Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity();
//     Twb.block(0, 0, 3 ,3) = Q.normalized().toRotationMatrix();
//     Twb(0, 3) = msg->pose.pose.position.x;
//     Twb(1, 3) = msg->pose.pose.position.y;
//     Twb(2, 3) = msg->pose.pose.position.z;
//     // std::cout<<"Twb rt:\n"<<Twb<<std::endl;
//     // std::cout<<phi<<" "<<theta<<" "<<psi<<std::endl;
//     Eigen::VectorXf X_des, X_real;
//     X_des.resize(12);
//     X_real.resize(12);
//     // Twb_last = Twb;
//     // std::cout<<"Tw0:\n"<<Tw0<<std::endl;
//     Eigen::Matrix4d TWfluWned;
//     TWfluWned << 1, 0, 0, 0, 
//                 0, -1, 0, 0,
//                 0, 0, -1, 0, 
//                 0, 0, 0, 1;
//     Eigen::Matrix4d TWflu0 = TWfluWned * Tw0 * TWfluWned.inverse();
//     Eigen::Matrix4d TWflub = TWfluWned * Twb * TWfluWned.inverse();
//     Eigen::Matrix4d T0flub = TWflu0.inverse() * TWflub;
//     Eigen::Vector3d VWned(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
//     Eigen::Vector3d VBned = Twb.block(0, 0, 3 ,3).inverse() * VWned;
//     Eigen::Vector3d VBflu = TWfluWned.block<3, 3>(0, 0) * VBned;
//     Eigen::Vector3d Wned(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
//     Eigen::Vector3d Wflu = TWfluWned.block<3, 3>(0, 0) * Wned;
//     const float phi = std::asin(T0flub(2, 1));
//     const float theta = std::atan2(-T0flub(2, 0)/std::cos(phi), T0flub(2, 2)/std::cos(phi));
//     const float psi = std::atan2(-T0flub(0, 1)/std::cos(phi), T0flub(1, 1)/std::cos(phi));
//     // X_real<<T0flub(0, 3), T0flub(1, 3), T0flub(2, 3), 
//     //     VBflu.x(), VBflu.y(), VBflu.z(), 
//     //     phi, theta, psi, Wflu.x(), Wflu.y(), Wflu.z();

//     X_real<< msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, 
//         VBflu.x(), VBflu.y(), VBflu.z(), 
//         phi, theta, psi, Wflu.x(), Wflu.y(), Wflu.z();

//     while(!isgetpos){}

//     X_des << pos_received[0], pos_received[1], pos_received[2], 
//     pos_received[3], pos_received[4], pos_received[5],
//     phi, theta, pos_received[9],
//     Wflu.x(), Wflu.y(), Wflu.z();

//     // 赋值 X_real 之后插入打印语句
//     std::cout << "X_real: : " << X_real(0) << ", " << X_real(1) << ", " << X_real(2) << ", " << X_real(3) << ", " << X_real(4) << ", " << X_real(5) << ", " << X_real(6) << ", " << X_real(7) << ", " << X_real(8) << ", " << X_real(9) << ", " << X_real(10) << ", " << X_real(11) << std::endl<<std::endl;
//     std::cout << "X_des: : " << X_des(0) << ", " << X_des(1) << ", " << X_des(2) << std::endl<<std::endl;

//     // X_des << 1.0, 1.0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0;
//     Eigen::Vector4f output = g_PDcontroller.execute( X_des, X_real);
//     airsim_ros::RotorPWM pwm_cmd;
//     pwm_cmd.rotorPWM0 = output[0];
//     pwm_cmd.rotorPWM1 = output[1];
//     pwm_cmd.rotorPWM2 = output[2];
//     pwm_cmd.rotorPWM3 = output[3];
//     // // std::cout<<pwm_cmd.rotorPWM0<<" "<<pwm_cmd.rotorPWM1<<" "<<pwm_cmd.rotorPWM2<<" "<<pwm_cmd.rotorPWM3<<" "<<std::endl;
//     if(X_real[0]<3.0)
//     {
//         g_pwm_publisher.publish(pwm_cmd);
//     }else
//     {
//         if(trigger_port > 12)
//             trigger_port = 0;
//     }

// }

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    cb_cnt++;
    if (cb_cnt / 100 < 10) return;
    if (!get_init_pose) return;

    Eigen::Quaternion Q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity();
    Twb.block(0, 0, 3, 3) = Q.normalized().toRotationMatrix();
    Twb(0, 3) = msg->pose.pose.position.x;
    Twb(1, 3) = msg->pose.pose.position.y;
    Twb(2, 3) = msg->pose.pose.position.z;

    Eigen::VectorXf X_des, X_real;
    X_des.resize(12);
    X_real.resize(12);

    Eigen::Matrix4d TWfluWned;
    TWfluWned << 1, 0, 0, 0,
                 0, -1, 0, 0,
                 0, 0, -1, 0,
                 0, 0, 0, 1;
    Eigen::Matrix4d TWflu0 = TWfluWned * Tw0 * TWfluWned.inverse();
    Eigen::Matrix4d TWflub = TWfluWned * Twb * TWfluWned.inverse();
    Eigen::Matrix4d T0flub = TWflu0.inverse() * TWflub;
    
    Eigen::Vector3d VWned(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Eigen::Vector3d VBned = Twb.block(0, 0, 3, 3).inverse() * VWned;
    Eigen::Vector3d VBflu = TWfluWned.block<3, 3>(0, 0) * VBned;
    Eigen::Vector3d Wned(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    Eigen::Vector3d Wflu = TWfluWned.block<3, 3>(0, 0) * Wned;
    
    const float phi = std::asin(T0flub(2, 1));
    const float theta = std::atan2(-T0flub(2, 0) / std::cos(phi), T0flub(2, 2) / std::cos(phi));
    const float psi = std::atan2(-T0flub(0, 1) / std::cos(phi), T0flub(1, 1) / std::cos(phi));
    
    X_real << T0flub(0, 3), T0flub(1, 3), T0flub(2, 3),
              VBflu.x(), VBflu.y(), VBflu.z(),
              phi, theta, psi, Wflu.x(), Wflu.y(), Wflu.z();

    // 修改 X_des 赋值
    while (!isgetpos) {}

    // 将 pos_received 的数据作为目标位置，并进行变换，更新目标位置 X_des
    Eigen::Vector3d transformed_pos = TWfluWned.block<3, 3>(0, 0) * Eigen::Vector3d(pos_received[0], pos_received[1], pos_received[2]);
    
    X_des << transformed_pos.x(), transformed_pos.y(), transformed_pos.z(),
             pos_received[3], pos_received[4], pos_received[5],
             phi, theta, pos_received[9],
             Wflu.x(), Wflu.y(), Wflu.z();

    // 继续后续处理
    std::cout << "X_real: " << X_real.transpose() << std::endl;
    std::cout << "X_des: " << X_des.transpose() << std::endl << std::endl;

    Eigen::Vector4f output = g_PDcontroller.execute(X_des, X_real);
    airsim_ros::RotorPWM pwm_cmd;
    pwm_cmd.rotorPWM0 = output[0];
    pwm_cmd.rotorPWM1 = output[1];
    pwm_cmd.rotorPWM2 = output[2];
    pwm_cmd.rotorPWM3 = output[3];
    
    if (X_real[0] < 3.0) {
        g_pwm_publisher.publish(pwm_cmd);
    } else {
        if (trigger_port > 12)
            trigger_port = 0;
    }
}
