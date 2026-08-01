// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h> //header pose(位置Point和朝向四元数Quaternion)
// #include <mavros_msgs/CommandBool.h> // 用于arming（解锁/上锁）的服务请求
// #include <mavros_msgs/SetMode.h>  //服务
// #include <mavros_msgs/State.h> // 消息 飞控实时状态话题 包括connected,armed,mode
 
// mavros_msgs::State current_state; //保存最近接收到的飞控状态
// // std::string uav_id = "uav5";

// void state_cb(const mavros_msgs::State::ConstPtr& msg)
// {
//     current_state = *msg;
// }
 
// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "uav_base");
//     ros::NodeHandle nh;
 
//     ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
            
//     ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

//     ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
//     ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
 
//     //PX4 要求 OFFBOARD 模式下必须持续收到 setpoint，且频率 > 2 Hz
//     ros::Rate rate(20.0f);
 
//     // 启动前等待连接
//     while(ros::ok() && !current_state.connected) //ros运行且未连接时继续循环
//     {
//         ros::spinOnce(); //消息回调处理函数，若无新消息则继续执行后面代码
//         rate.sleep(); //控制循环的执行频率，避免 CPU 疯狂占用
//     }
 
//     geometry_msgs::PoseStamped pose;
//     pose.pose.position.z = 1; //起飞到该位置先悬停
    
//     //PX4 要求进入 OFFBOARD 前已经收到至少 2 秒以上的 setpoint
//     for(int i = 100; ros::ok() && i > 0; --i) 
//     {
//         local_pos_pub.publish(pose);
//         ros::spinOnce();
//         rate.sleep();
//     }//可以确保飞控缓存中已经有稳定的 setpoint 数据
 
//     mavros_msgs::SetMode offb_set_mode;
//     offb_set_mode.request.custom_mode = "OFFBOARD"; 
 
//     mavros_msgs::CommandBool arm_cmd;
//     arm_cmd.request.value = true;
 
//     ros::Time last_request = ros::Time::now(); //定义时间戳，用来节流
//     //如果飞控在 500ms 内没有收到新的 setpoint，PX4 会自动退出 OFFBOARD 模式
//     //如果飞控检测到状态异常（如姿态过大、传感器异常），PX4 会自动上锁
//     while(ros::ok()) //主循环：模式切换，解锁，发布任务
//     {
//         if( current_state.mode != "OFFBOARD" &&  //每五秒尝试一次切换模式，优先级高
//             (ros::Time::now() - last_request > ros::Duration(5.0f))) 
//         {                                         //ROS节点 → MAVROS → MAVLink → PX4
//             if( set_mode_client.call(offb_set_mode) && //.call()发送模式切换请求等待响应
//                 offb_set_mode.response.mode_sent) //模式是否切换成功
//             {
//                 ROS_INFO("Offboard enabled");
//             }
//             last_request = ros::Time::now();
//         }
//         else 
//         {
//             if( !current_state.armed && //每五秒尝试解锁
//                 (ros::Time::now() - last_request > ros::Duration(5.0f)))
//             {
//                 if( arming_client.call(arm_cmd) && arm_cmd.response.success)
//                 {
//                     ROS_INFO("Vehicle armed");
//                 }
//                 last_request = ros::Time::now();
//             }
//         }
 
//         ros::spinOnce();
//         rate.sleep();
//     }
 
//     return 0;
// }
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Empty.h>

class DroneController {
private:
    ros::NodeHandle nh;
    ros::Subscriber state_sub;
    ros::Subscriber vel_sub;
    ros::Publisher local_pos_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    
    mavros_msgs::State current_state;
    bool goal_received = false;
    bool velocity_cmd_received = false;
    geometry_msgs::PoseStamped goal_pose;

public:
    DroneController() {
        // 订阅飞控状态
        state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &DroneController::state_cb, this);
        
        // 服务客户端
        arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        
        // 本地位置指令发布
        local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10, &DroneController::velocity_cb, this);
        
        // 等待MAVROS连接
        ROS_INFO("等待MAVROS连接...");
        while (ros::ok() && !current_state.connected) {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        ROS_INFO("MAVROS已连接");

        // 初始化悬停目标点
        goal_pose.header.frame_id = "map";
        // goal_pose.pose.position.x = 0;
        // goal_pose.pose.position.y = 0;
        goal_pose.pose.position.z = 1.0;
        // goal_pose.pose.orientation.w = 1.0;
        goal_received = true; //确保进入OFFBOARD前已经有目标点数据
        
        ROS_INFO("开始解锁和模式切换流程...");
    }

    // 飞控状态回调
    void state_cb(const mavros_msgs::State::ConstPtr& msg) {
        current_state = *msg;
    }

    // 速度指令回调：仅在无人机已解锁(armed)后，收到第一个速度指令才停止发布位置 setpoint。
    // 解锁前必须持续发布位置 setpoint（PX4 进入 OFFBOARD 的前置条件），
    // 否则 mpc_node 未就绪时发出的零速度指令会提前中断 setpoint 流，导致无法解锁起飞。
    void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        if (!current_state.armed) {
            return;
        }
        if (!velocity_cmd_received) {
            velocity_cmd_received = true;
            ROS_INFO("已解锁,收到第一个速度指令,停止发布位置 setpoint,切换为速度控制");
        }
    }

    // 解锁请求
    bool arm_drone(bool enable) {
        mavros_msgs::CommandBool srv;
        srv.request.value = enable;
        if (arming_client.call(srv)) {
            return srv.response.success;
        }
        return false;
    }

    // 切换OFFBOARD模式
    bool set_offboard_mode() {
        mavros_msgs::SetMode srv;
        srv.request.custom_mode = "OFFBOARD";
        if (set_mode_client.call(srv)) {
            return srv.response.mode_sent;
        }
        return false;
    }

    // 发布定点指令
    void publish_position_setpoint() {
        goal_pose.header.stamp = ros::Time::now();
        // PX4本地控制必须使用 local_origin 坐标系
        goal_pose.header.frame_id = "local_origin";
        local_pos_pub.publish(goal_pose);
    }

    void run() {
        ros::Rate rate(20.0); // 20Hz 高频发布
        
        // 预发布足够多的定点指令（OFFBOARD前置必要条件）
        ROS_INFO("持续发送位置指令");
        for (int i = 0; i < 200; ++i) {
            publish_position_setpoint();
            ros::spinOnce();
            rate.sleep();
        }

        // 循环持续请求切换模式 + 等待飞控实际刷新模式状态
        ROS_INFO("正在切换至OFFBOARD模式...");
        while (ros::ok() && current_state.mode != "OFFBOARD") {
            set_offboard_mode();  // 持续下发模式请求
            publish_position_setpoint();
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("OFFBOARD模式 实际生效成功");

        // 循环持续请求解锁 + 等待解锁状态生效
        ROS_INFO("正在解锁无人机...");
        while (ros::ok() && !current_state.armed) {
            arm_drone(true);      // 持续下发解锁请求
            publish_position_setpoint();
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("无人机解锁成功");
        
        ROS_INFO("===== 无人机就绪 =====");
        
        // 循环保持控制：如果未收到速度命令则继续发布位置指令
        while (ros::ok()) {
            if (!velocity_cmd_received) {
                publish_position_setpoint();
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "uav_base");
    DroneController controller;
    controller.run();
    return 0;
}