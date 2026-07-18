#include <ros/ros.h>
#include "form_nav/cloud_process.h"

class DwaPlanner
{
public:
    DwaPlanner(ros::NodeHandle& nh)
        : cloud_processor_(nh)  // 初始化点云处理器
    {
        ROS_INFO("PointCloud Processor 启动成功！");
    }

    void run()
    {
        ros::Rate rate(10);  // 10Hz控制频率
        while (ros::ok())
        {
            ros::spinOnce();

            // ====================== 【核心调用】获取障碍物 ======================
            if (!cloud_processor_.hasCloudData())
            {
                ROS_WARN_THROTTLE(1, "等待点云数据...");
                rate.sleep();
                continue;
            }

            // 获取处理好的障碍物列表
            std::vector<dwa_planner::Obstacle> obstacles = cloud_processor_.getObstacles();
            ROS_INFO("当前检测到障碍物数量：%lu", obstacles.size());

            // ====================== 遍历障碍物，使用信息 ======================
            for (size_t i = 0; i < obstacles.size(); i++)
            {
                const dwa_planner::Obstacle& obs = obstacles[i];
                // 1. 障碍物质心（x,y,z）
                double x = obs.center.x();
                double y = obs.center.y();
                double z = obs.center.z();

                // 2. 是否动态障碍物
                bool is_dynamic = obs.is_dynamic;

                // 3. 障碍物速度（vx, vy, vz）
                double vx = obs.velocity.x();
                double vy = obs.velocity.y();

                // 4. 打印信息（调试用）
                ROS_INFO("障碍[%lu]：位置(%.2f, %.2f, %.2f) | 速度(%.2f, %.2f) | %s",
                         i, x, y, z, vx, vy, is_dynamic ? "动态" : "静态");
            }

            rate.sleep();
        }
    }

private:
    // 点云处理器对象
    dwa_planner::PointCloudProcessor cloud_processor_;
};

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "cloud_process_node");
    ros::NodeHandle nh;

    DwaPlanner planner(nh);
    planner.run();

    return 0;
}