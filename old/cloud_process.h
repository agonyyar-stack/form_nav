#ifndef UAV_DWA_PLANNER_POINTCLOUD_PROCESSOR_H
#define UAV_DWA_PLANNER_POINTCLOUD_PROCESSOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Dense>

#include <mutex>
#include <vector>

namespace dwa_planner
{

struct Obstacle
{
    Eigen::Vector3d center;
    Eigen::Vector2d center2d() const
    {
        return center.head<2>(); //head<>表示取前n个元素，这里取前2个元素，即x和y坐标
    }
    Eigen::Vector3d velocity;
    Eigen::Vector2d velocity2d() const
    {
        return velocity.head<2>(); //head<>表示取前n个元素，这里取前2个元素，即x和y坐标
    }

    bool is_dynamic;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; //存储障碍物对应的点云数据

    Obstacle() //默认构造函数，初始化成员变量
    {
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>); //创建一个新的点云对象，并将智能指针指向它
        velocity.setZero(); //将速度向量初始化为零
        is_dynamic = false;
    }
};

class PointCloudProcessor
{
public:

    explicit PointCloudProcessor(ros::NodeHandle& nh); //explicit关键字表示这个构造函数只能通过显式调用来使用，防止隐式类型转换导致的错误

    ~PointCloudProcessor() = default;

    bool hasCloudData() const;

    std::vector<Obstacle> getObstacles();

private:

    // ROS
    ros::NodeHandle nh_;

    ros::Subscriber cloud_sub_;

    // Point cloud cache
    sensor_msgs::PointCloud2ConstPtr latest_cloud_msg_;

    mutable std::mutex cloud_mutex_;

    // Obstacle storage
    std::vector<Obstacle> obstacles_;

    // Parameters
    double voxel_leaf_size_;

    double crop_min_x_;
    double crop_max_x_;

    double crop_min_y_;
    double crop_max_y_;

    double crop_min_z_;
    double crop_max_z_;

    double cluster_tolerance_;

    int cluster_min_size_;

    int cluster_max_size_;

    double dynamic_threshold_;

private:

    // ROS callback
    void cloudCallback(
        const sensor_msgs::PointCloud2ConstPtr& msg);

    // Main processing pipeline
    void processPointCloud();

    // Convert ROS cloud to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertCloud(
        const sensor_msgs::PointCloud2ConstPtr& msg);

    // Remove outliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // Downsample cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDownsample(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // ROI crop
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropROI(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // Cluster extraction
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
    extractClusters(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // Generate obstacle objects
    void buildObstacles(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);

    // Dynamic obstacle detection
    bool detectDynamicObstacle(
        const Eigen::Vector3d& current_center,
        Eigen::Vector3d& estimated_velocity);

private:

    // Previous obstacle centers
    std::vector<Eigen::Vector3d> previous_centers_;

    ros::Time previous_time_;
};

} // namespace dwa_planner

#endif