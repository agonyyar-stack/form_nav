#include "form_nav/cloud_process.h"

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/segmentation/extract_clusters.h>

#include <pcl/search/kdtree.h>

namespace dwa_planner
{

PointCloudProcessor::PointCloudProcessor(ros::NodeHandle& nh): nh_(nh)
{
    // Parameters
    voxel_leaf_size_ = 0.4; //体素滤波的体素大小

    crop_min_x_ = -8.0; //裁剪区域的最小x坐标。裁剪区域即感兴趣区域，通常设置为无人机前方的一定范围内，以减少处理负担
    crop_max_x_ =  8.0;

    crop_min_y_ = -8.0;
    crop_max_y_ =  8.0;

    crop_min_z_ = -1.0;
    crop_max_z_ =  3.0;

    cluster_tolerance_ = 0.5; //聚类的距离阈值

    cluster_min_size_ = 10; //聚类的最小点数
    cluster_max_size_ = 1000; //聚类的最大点数

    dynamic_threshold_ = 0.15; //速度阈值，单位为m/s，超过这个速度的障碍物被认为是动态的

    // Subscriber
    // cloud_sub_ = nh_.subscribe("/livox/lidar", 1, &PointCloudProcessor::cloudCallback, this);
    // cloud_sub_ = nh_.subscribe("/dynamic_map_generator/global_cloud", 1, &PointCloudProcessor::cloudCallback, this);
    cloud_sub_ = nh_.subscribe("/global_cloud", 1, &PointCloudProcessor::cloudCallback, this);

    ROS_INFO("[PointCloudProcessor] Initialized.");
}

bool PointCloudProcessor::hasCloudData() const //检查是否有最新的点云数据可用
{
    std::lock_guard<std::mutex> lock(cloud_mutex_); //锁定互斥量，确保线程安全地访问latest_cloud_msg_

    return latest_cloud_msg_ != nullptr; //nullptr表示没有数据，非空表示有数据可用
}

std::vector<Obstacle> PointCloudProcessor::getObstacles()
{
    processPointCloud(); //调用处理流程，更新障碍物列表

    return obstacles_;
}

void PointCloudProcessor::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(cloud_mutex_);

    latest_cloud_msg_ = msg;
}

void PointCloudProcessor::processPointCloud() //处理点云数据，更新障碍物列表
{
    sensor_msgs::PointCloud2ConstPtr cloud_msg;

    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);

        cloud_msg = latest_cloud_msg_;
    }

    if (!cloud_msg)
    {
        return;
    }

    auto cloud = convertCloud(cloud_msg); //将ROS点云消息转换为PCL点云格式

    cloud = removeOutliers(cloud); //去除离群点，减少噪声对后续处理的影响

    cloud = voxelDownsample(cloud); //体素滤波，降低点云密度，提高处理效率，同时保留整体结构信息

    cloud = cropROI(cloud); //裁剪点云，保留感兴趣区域内的点，去除过远或过近的点

    auto clusters = extractClusters(cloud); //欧式聚类，将点云分割成多个簇，每个簇对应一个潜在的障碍物

    buildObstacles(clusters); //根据聚类结果构建障碍物对象，计算每个障碍物的中心位置和速度，并判断是否为动态障碍物
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
PointCloudProcessor::convertCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*msg, *cloud);

    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
PointCloudProcessor::removeOutliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

    sor.setInputCloud(cloud);

    sor.setMeanK(20);

    sor.setStddevMulThresh(1.0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    sor.filter(*filtered);

    return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
PointCloudProcessor::voxelDownsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;

    voxel_filter.setInputCloud(cloud);

    voxel_filter.setLeafSize(
        voxel_leaf_size_,
        voxel_leaf_size_,
        voxel_leaf_size_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    voxel_filter.filter(*filtered);

    return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
PointCloudProcessor::cropROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::PassThrough<pcl::PointXYZ> pass;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZ>);

    pass.setInputCloud(cloud);

    pass.setFilterFieldName("x");
    pass.setFilterLimits(crop_min_x_, crop_max_x_);
    pass.filter(*cropped);

    pass.setInputCloud(cropped);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(crop_min_y_, crop_max_y_);
    pass.filter(*cropped);

    pass.setInputCloud(cropped);

    pass.setFilterFieldName("z");
    pass.setFilterLimits(crop_min_z_, crop_max_z_);
    pass.filter(*cropped);

    return cropped;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
PointCloudProcessor::extractClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) 
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    ec.setClusterTolerance(cluster_tolerance_);

    ec.setMinClusterSize(cluster_min_size_);

    ec.setMaxClusterSize(cluster_max_size_);

    ec.setSearchMethod(tree);

    ec.setInputCloud(cloud);

    ec.extract(cluster_indices);

    for (const auto& indices : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);

        for (int idx : indices.indices)
        {
            cluster->points.push_back(cloud->points[idx]);
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    return clusters;
}

void PointCloudProcessor::buildObstacles(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters)
{
    obstacles_.clear(); 

    std::vector<Eigen::Vector3d> current_centers;

    ros::Time current_time = ros::Time::now();

    for (const auto& cluster : clusters)
    {
        Eigen::Vector3d center(0.0, 0.0, 0.0);

        for (const auto& p : cluster->points)
        {
            center.x() += p.x;
            center.y() += p.y;
            center.z() += p.z;
        }

        center /= cluster->points.size();

        Obstacle obs;

        obs.center = center;

        obs.cloud = cluster;

        Eigen::Vector3d velocity;

        obs.is_dynamic = detectDynamicObstacle(center,velocity);

        obs.velocity = velocity;

        obstacles_.push_back(obs);

        current_centers.push_back(center);
    }

    previous_centers_ = current_centers;

    previous_time_ = current_time;
}

bool PointCloudProcessor::detectDynamicObstacle(
    const Eigen::Vector3d& current_center,Eigen::Vector3d& estimated_velocity)
{
    estimated_velocity.setZero();

    if (previous_centers_.empty())
    {
        return false;
    }

    double min_dist = 1e9;

    Eigen::Vector3d matched_center;

    bool matched = false;

    for (const auto& prev : previous_centers_)
    {
        double dist = (current_center - prev).norm();

        if (dist < min_dist)
        {
            min_dist = dist;

            matched_center = prev;

            matched = true;
        }
    }

    if (!matched) {return false;}

    double dt = (ros::Time::now() - previous_time_).toSec();

    if (dt < 1e-3) {return false;}

    estimated_velocity =(current_center - matched_center) / dt;

    double speed = estimated_velocity.norm();

    return speed > dynamic_threshold_;
}

}
