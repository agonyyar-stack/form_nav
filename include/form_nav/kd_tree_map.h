#ifndef KDTREE_MAP_H
#define KDTREE_MAP_H

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "form_nav/kd_tree_two.h"

/**
 * @brief 基于 nanoflann 的障碍物 KD-tree 封装。
 *
 * 用途（第三步：MPC 静态避障）：
 *   1. 节点收到一帧障碍物点云（仿真阶段为 /map_generator/global_cloud，
 *      后续可替换为感知模块输出的 local cloud），降采样后调用 setCloud() 建树；
 *   2. MPC 每周期对预测时域内的查询点调用 queryNearest() 取最近 M 个障碍点，
 *      障碍点以参数形式进入 CasADi NLP（不在符号图内做树遍历）。
 *
 * 线程安全：setCloud() 与查询接口互斥，可在 ROS 回调线程与 MPC 求解线程
 * 之间安全共享。
 */
class KdTreeMap {
public:
    KdTreeMap();
    ~KdTreeMap() = default;

    /// 设置/重建障碍物 KD-tree（内部做体素降采样，voxel_size<=0 时不降采样）
    void setCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                  double voxel_size = 0.1);

    /// 清空地图
    void clear();

    /// 地图是否为空（无障碍点）
    bool empty() const;

    /// 当前障碍点数量
    size_t size() const;

    /**
     * @brief 查询 point 最近的 k 个障碍点（按距离升序）
     * @param point  查询点（世界坐标系）
     * @param k      期望的近邻数量
     * @param points 输出：近邻障碍点坐标（world）
     * @param squared_distances 输出：平方距离
     * @param indices 输出：近邻点在当前树中的索引（重规划循环用它判断最近点是否变化）
     * @return 实际查到的近邻数量（可能小于 k，地图点数不足时）
     */
    int queryNearest(const Eigen::Vector3d& point, int k,
                     std::vector<Eigen::Vector3d>& points,
                     std::vector<double>& squared_distances,
                     std::vector<int>& indices);

    /// 查询 point 的最近障碍距离（无点返回一个大数）
    double nearestDistance(const Eigen::Vector3d& point) const;

private:
    mutable std::mutex mutex_;
    std::shared_ptr<KDTreeTwo<double>> tree_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
};

#endif  // KDTREE_MAP_H
