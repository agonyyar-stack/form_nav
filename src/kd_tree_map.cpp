#include "form_nav/kd_tree_map.h"

#include <pcl/filters/voxel_grid.h>

#include <limits>

KdTreeMap::KdTreeMap()
    : tree_(std::make_shared<KDTreeTwo<double>>()),
      cloud_(new pcl::PointCloud<pcl::PointXYZ>()) {}

void KdTreeMap::setCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                         double voxel_size) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(
        new pcl::PointCloud<pcl::PointXYZ>());
    if (voxel_size > 0.0 && cloud && !cloud->empty()) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxel.filter(*downsampled);
    } else if (cloud) {
        *downsampled = *cloud;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    *cloud_ = *downsampled;
    tree_->InitializeNew(cloud_);
}

void KdTreeMap::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    cloud_->clear();
    tree_->Clear();
}

bool KdTreeMap::empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cloud_->empty();
}

size_t KdTreeMap::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cloud_->size();
}

int KdTreeMap::queryNearest(const Eigen::Vector3d& point, int k,
                            std::vector<Eigen::Vector3d>& points,
                            std::vector<double>& squared_distances,
                            std::vector<int>& indices) {
    points.clear();
    squared_distances.clear();
    indices.clear();

    std::lock_guard<std::mutex> lock(mutex_);
    if (cloud_->empty() || k <= 0) {
        return 0;
    }

    tree_->SearchForNearest(point.x(), point.y(), point.z(), k);
    const std::vector<pcl::PointXYZ>& closest = tree_->closest_pts;
    const std::vector<double>& dists = tree_->squared_distances;
    const std::vector<int>& idx = tree_->indices;

    const int n = static_cast<int>(closest.size());
    for (int i = 0; i < n; ++i) {
        points.emplace_back(closest[i].x, closest[i].y, closest[i].z);
        squared_distances.push_back(dists[i]);
        indices.push_back(idx[i]);
    }
    return n;
}

double KdTreeMap::nearestDistance(const Eigen::Vector3d& point) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (cloud_->empty()) {
        return std::numeric_limits<double>::max();
    }
    tree_->SearchForNearest(point.x(), point.y(), point.z(), 1);
    if (tree_->squared_distances.empty()) {
        return std::numeric_limits<double>::max();
    }
    return std::sqrt(tree_->squared_distances[0]);
}
