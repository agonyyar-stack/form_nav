// fixed_obs.cpp
// 固定复杂障碍物（静态）场景生成节点
//
// 参考 random_obs.cpp 的发布方式（/map_generator/global_cloud、
// /map_generator/local_cloud，world 坐标系），改为完全确定性的手摆场景，
// 便于对不同算法策略（A* / kinodynamic A* / DWA / MPC 等）做定量对比：
// 同一场景、同一目标、每次运行完全一致。
//
// 场景默认参数（左边缘中心位于 (x,y)=(-20,0)）：
//   x ∈ [-20, 0]，y ∈ [-10, 10]，z ∈ [0, 3]   （20m x 20m x 3m）
//   场景左侧（起飞区）与右侧（目标区）保持空旷。
// 场景内仅包含立方体障碍物：高度 0.5~3m，底面长宽 0.5~3m（长宽可不相同），
// 无连续墙体；障碍物散乱分布、互不重叠，保留蜿蜒可行的通道。
// 共 30 个障碍物，其中大尺寸（底面≥2m 或高度≥2.5m）约 6 个，其余为中/小型。

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <string>
#include <vector>

ros::Publisher global_map_pub_;
ros::Publisher local_map_pub_;

double scene_left_x_ = -20.0;  // 场景左边缘 x
double scene_center_y_ = 0.0;  // 场景中心 y
double scene_size_x_ = 20.0;
double scene_size_y_ = 20.0;
double scene_size_z_ = 3.0;
double resolution_ = 0.2;
double pub_rate_ = 10.0;
std::string frame_id_ = "world";

struct Box
{
  double x0, x1;  // 世界坐标 x 范围
  double y0, y1;  // 世界坐标 y 范围
  double z0, z1;  // 世界坐标 z 范围
};

// 按全局栅格（分辨率 resolution_）把长方体体素化后加入点云
static void addBoxToCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, const Box& b)
{
  const int ix0 = static_cast<int>(std::floor(b.x0 / resolution_));
  const int ix1 = static_cast<int>(std::ceil(b.x1 / resolution_));
  const int iy0 = static_cast<int>(std::floor(b.y0 / resolution_));
  const int iy1 = static_cast<int>(std::ceil(b.y1 / resolution_));
  const int iz0 = static_cast<int>(std::floor(b.z0 / resolution_));
  const int iz1 = static_cast<int>(std::ceil(b.z1 / resolution_));

  for (int ix = ix0; ix < ix1; ++ix)
    for (int iy = iy0; iy < iy1; ++iy)
      for (int iz = iz0; iz < iz1; ++iz)
      {
        pcl::PointXYZ p;
        p.x = (ix + 0.5) * resolution_;
        p.y = (iy + 0.5) * resolution_;
        p.z = (iz + 0.5) * resolution_;
        cloud.points.push_back(p);
      }
}

// 单个立方体障碍物定义（相对场景左边缘 ox 与场景中心 y oy）
struct Cuboid
{
  double rx;  // 中心相对场景左边缘的 x 偏移 (m)
  double ry;  // 中心相对场景中心 y 的偏移 (m)
  double lx;  // 底面长 (m)
  double ly;  // 底面宽 (m)
  double h;   // 高度 (m)
};

// 构建固定复杂场景：仅立方体障碍物，散乱分布（无网格/行列对齐），
// 入口（x≈-19，y≈0）、出口（x≈-2，y≈0）附近留空，形成蜿蜒可行路径
static void buildFixedScene(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  const double ox = scene_left_x_;   // 场景左边缘
  const double oy = scene_center_y_; // 场景中心 y

  // 30 个立方体：{中心相对 ox 的 x, 中心相对 oy 的 y, 底面长, 底面宽, 高度}
  // 坐标刻意散乱（不落在等距网格上），大尺寸障碍物占少数
  const std::vector<Cuboid> cuboids = {
      // 左侧（入口附近）
      {0.8, -7.0, 1.3, 1.6, 1.6},
      {1.6, -4.8, 1.1, 1.3, 0.9},
      {2.2, 4.2, 1.6, 1.2, 2.4},
      {1.8, 7.6, 1.3, 1.6, 1.2},
      {3.6, -9.0, 1.6, 1.3, 2.0},
      // 中左
      {4.2, -6.2, 2.1, 1.5, 2.8},
      {4.8, -2.4, 1.2, 1.6, 1.8},
      {5.5, 1.5, 1.5, 1.4, 3.0},
      {4.6, 5.8, 1.3, 1.5, 1.0},
      {6.0, 8.2, 1.8, 1.2, 2.2},
      // 中部
      {7.6, -8.2, 1.5, 1.7, 1.4},
      {8.3, -5.0, 2.2, 1.4, 2.6},
      {7.2, -1.0, 1.2, 1.5, 2.0},
      {9.0, 2.4, 1.6, 1.3, 1.2},
      {8.6, 6.4, 1.4, 1.8, 2.4},
      {10.2, 8.8, 1.8, 1.5, 1.0},
      // 中右
      {11.5, -7.0, 1.3, 1.6, 2.2},
      {12.6, -3.6, 2.0, 1.3, 1.5},
      {13.6, -7.8, 1.4, 1.3, 2.8},
      {11.8, 3.6, 1.2, 1.6, 0.8},
      {13.8, 6.8, 1.7, 1.3, 1.9},
      {14.6, 9.0, 1.4, 1.2, 2.3},
      {14.2, 2.6, 1.3, 1.4, 2.0},
      // 右侧（出口附近）
      {15.8, -8.6, 1.6, 1.8, 1.3},
      {16.6, -5.4, 1.3, 1.5, 2.4},
      {15.4, -2.6, 1.8, 1.2, 1.7},
      {16.2, 5.2, 1.5, 1.3, 1.1},
      {18.4, 7.4, 1.6, 1.4, 2.6},
      {19.2, 4.6, 1.2, 1.5, 0.8},
      {19.0, -5.8, 1.3, 1.2, 1.6},
  };

  for (const auto& c : cuboids)
  {
    Box b;
    b.x0 = ox + c.rx - c.lx / 2.0;
    b.x1 = ox + c.rx + c.lx / 2.0;
    b.y0 = oy + c.ry - c.ly / 2.0;
    b.y1 = oy + c.ry + c.ly / 2.0;
    b.z0 = 0.0;
    b.z1 = c.h;
    addBoxToCloud(cloud, b);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fixed_obs");
  ros::NodeHandle n("~");

  n.param("scene/left_x", scene_left_x_, -20.0);
  n.param("scene/center_y", scene_center_y_, 0.0);
  n.param("scene/size_x", scene_size_x_, 20.0);
  n.param("scene/size_y", scene_size_y_, 20.0);
  n.param("scene/size_z", scene_size_z_, 3.0);
  n.param("scene/resolution", resolution_, 0.2);
  n.param("pub_rate", pub_rate_, 10.0);
  n.param("frame_id", frame_id_, std::string("world"));

  global_map_pub_ =
      n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);
  local_map_pub_ =
      n.advertise<sensor_msgs::PointCloud2>("/map_generator/local_cloud", 1);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  buildFixedScene(cloud);
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame_id_;
  msg.header.stamp = ros::Time::now();

  ROS_WARN("固定场景已生成: x=[%.2f, %.2f], y=[%.2f, %.2f], z=[0, %.2f], "
           "左边缘中心=(%.2f, %.2f), 共 %zu 个点",
           scene_left_x_, scene_left_x_ + scene_size_x_,
           scene_center_y_ - scene_size_y_ / 2.0,
           scene_center_y_ + scene_size_y_ / 2.0, scene_size_z_,
           scene_left_x_, scene_center_y_, cloud.points.size());

  ros::Rate loop_rate(pub_rate_);
  while (ros::ok())
  {
    msg.header.stamp = ros::Time::now();
    global_map_pub_.publish(msg);
    local_map_pub_.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
