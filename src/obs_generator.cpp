// obs_generator.cpp
// 场景生成节点（静态 + 动态障碍物），配置由 YAML 场景文件驱动
//
// 由 scene_editor.py（可视化编辑器）生成/编辑 YAML 场景文件，本节点在仿真
// 启动时加载并发布：
//   /map_generator/global_cloud、/map_generator/local_cloud —— 静态障碍物
//     （cuboid/cylinder 体素化点云，A*/静态 KD-tree 使用）
//   /map_generator/dynamic_cloud —— 动态障碍物点云（仅可视化，不进规划）
//   /dynamic_obstacles/trajectory —— 动态障碍物预测轨迹（MPC 动态避障输入）
//
// 动态障碍物仅支持直线往复运动：
//   p(t) = center + A*sin(ωt + φ)*dir，A = length/2，ω = v_max / A
//   speed_max 默认 1.5 m/s（YAML 可配置），方向/长度/位置/形状由 YAML 自定义。
//
// 启动时对每个动态障碍物的扫掠区域（AABB + 半径膨胀）与静态障碍物做相交
// 检查，冲突则打印 WARN 便于提前发现布局问题。

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "form_nav/DynamicObstacleTrajectory.h"

ros::Publisher global_map_pub_;
ros::Publisher local_map_pub_;
ros::Publisher dyn_map_pub_;   // 动态障碍物点云（仅可视化）
ros::Publisher dyn_traj_pub_;  // 动态障碍物预测轨迹（MPC 避障输入）

// 场景参数（可由 YAML scene 字段覆盖）
double scene_left_x_ = -20.0;
double scene_center_y_ = 0.0;
double scene_size_x_ = 20.0;
double scene_size_y_ = 20.0;
double scene_size_z_ = 3.0;
double resolution_ = 0.2;
double pub_rate_ = 10.0;
std::string frame_id_ = "world";
std::string scene_file_;

// 动态障碍物轨迹发布参数（与 MPC 网格对齐）
double dyn_traj_dt_ = 0.05;
int dyn_traj_steps_ = 21;
int dyn_obs_num_ = -1;  // -1=全部使用 YAML 中的定义, 0=关闭, n>0=只取前 n 个

// ============ 障碍物数据模型 ============
struct StaticObstacle
{
  int shape;      // 0=cuboid, 1=cylinder
  double x, y;    // 中心（世界系，底面 z=0）
  double lx, ly;  // cuboid 底面长宽 (m)
  double h;       // cuboid 高度 / cylinder 高度 (m)
  double radius;  // cylinder 半径 (m)
};

struct DynamicObstacle
{
  int shape;        // 0=球体, 1=圆柱
  double radius;    // 障碍物半径 (m)
  double height;    // 圆柱高度 (m)，球体填 0
  double z_center;  // 形状中心高度 (m)：圆柱 = H/2，球体 = 球心高度
  double cx, cy;    // 运动中心（世界系）
  double direction; // 运动方向角 (rad)，0=+x, π/2=+y
  double amplitude; // 直线往复振幅 (m) = length/2
  double speed_max; // 最大速度 (m/s)
  double phase;     // 初始相位 (rad)
};

struct XY
{
  double x, y;
};

std::vector<StaticObstacle> static_obstacles_;
std::vector<DynamicObstacle> dyn_obstacles_;
pcl::PointCloud<pcl::PointXYZ> dyn_cloud_;
ros::Time start_time_;
// ======================================

// YAML 读取辅助：取键值，缺失时返回默认值
static double getOr(const YAML::Node& n, const std::string& key, double def)
{
  return n[key] ? n[key].as<double>() : def;
}

// 加载 YAML 场景文件，填充静态/动态障碍物表
static bool loadScene(const std::string& path)
{
  try
  {
    const YAML::Node root = YAML::LoadFile(path);
    if (!root["scene"] || !root["static_obstacles"])
    {
      ROS_ERROR("[obs_generator] 场景文件缺少 scene/static_obstacles 字段: %s",
                path.c_str());
      return false;
    }

    const YAML::Node scene = root["scene"];
    scene_left_x_ = getOr(scene, "left_x", scene_left_x_);
    scene_center_y_ = getOr(scene, "center_y", scene_center_y_);
    scene_size_x_ = getOr(scene, "size_x", scene_size_x_);
    scene_size_y_ = getOr(scene, "size_y", scene_size_y_);
    scene_size_z_ = getOr(scene, "size_z", scene_size_z_);
    resolution_ = getOr(scene, "resolution", resolution_);
    if (root["frame_id"])
      frame_id_ = root["frame_id"].as<std::string>();

    static_obstacles_.clear();
    for (const auto& node : root["static_obstacles"])
    {
      StaticObstacle obs;
      const std::string shape =
          node["shape"] ? node["shape"].as<std::string>() : "cuboid";
      obs.shape = (shape == "cylinder") ? 1 : 0;
      obs.x = getOr(node, "x", 0.0);
      obs.y = getOr(node, "y", 0.0);
      obs.lx = obs.ly = obs.h = obs.radius = 0.0;
      if (obs.shape == 1)  // cylinder
      {
        obs.radius = getOr(node, "radius", 0.5);
        obs.h = getOr(node, "height", 1.0);
      }
      else  // cuboid：优先 size: [lx, ly, h]
      {
        if (node["size"] && node["size"].IsSequence() &&
            node["size"].size() == 3)
        {
          obs.lx = node["size"][0].as<double>();
          obs.ly = node["size"][1].as<double>();
          obs.h = node["size"][2].as<double>();
        }
        else
        {
          obs.lx = getOr(node, "lx", 1.0);
          obs.ly = getOr(node, "ly", 1.0);
          obs.h = getOr(node, "h", 1.0);
        }
      }
      static_obstacles_.push_back(obs);
    }

    dyn_obstacles_.clear();
    if (root["dynamic_obstacles"])
    {
      int idx = 0;
      for (const auto& node : root["dynamic_obstacles"])
      {
        if (dyn_obs_num_ >= 0 && idx >= dyn_obs_num_)
          break;
        DynamicObstacle obs;
        const std::string shape =
            node["shape"] ? node["shape"].as<std::string>() : "cylinder";
        obs.shape = (shape == "sphere") ? 0 : 1;
        obs.radius = getOr(node, "radius", 0.3);
        obs.height = (obs.shape == 1) ? getOr(node, "height", 2.0) : 0.0;
        obs.z_center = getOr(node, "z_center", 1.0);
        obs.cx = 0.0;
        obs.cy = 0.0;
        if (node["center"] && node["center"].IsSequence() &&
            node["center"].size() == 2)
        {
          obs.cx = node["center"][0].as<double>();
          obs.cy = node["center"][1].as<double>();
        }
        obs.direction = getOr(node, "direction_deg", 90.0) * M_PI / 180.0;
        obs.amplitude = getOr(node, "length", 3.0) / 2.0;
        obs.speed_max = getOr(node, "speed_max", 1.5);
        obs.phase = getOr(node, "phase", 0.0);
        if (obs.amplitude <= 0.0 || obs.speed_max <= 0.0)
        {
          ROS_WARN("[obs_generator] 动态障碍物 %d 的 length/speed_max 非法，"
                   "跳过", idx);
          ++idx;
          continue;
        }
        dyn_obstacles_.push_back(obs);
        ++idx;
      }
    }
    return true;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("[obs_generator] YAML 加载失败: %s", e.what());
    return false;
  }
}

// ============ 静态障碍物体素化 ============
// 按全局栅格（分辨率 resolution_）把长方体体素化后加入点云
static void addBoxToCloud(pcl::PointCloud<pcl::PointXYZ>& cloud,
                          double x0, double x1, double y0, double y1,
                          double z0, double z1)
{
  const int ix0 = static_cast<int>(std::floor(x0 / resolution_));
  const int ix1 = static_cast<int>(std::ceil(x1 / resolution_));
  const int iy0 = static_cast<int>(std::floor(y0 / resolution_));
  const int iy1 = static_cast<int>(std::ceil(y1 / resolution_));
  const int iz0 = static_cast<int>(std::floor(z0 / resolution_));
  const int iz1 = static_cast<int>(std::ceil(z1 / resolution_));

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

// 把实心圆柱（底面贴地、半径 r、高 h）体素化后加入点云
static void addSolidCylinderToCloud(pcl::PointCloud<pcl::PointXYZ>& cloud,
                                    double cx, double cy, double r, double h)
{
  const int ix0 = static_cast<int>(std::floor((cx - r) / resolution_));
  const int ix1 = static_cast<int>(std::ceil((cx + r) / resolution_));
  const int iy0 = static_cast<int>(std::floor((cy - r) / resolution_));
  const int iy1 = static_cast<int>(std::ceil((cy + r) / resolution_));
  const int iz0 = 0;
  const int iz1 = static_cast<int>(std::ceil(h / resolution_));

  for (int ix = ix0; ix < ix1; ++ix)
    for (int iy = iy0; iy < iy1; ++iy)
    {
      const double x = (ix + 0.5) * resolution_;
      const double y = (iy + 0.5) * resolution_;
      const double dx = x - cx;
      const double dy = y - cy;
      if (dx * dx + dy * dy > r * r)
        continue;
      for (int iz = iz0; iz < iz1; ++iz)
      {
        pcl::PointXYZ p;
        p.x = x;
        p.y = y;
        p.z = (iz + 0.5) * resolution_;
        cloud.points.push_back(p);
      }
    }
}

// 构建静态障碍物点云
static void buildStaticCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  for (const auto& obs : static_obstacles_)
  {
    if (obs.shape == 1)
      addSolidCylinderToCloud(cloud, obs.x, obs.y, obs.radius, obs.h);
    else
      addBoxToCloud(cloud, obs.x - obs.lx / 2.0, obs.x + obs.lx / 2.0,
                    obs.y - obs.ly / 2.0, obs.y + obs.ly / 2.0, 0.0, obs.h);
  }
}
// ======================================

// ============ 动态障碍物 ============
// 计算单个动态障碍物在时刻 t 的平面位置（世界系，z 恒为 z_center）
// 直线往复：p(t) = center + A*sin(ωt + φ)*dir，ω = v_max / A
static XY computeDynamicPosition(const DynamicObstacle& obs, double t)
{
  const double omega = obs.speed_max / obs.amplitude;
  const double s = obs.amplitude * std::sin(omega * t + obs.phase);
  XY p;
  p.x = obs.cx + s * std::cos(obs.direction);
  p.y = obs.cy + s * std::sin(obs.direction);
  return p;
}

// 在指定位置生成圆柱表面点云（侧面 + 顶面，底面贴地）
static void addCylinderToCloud(pcl::PointCloud<pcl::PointXYZ>& cloud,
                               double cx, double cy, double r, double h)
{
  pcl::PointXYZ pt;
  const int heiNum = static_cast<int>(std::ceil(h / resolution_));
  const int angNum = std::max(
      8, static_cast<int>(std::ceil(2.0 * M_PI * r / resolution_)));

  for (int t = 0; t < heiNum; ++t)
    for (int a = 0; a < angNum; ++a)
    {
      const double ang = 2.0 * M_PI * a / angNum;
      pt.x = cx + r * std::cos(ang);
      pt.y = cy + r * std::sin(ang);
      pt.z = (t + 0.5) * resolution_ + 1e-2;
      cloud.points.push_back(pt);
    }

  const int radNum = static_cast<int>(std::ceil(r / resolution_));
  for (int i = -radNum; i <= radNum; ++i)
    for (int j = -radNum; j <= radNum; ++j)
    {
      const double x = cx + (i + 0.5) * resolution_;
      const double y = cy + (j + 0.5) * resolution_;
      if ((x - cx) * (x - cx) + (y - cy) * (y - cy) <= r * r)
      {
        pt.x = x;
        pt.y = y;
        pt.z = h + 1e-2;
        cloud.points.push_back(pt);
      }
    }
}

// 在指定位置生成球体表面点云（球心 (cx, cy, cz)、半径 r）
static void addSphereToCloud(pcl::PointCloud<pcl::PointXYZ>& cloud,
                             double cx, double cy, double cz, double r)
{
  pcl::PointXYZ pt;
  const int ringNum =
      std::max(4, static_cast<int>(std::ceil(M_PI * r / resolution_)));
  for (int i = 0; i <= ringNum; ++i)
  {
    const double phi = M_PI * i / ringNum;  // 0..pi
    const double z = cz + r * std::cos(phi);
    const double rr = r * std::sin(phi);
    const int angNum = std::max(
        4, static_cast<int>(std::ceil(2.0 * M_PI * rr / resolution_)));
    for (int a = 0; a < angNum; ++a)
    {
      const double ang = 2.0 * M_PI * a / angNum;
      pt.x = cx + rr * std::cos(ang);
      pt.y = cy + rr * std::sin(ang);
      pt.z = z;
      cloud.points.push_back(pt);
    }
  }
}

// 更新动态障碍物点云（每帧调用）
static void updateDynamicCloud()
{
  dyn_cloud_.clear();
  const double t = (ros::Time::now() - start_time_).toSec();
  for (const auto& obs : dyn_obstacles_)
  {
    const XY p = computeDynamicPosition(obs, t);
    if (obs.shape == 1)
      addCylinderToCloud(dyn_cloud_, p.x, p.y, obs.radius, obs.height);
    else
      addSphereToCloud(dyn_cloud_, p.x, p.y, obs.z_center, obs.radius);
  }
  dyn_cloud_.width = dyn_cloud_.points.size();
  dyn_cloud_.height = 1;
  dyn_cloud_.is_dense = true;
}

// 发布动态障碍物预测轨迹（ground truth，供 MPC 动态避障使用）。
// 采样网格与 MPC 对齐：t_k = 消息时间 + k*dt，步数 = N+1（默认 21）。
static void publishDynamicTrajectories()
{
  const ros::Time stamp = ros::Time::now();
  const double t0 = (stamp - start_time_).toSec();
  const double h = 0.01;

  for (size_t i = 0; i < dyn_obstacles_.size(); ++i)
  {
    const DynamicObstacle& obs = dyn_obstacles_[i];
    form_nav::DynamicObstacleTrajectory msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.obstacle_id = static_cast<int>(i);
    msg.shape = (obs.shape == 1) ? 1 : 0;
    msg.radius = obs.radius;
    msg.height = obs.height;
    msg.dt = dyn_traj_dt_;
    msg.steps = static_cast<uint32_t>(dyn_traj_steps_);
    msg.states.reserve(static_cast<size_t>(msg.steps) * 8);

    for (uint32_t k = 0; k < msg.steps; ++k)
    {
      const double tk = t0 + k * msg.dt;
      const XY p = computeDynamicPosition(obs, tk);
      const XY pm = computeDynamicPosition(obs, tk - h);
      const XY pp = computeDynamicPosition(obs, tk + h);
      const double vx = (pp.x - pm.x) / (2.0 * h);
      const double vy = (pp.y - pm.y) / (2.0 * h);
      msg.states.insert(msg.states.end(), {p.x, p.y, obs.z_center, 0.0,
                                           vx, vy, 0.0, 0.0});
    }
    dyn_traj_pub_.publish(msg);
  }
}

// 启动时检查动态障碍物扫掠区域是否与静态障碍物相交（xy AABB + 半径膨胀，
// 同时考虑 z 范围），冲突打印 WARN。
static void checkDynamicStaticCollisions()
{
  for (size_t di = 0; di < dyn_obstacles_.size(); ++di)
  {
    const DynamicObstacle& d = dyn_obstacles_[di];
    const double cdx = std::cos(d.direction) * d.amplitude;
    const double cdy = std::sin(d.direction) * d.amplitude;
    const double minx = std::min(d.cx - cdx, d.cx + cdx) - d.radius;
    const double maxx = std::max(d.cx - cdx, d.cx + cdx) + d.radius;
    const double miny = std::min(d.cy - cdy, d.cy + cdy) - d.radius;
    const double maxy = std::max(d.cy - cdy, d.cy + cdy) + d.radius;
    const double dminz = (d.shape == 1) ? 0.0 : d.z_center - d.radius;
    const double dmaxz = (d.shape == 1) ? d.height
                                        : d.z_center + d.radius;

    for (size_t si = 0; si < static_obstacles_.size(); ++si)
    {
      const StaticObstacle& s = static_obstacles_[si];
      const double sminx = s.x - (s.shape == 1 ? s.radius : s.lx / 2.0);
      const double smaxx = s.x + (s.shape == 1 ? s.radius : s.lx / 2.0);
      const double sminy = s.y - (s.shape == 1 ? s.radius : s.ly / 2.0);
      const double smaxy = s.y + (s.shape == 1 ? s.radius : s.ly / 2.0);
      const bool overlap_xy = maxx > sminx && minx < smaxx &&
                              maxy > sminy && miny < smaxy;
      const bool overlap_z = dmaxz > 0.0 && dminz < s.h;
      if (overlap_xy && overlap_z)
      {
        ROS_WARN("[obs_generator] 动态障碍物 %zu (中心 %.1f,%.1f) 的扫掠区域"
                 "与静态障碍物 %zu (%.1f,%.1f) 相交!",
                 di, d.cx, d.cy, si, s.x, s.y);
      }
    }
  }
}
// ======================================

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obs_generator");
  ros::NodeHandle n("~");

  n.param<std::string>("scene_file", scene_file_, "");
  n.param("pub_rate", pub_rate_, 10.0);
  n.param("dyn_obs/traj_dt", dyn_traj_dt_, 0.05);
  n.param("dyn_obs/traj_steps", dyn_traj_steps_, 21);
  n.param("dyn_obs/num", dyn_obs_num_, -1);

  if (scene_file_.empty())
  {
    ROS_ERROR("[obs_generator] 未指定 scene_file 参数，退出。");
    return 1;
  }
  if (!loadScene(scene_file_))
    return 1;

  global_map_pub_ =
      n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);
  local_map_pub_ =
      n.advertise<sensor_msgs::PointCloud2>("/map_generator/local_cloud", 1);
  dyn_map_pub_ =
      n.advertise<sensor_msgs::PointCloud2>("/map_generator/dynamic_cloud", 1);
  dyn_traj_pub_ = n.advertise<form_nav::DynamicObstacleTrajectory>(
      "/dynamic_obstacles/trajectory", 10);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  buildStaticCloud(cloud);
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;

  checkDynamicStaticCollisions();
  start_time_ = ros::Time::now();

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame_id_;
  msg.header.stamp = ros::Time::now();

  ROS_WARN("[obs_generator] 场景加载完成: file=%s, x=[%.2f, %.2f], "
           "y=[%.2f, %.2f], z=[0, %.2f], 静态 %zu 个 (点 %zu), "
           "动态 %zu 个",
           scene_file_.c_str(), scene_left_x_, scene_left_x_ + scene_size_x_,
           scene_center_y_ - scene_size_y_ / 2.0,
           scene_center_y_ + scene_size_y_ / 2.0, scene_size_z_,
           static_obstacles_.size(), cloud.points.size(),
           dyn_obstacles_.size());

  ros::Rate loop_rate(pub_rate_);
  while (ros::ok())
  {
    if (!dyn_obstacles_.empty())
    {
      updateDynamicCloud();
      publishDynamicTrajectories();

      sensor_msgs::PointCloud2 dyn_msg;
      pcl::toROSMsg(dyn_cloud_, dyn_msg);
      dyn_msg.header.stamp = ros::Time::now();
      dyn_msg.header.frame_id = frame_id_;
      dyn_map_pub_.publish(dyn_msg);
    }

    msg.header.stamp = ros::Time::now();
    global_map_pub_.publish(msg);
    local_map_pub_.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
