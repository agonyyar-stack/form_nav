#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>
#include "form_nav/DynamicObstacleTrajectory.h"

using namespace std;

// pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

random_device rd;
// default_random_engine eng(4);
default_random_engine eng;
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;
uniform_real_distribution<double> rand_inf;

ros::Publisher _local_map_pub;
ros::Publisher _all_map_pub;
ros::Publisher _dyn_map_pub;    // 动态障碍物点云（仅可视化）
ros::Publisher _dyn_traj_pub;   // 动态障碍物预测轨迹（MPC 避障输入）
ros::Publisher click_map_pub_;
ros::Subscriber _odom_sub;

vector<double> _state;

int _obs_num;
double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
double _z_limit, _sensing_range, _resolution, _pub_rate;
double _min_dist;

bool _map_ok = false;
bool _has_odom = false;

int circle_num_;
double radius_l_, radius_h_, z_l_, z_h_;
double theta_;
uniform_real_distribution<double> rand_radius_;
uniform_real_distribution<double> rand_radius2_;
uniform_real_distribution<double> rand_theta_;
uniform_real_distribution<double> rand_z_;

// ============ 动态障碍物相关 ============
struct DynamicObstacle
{
  enum TrajType { STRAIGHT = 0, CIRCLE = 1, ELLIPSE = 2 };
  enum Shape { SPHERE = 0, CYLINDER = 1 };

  TrajType type;
  Shape shape;
  double radius;          // 障碍物半径 (m)
  double height;          // 圆柱高度 (m)，球体填 0
  double z_center;        // 形状中心高度 (m)
  Eigen::Vector2d center;       // 轨迹中心 / 起点
  double direction;             // 直线运动方向角 (rad)
  double speed_max;             // 最大速度 (m/s)
  double period;                // 运动周期 (s)
  double radius_a;              // 圆半径 / 椭圆半长轴
  double radius_b;              // 椭圆半短轴
  double ellipse_angle;         // 椭圆旋转角 (rad)
};

int _pattern = 1;  // 1=仅静态, 2=仅动态, 3=静态+动态
int _dyn_obs_num = 0;
double _dyn_obs_radius = 0.3;   // 障碍物半径 (m)
double _dyn_obs_height = 2.0;   // 高度 (m)
double _dyn_traj_dt    = 0.05;  // 障碍物轨迹采样步长 (s)，对齐 MPC
int    _dyn_traj_steps = 11;    // 障碍物轨迹步数 (N+1)
double _dyn_speed_l    = 0.5;   // 最小速度 (m/s)
double _dyn_speed_h    = 2.0;   // 最大速度 (m/s)
double _dyn_period_l   = 3.0;   // 最小周期 (s)
double _dyn_period_h   = 8.0;   // 最大周期 (s)
double _dyn_radius_l   = 1.5;   // 最小轨迹半径 (m)
double _dyn_radius_h   = 5.0;   // 最大轨迹半径 (m)
uniform_real_distribution<double> rand_z_center;

vector<DynamicObstacle> _dyn_obstacles;
ros::Time _start_time;
pcl::PointCloud<pcl::PointXYZ> _dynCloud;
// ========================================

sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

sensor_msgs::PointCloud2 localMap_pcd;
sensor_msgs::PointCloud2 dynMap_pcd;
pcl::PointCloud<pcl::PointXYZ> clicked_cloud_;

void RandomMapGenerate()
{
  pcl::PointXYZ pt_random;

  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);

  rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

  // generate polar obs
  for (int i = 0; i < _obs_num; i++)
  {
    double x, y, w, h;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil(w / _resolution);

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
      {
        h = rand_h(eng);
        int heiNum = ceil(h / _resolution);
        for (int t = -2; t < heiNum; t++)
        {
          pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
          pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
          pt_random.z = (t + 0.5) * _resolution + 1e-2;
          cloudMap.points.push_back(pt_random);
        }
      }
  }

  // generate circle obs
  for (int i = 0; i < circle_num_; ++i)
  {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    Eigen::Vector3d translate(x, y, z);

    double theta = rand_theta_(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
        1;

    double radius1 = rand_radius_(eng);
    double radius2 = rand_radius2_(eng);

    // draw a circle centered at (x,y,z)
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2)
    {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      // inflate
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz)
          {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                           ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            cloudMap.push_back(pt_random);
          }
    }
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_WARN("Finished generate random map ");

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

void RandomMapGenerateCylinder()
{
  pcl::PointXYZ pt_random;

  vector<Eigen::Vector2d> obs_position;

  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);
  rand_inf = uniform_real_distribution<double>(0.5, 1.5);

  rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

  // generate polar obs
  for (int i = 0; i < _obs_num && ros::ok(); i++)
  {
    double x, y, w, h, inf;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);
    inf = rand_inf(eng);

    bool flag_continue = false;
    for (auto p : obs_position)
      if ((Eigen::Vector2d(x, y) - p).norm() < _min_dist /*metres*/)
      {
        i--;
        flag_continue = true;
        break;
      }
    if (flag_continue)
      continue;

    obs_position.push_back(Eigen::Vector2d(x, y));

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    int widNum = ceil((w * inf) / _resolution);
    double radius = (w * inf) / 2;

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
      {
        h = rand_h(eng);
        int heiNum = ceil(h / _resolution);
        for (int t = -2; t < heiNum; t++)
        {
          double temp_x = x + (r + 0.5) * _resolution + 1e-2;
          double temp_y = y + (s + 0.5) * _resolution + 1e-2;
          double temp_z = (t + 0.5) * _resolution + 1e-2;
          if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x, y)).norm() <= radius)
          {
            pt_random.x = temp_x;
            pt_random.y = temp_y;
            pt_random.z = temp_z;
            cloudMap.points.push_back(pt_random);
          }
        }
      }
  }

  // generate circle obs
  for (int i = 0; i < circle_num_; ++i)
  {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    Eigen::Vector3d translate(x, y, z);

    double theta = rand_theta_(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
        1;

    double radius1 = rand_radius_(eng);
    double radius2 = rand_radius2_(eng);

    // draw a circle centered at (x,y,z)
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2)
    {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);
      cpt(2) = radius2 * sin(angle);

      // inflate
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz)
          {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                           ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            cloudMap.push_back(pt_random);
          }
    }
  }

  // for (double x = -1.5; x <= 1.5; x += 0.1)
  //   for (double y = -1.5; y <= 1.5; y += 0.1)
  //     for (double z = -0.5; z <= 3.0; z += 0.1)
  //     {
  //       pt_random.x = x;
  //       pt_random.y = y;
  //       pt_random.z = z;
  //       cloudMap.push_back(pt_random);
  //     }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_WARN("Finished generate random map ");

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

// ============ 动态障碍物函数 ============
// 在指定位置生成圆柱点云（侧面 + 顶面，底面贴地，半径 r、高 h）
void addCylinderToCloud(pcl::PointCloud<pcl::PointXYZ>& cloud,
                        double cx, double cy, double r, double h)
{
  pcl::PointXYZ pt;
  int heiNum = ceil(h / _resolution);
  int angNum = std::max(8, static_cast<int>(ceil(2.0 * M_PI * r / _resolution)));

  // 侧面
  for (int t = 0; t < heiNum; t++)
    for (int a = 0; a < angNum; a++)
    {
      double ang = 2.0 * M_PI * a / angNum;
      pt.x = cx + r * cos(ang);
      pt.y = cy + r * sin(ang);
      pt.z = (t + 0.5) * _resolution + 1e-2;
      cloud.points.push_back(pt);
    }

  // 顶面
  int radNum = ceil(r / _resolution);
  for (int i = -radNum; i <= radNum; i++)
    for (int j = -radNum; j <= radNum; j++)
    {
      double x = cx + (i + 0.5) * _resolution;
      double y = cy + (j + 0.5) * _resolution;
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
void addSphereToCloud(pcl::PointCloud<pcl::PointXYZ>& cloud,
                      double cx, double cy, double cz, double r)
{
  pcl::PointXYZ pt;
  int ringNum = std::max(4, static_cast<int>(ceil(M_PI * r / _resolution)));
  for (int i = 0; i <= ringNum; i++)
  {
    double phi = M_PI * i / ringNum;   // 0..pi
    double z = cz + r * cos(phi);
    double rr = r * sin(phi);
    int angNum = std::max(4, static_cast<int>(ceil(2.0 * M_PI * rr / _resolution)));
    for (int a = 0; a < angNum; a++)
    {
      double ang = 2.0 * M_PI * a / angNum;
      pt.x = cx + rr * cos(ang);
      pt.y = cy + rr * sin(ang);
      pt.z = z;
      cloud.points.push_back(pt);
    }
  }
}

// 计算单个动态障碍物当前位置
Eigen::Vector2d computeDynamicPosition(const DynamicObstacle& obs, double t)
{
  if (obs.type == DynamicObstacle::STRAIGHT)
  {
    // 直线往复: 速度 v(t) = v_max * sin(2π t / T)
    // 位移 s(t) = v_max * T / (2π) * (1 - cos(2π t / T))
    double omega = 2.0 * M_PI / obs.period;
    double amplitude = obs.speed_max / omega;  // v_max * T / (2π)
    double s = amplitude * (1.0 - cos(omega * t));
    Eigen::Vector2d dir(cos(obs.direction), sin(obs.direction));
    return obs.center + s * dir;
  }
  else if (obs.type == DynamicObstacle::CIRCLE)
  {
    double omega = 2.0 * M_PI / obs.period;
    double x = obs.center.x() + obs.radius_a * cos(omega * t);
    double y = obs.center.y() + obs.radius_a * sin(omega * t);
    return Eigen::Vector2d(x, y);
  }
  else  // ELLIPSE
  {
    double omega = 2.0 * M_PI / obs.period;
    double xp = obs.radius_a * cos(omega * t);
    double yp = obs.radius_b * sin(omega * t);
    double cs = cos(obs.ellipse_angle);
    double sn = sin(obs.ellipse_angle);
    double x = obs.center.x() + xp * cs - yp * sn;
    double y = obs.center.y() + xp * sn + yp * cs;
    return Eigen::Vector2d(x, y);
  }
}

// 初始化动态障碍物
void initDynamicObstacles()
{
  uniform_real_distribution<double> rand_dir(0.0, 2.0 * M_PI);
  uniform_real_distribution<double> rand_type(0.0, 1.0);
  uniform_real_distribution<double> rand_speed(_dyn_speed_l, _dyn_speed_h);
  uniform_real_distribution<double> rand_period(_dyn_period_l, _dyn_period_h);
  uniform_real_distribution<double> rand_radius(_dyn_radius_l, _dyn_radius_h);
  rand_z_center = uniform_real_distribution<double>(
      0.6, std::max(1.2, _z_size - 0.6));

  // 安全边距 (障碍物半径 + 轨迹振幅)
  double margin = _dyn_obs_radius + _dyn_radius_h;

  for (int i = 0; i < _dyn_obs_num; i++)
  {
    DynamicObstacle obs;
    double r = rand_type(eng);

    // 随机选择轨迹类型: 40% 直线, 30% 圆, 30% 椭圆
    if (r < 0.4)
      obs.type = DynamicObstacle::STRAIGHT;
    else if (r < 0.7)
      obs.type = DynamicObstacle::CIRCLE;
    else
      obs.type = DynamicObstacle::ELLIPSE;

    // 随机选择形状: 50% 圆柱, 50% 球体
    obs.shape = (rand_type(eng) < 0.5) ? DynamicObstacle::CYLINDER
                                       : DynamicObstacle::SPHERE;
    obs.radius = _dyn_obs_radius;
    obs.height = _dyn_obs_height;
    obs.z_center = (obs.shape == DynamicObstacle::SPHERE)
                       ? rand_z_center(eng)
                       : obs.height / 2.0;

    obs.speed_max = rand_speed(eng);
    obs.period   = rand_period(eng);
    obs.direction = rand_dir(eng);
    obs.radius_a  = rand_radius(eng);
    obs.radius_b  = rand_radius(eng) * 0.7;  // 椭圆的短半轴稍小

    if (obs.type == DynamicObstacle::STRAIGHT)
    {
      // 直线: center 是振荡中心，振幅 amp = v_max * T / (2π)
      double amp = obs.speed_max * obs.period / (2.0 * M_PI);
      double safe_margin = _dyn_obs_radius + amp;
      uniform_real_distribution<double> rand_x_s(_x_l + safe_margin, _x_h - safe_margin);
      uniform_real_distribution<double> rand_y_s(_y_l + safe_margin, _y_h - safe_margin);
      obs.center.x() = rand_x_s(eng);
      obs.center.y() = rand_y_s(eng);
      obs.radius_a = 0; obs.radius_b = 0; obs.ellipse_angle = 0;
    }
    else if (obs.type == DynamicObstacle::CIRCLE)
    {
      double radius = obs.radius_a;
      // 用 speed_max 反算周期，使实际线速度 = speed_max
      obs.period = 2.0 * M_PI * radius / obs.speed_max;
      double safe_margin = _dyn_obs_radius + radius;
      uniform_real_distribution<double> rand_x_c(_x_l + safe_margin, _x_h - safe_margin);
      uniform_real_distribution<double> rand_y_c(_y_l + safe_margin, _y_h - safe_margin);
      obs.center.x() = rand_x_c(eng);
      obs.center.y() = rand_y_c(eng);
      obs.direction = 0; obs.radius_b = 0; obs.ellipse_angle = 0;
    }
    else  // ELLIPSE
    {
      obs.ellipse_angle = rand_dir(eng);
      // 用 speed_max 反算周期，使最大切向速度 ≈ speed_max
      double effective_r = std::max(obs.radius_a, obs.radius_b);
      obs.period = 2.0 * M_PI * effective_r / obs.speed_max;
      double safe_margin = _dyn_obs_radius + std::max(obs.radius_a, obs.radius_b);
      uniform_real_distribution<double> rand_x_e(_x_l + safe_margin, _x_h - safe_margin);
      uniform_real_distribution<double> rand_y_e(_y_l + safe_margin, _y_h - safe_margin);
      obs.center.x() = rand_x_e(eng);
      obs.center.y() = rand_y_e(eng);
      obs.direction = 0;
    }

    _dyn_obstacles.push_back(obs);
  }

  cout << "Initialized " << _dyn_obs_num << " dynamic obstacles." << endl;
}

// 更新动态障碍物点云 (每帧调用)
void updateDynamicObstacles()
{
  _dynCloud.clear();
  double t = (ros::Time::now() - _start_time).toSec();

  for (auto& obs : _dyn_obstacles)
  {
    Eigen::Vector2d pos = computeDynamicPosition(obs, t);
    if (obs.shape == DynamicObstacle::CYLINDER)
      addCylinderToCloud(_dynCloud, pos.x(), pos.y(), obs.radius, obs.height);
    else
      addSphereToCloud(_dynCloud, pos.x(), pos.y(), obs.z_center, obs.radius);
  }

  _dynCloud.width = _dynCloud.points.size();
  _dynCloud.height = 1;
  _dynCloud.is_dense = true;
}

// 发布动态障碍物预测轨迹（已知轨迹 ground truth，供 MPC 避障惩罚使用）。
// 采样网格与 MPC 对齐：t_k = 消息时间 + k*dt，步数 = N+1；
// 位置用解析轨迹精确采样，速度用中心差分近似（仅记录/可视化用）。
void publishDynamicTrajectories()
{
  const ros::Time stamp = ros::Time::now();
  const double t0 = (stamp - _start_time).toSec();
  const double h = 0.01;

  for (size_t i = 0; i < _dyn_obstacles.size(); ++i)
  {
    const DynamicObstacle& obs = _dyn_obstacles[i];
    form_nav::DynamicObstacleTrajectory msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "world";
    msg.obstacle_id = static_cast<int>(i);
    msg.shape = (obs.shape == DynamicObstacle::CYLINDER) ? 1 : 0;
    msg.radius = obs.radius;
    msg.height = obs.height;
    msg.dt = _dyn_traj_dt;
    msg.steps = static_cast<uint32_t>(_dyn_traj_steps);
    msg.states.reserve(static_cast<size_t>(msg.steps) * 8);

    for (uint32_t k = 0; k < msg.steps; ++k)
    {
      const double tk = t0 + k * msg.dt;
      const Eigen::Vector2d p = computeDynamicPosition(obs, tk);
      const Eigen::Vector2d pm = computeDynamicPosition(obs, tk - h);
      const Eigen::Vector2d pp = computeDynamicPosition(obs, tk + h);
      const double vx = (pp.x() - pm.x()) / (2.0 * h);
      const double vy = (pp.y() - pm.y()) / (2.0 * h);
      msg.states.insert(msg.states.end(), {
          p.x(), p.y(), obs.z_center, 0.0,
          vx, vy, 0.0, 0.0});
    }
    _dyn_traj_pub.publish(msg);
  }
}
// ========================================

void clickCallback(const geometry_msgs::PoseStamped &msg)
{
  double x = msg.pose.position.x;
  double y = msg.pose.position.y;
  double w = rand_w(eng);
  double h;
  pcl::PointXYZ pt_random;

  x = floor(x / _resolution) * _resolution + _resolution / 2.0;
  y = floor(y / _resolution) * _resolution + _resolution / 2.0;

  int widNum = ceil(w / _resolution);

  for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
    for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
    {
      h = rand_h(eng);
      int heiNum = ceil(h / _resolution);
      for (int t = -1; t < heiNum; t++)
      {
        pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
        pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
        pt_random.z = (t + 0.5) * _resolution + 1e-2;
        clicked_cloud_.points.push_back(pt_random);
        cloudMap.points.push_back(pt_random);
      }
    }
  clicked_cloud_.width = clicked_cloud_.points.size();
  clicked_cloud_.height = 1;
  clicked_cloud_.is_dense = true;

  pcl::toROSMsg(clicked_cloud_, localMap_pcd);
  localMap_pcd.header.frame_id = "world";
  click_map_pub_.publish(localMap_pcd);

  cloudMap.width = cloudMap.points.size();

  return;
}

int i = 0;
void pubPoints()
{
  while (ros::ok())
  {
    ros::spinOnce();
    if (_map_ok)
      break;
  }

  // 更新动态障碍物位置
  if ((_pattern == 2 || _pattern == 3) && _dyn_obs_num > 0)
  {
    updateDynamicObstacles();
  }

  // 拆通道发布：
  //   /map_generator/global_cloud —— 仅静态障碍物（A* 全局规划 + MPC 静态 KD-tree 使用）
  //   /map_generator/dynamic_cloud —— 仅动态障碍物点云（仅可视化，不影响规划）
  pcl::PointCloud<pcl::PointXYZ> staticCloud;
  if (_pattern == 1 || _pattern == 3)
  {
    staticCloud = cloudMap;  // 包含静态障碍物
  }
  // 仅动态模式：发空点云保活订阅端（A* 等待地图消息），字段合法可被 pcl::fromROSMsg 解析
  pcl::toROSMsg(staticCloud, globalMap_pcd);
  globalMap_pcd.header.stamp = ros::Time::now();
  globalMap_pcd.header.frame_id = "world";
  _all_map_pub.publish(globalMap_pcd);

  if ((_pattern == 2 || _pattern == 3) && _dyn_obs_num > 0)
  {
    pcl::toROSMsg(_dynCloud, dynMap_pcd);
    dynMap_pcd.header.stamp = ros::Time::now();
    dynMap_pcd.header.frame_id = "world";
    _dyn_map_pub.publish(dynMap_pcd);
    publishDynamicTrajectories();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "random_static_obs");
  ros::NodeHandle n("~");

  _local_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/local_cloud", 1);
  _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);
  _dyn_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/dynamic_cloud", 1);
  _dyn_traj_pub =
      n.advertise<form_nav::DynamicObstacleTrajectory>("/dynamic_obstacles/trajectory", 10);

  // click_map_pub_ = n.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/local_map", 1);

  n.param("map/x_size", _x_size, 50.0);
  n.param("map/y_size", _y_size, 50.0);
  n.param("map/z_size", _z_size, 5.0);
  n.param("map/obs_num", _obs_num, 30);
  n.param("map/resolution", _resolution, 0.1);
  n.param("map/circle_num", circle_num_, 30);

  n.param("ObstacleShape/lower_rad", _w_l, 0.3);
  n.param("ObstacleShape/upper_rad", _w_h, 0.8);
  n.param("ObstacleShape/lower_hei", _h_l, 3.0);
  n.param("ObstacleShape/upper_hei", _h_h, 7.0);

  n.param("ObstacleShape/radius_l", radius_l_, 7.0);
  n.param("ObstacleShape/radius_h", radius_h_, 7.0);
  n.param("ObstacleShape/z_l", z_l_, 7.0);
  n.param("ObstacleShape/z_h", z_h_, 7.0);
  n.param("ObstacleShape/theta", theta_, 7.0);

  n.param("pub_rate", _pub_rate, 10.0);
  n.param("min_distance", _min_dist, 1.0);

  // 障碍物生成模式: 1=仅静态, 2=仅动态, 3=静态+动态
  n.param("pattern", _pattern, 1);
  ROS_INFO("[random_obs] pattern = %d (1=static, 2=dynamic, 3=both)", _pattern);

  // 动态障碍物参数
  n.param("dyn_obs/num", _dyn_obs_num, 0);
  n.param("dyn_obs/size", _dyn_obs_radius, 0.3);  // 语义：半径 (m)
  n.param("dyn_obs/height", _dyn_obs_height, 2.0);
  n.param("dyn_obs/traj_dt", _dyn_traj_dt, 0.05);
  n.param("dyn_obs/traj_steps", _dyn_traj_steps, 11);
  n.param("dyn_obs/speed_l", _dyn_speed_l, 0.5);
  n.param("dyn_obs/speed_h", _dyn_speed_h, 2.0);
  n.param("dyn_obs/period_l", _dyn_period_l, 3.0);
  n.param("dyn_obs/period_h", _dyn_period_h, 8.0);
  n.param("dyn_obs/radius_l", _dyn_radius_l, 1.5);
  n.param("dyn_obs/radius_h", _dyn_radius_h, 5.0);

  _x_l = -_x_size / 2.0;
  _x_h = +_x_size / 2.0;

  _y_l = -_y_size / 2.0;
  _y_h = +_y_size / 2.0;

  _obs_num = min(_obs_num, (int)_x_size * 10);
  _z_limit = _z_size;

  ros::Duration(0.5).sleep(); // 休眠0.5秒，等待ROS系统完全初始化，确保后续操作的稳定性

  unsigned int seed = rd();
  // unsigned int seed = 1398191972;
  cout << "seed=" << seed << endl;
  eng.seed(seed);

  // 根据 pattern 决定生成哪种障碍物
  if (_pattern == 1 || _pattern == 3)
  {
    RandomMapGenerateCylinder();
  }
  else
  {
    _map_ok = true;  // 仅动态模式，无需静态地图
  }

  // 初始化动态障碍物
  if ((_pattern == 2 || _pattern == 3) && _dyn_obs_num > 0)
  {
    initDynamicObstacles();
  }

  _start_time = ros::Time::now();

  ros::Rate loop_rate(_pub_rate); // 创建循环速率控制，确保循环以指定的频率运行

  while (ros::ok())
  {
    pubPoints();
    ros::spinOnce(); //处理一次回调
    loop_rate.sleep();
  }
}
