#include <form_nav/a_star.h>

namespace path_searching
{

void Astar::setParam(ros::NodeHandle& nh)
{
  nh.param("astar/resolution", resolution_, 0.1);
  nh.param("astar/lambda_heu", lambda_, 1.0);
  nh.param("astar/allocated_node_num", allocated_node_num_, 100000);
  nh.param("astar/goal_tolerance", goal_tolerance_, 0.5);
}

void Astar::init()
{
  this->inv_resolution_ = 1.0 / resolution_;
  this->tie_breaker_ = 1.0 + 1e-4;              // default value
  path_node_pool_.resize(allocated_node_num_);  // allocate memory for path_node_pool_
  for (int i = 0; i < allocated_node_num_; i++)
  {
    path_node_pool_[i] = new PathNode();
  }

  if (!grid_map_)
  {
    std::cerr << "Error: grid_map_ is not set." << std::endl;
    return;
  }

  use_node_num_ = 0;
  grid_map_->getRegion(origin_, map_size_);
  resolution_ = grid_map_->getResolution();

  std::cout << "origin: " << origin_.transpose() << std::endl;
  std::cout << "map_size: " << map_size_.transpose() << std::endl;
  std::cout << "resolution: " << resolution_ << std::endl;
  std::cout << "lambda_heu: " << lambda_ << std::endl;
  std::cout << "allocated_node_num: " << allocated_node_num_ << std::endl;
}

Astar::~Astar()
{
  for (int i = 0; i < allocated_node_num_; i++)  // free memory for path_node_pool_
  {
    delete path_node_pool_[i];
  }
}

int Astar::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, std::vector<Eigen::Vector3d>& path)
{
  // 如果起点或终点不在地图内，则返回NO_PATH_FOUND
  ros::Time start_time = ros::Time::now();
  if (grid_map_->isInMap(end_pt) == false)
  {
    std::cerr << "Error: end_pt is out of map." << std::endl;
    return NO_PATH_FOUND;
  }
  if (use_node_num_ >= allocated_node_num_)
  {
    std::cerr << "Error: allocated_node_num is too small (start node)." << std::endl;
    return NO_PATH_FOUND;
  }
  // 将起点加入open_list中，并初始化其g_cost和f_cost
  PathNodePtr start_node = path_node_pool_[use_node_num_];  // bug fixed
  start_node->g_cost = 0.0;
  start_node->position = start_pt;
  start_node->parent = NULL;
  start_node->f_cost = lambda_ * getDiagonalHeu(start_pt, end_pt);
  start_node->node_state = IN_OPEN_LIST_;

  open_list_.push(start_node);
  expanded_nodes_.insert(start_node->position, start_node);
  use_node_num_ += 1;

  while (!open_list_.empty())
  {
    // 从open_list中取出f_cost最小的节点current_node，并加入close_list
    PathNodePtr current_node = open_list_.top();
    open_list_.pop();
    // 同一节点可能因代价更新被多次压入，跳过已闭合的旧条目
    if (current_node->node_state == IN_CLOSE_LIST_)
    {
      continue;
    }
    current_node->node_state = IN_CLOSE_LIST_;
    close_list_.insert(current_node->position, current_node);

    // 如果当前节点已经进入终点容差范围，则返回路径
    if ((current_node->position - end_pt).norm() < goal_tolerance_)
    {
      ros::Time end_time = ros::Time::now();
      std::cout << "reached end_pt" << std::endl;
      std::cout << "total_time: " << (end_time - start_time).toSec() << " seconds" << std::endl;
      PathNodePtr end_node = current_node;
      std::cout << "use_node_num: " << use_node_num_ << std::endl;
      std::cout << "total_cost:" << end_node->g_cost << std::endl;
      retrievePath(end_node, path);
      // 若终点到已搜索到的末端点的直线段无碰撞，则把精确终点追加进路径，
      // 使 MPC 的停靠目标与用户目标（+编队偏移）严格一致（kinoA* 的 shot 段
      // 起同样作用）；有碰撞时保留当前末端，避免穿障。
      if (!path.empty() && (path.back() - end_pt).norm() > 1e-6 &&
          isSegmentCollisionFree(path.back(), end_pt))
      {
        path.push_back(end_pt);
      }
      return REACH_END;
    }
    // 遍历当前节点的邻居节点，更新邻居节点的g_cost和f_cost，并将其加入open_list中
    for (double x = -resolution_; x <= resolution_; x += resolution_)
      for (double y = -resolution_; y <= resolution_; y += resolution_)
        for (double z = -resolution_; z <= resolution_; z += resolution_)
        {
          Eigen::Vector3d neighbor_pos = current_node->position + Eigen::Vector3d(x, y, z);
          // std::cout << "neighbor_pos: " << neighbor_pos.transpose() << std::endl;

          // 如果邻居节点不在地图内，则跳过
          if (grid_map_->isInMap(neighbor_pos) == false)
          {
            // std::cout << "out of map" << std::endl;
            continue;
          }
          // 如果邻居节点是障碍物，则跳过
          if (grid_map_->getInflateOccupancy(neighbor_pos) == true)
          {
            // std::cout << "known obstacle" << std::endl;
            continue;
          }
          // 如果邻居节点已经在close_list中，则跳过
          if (close_list_.find(neighbor_pos) != NULL)
          {
            // std::cout << "already in close_list" << std::endl;
            continue;
          }

          double delta_pos = Eigen::Vector3d(x, y, z).norm();
          double tmp_g_cost = current_node->g_cost + delta_pos;
          PathNodePtr tmp_node = expanded_nodes_.find(neighbor_pos);

          if (tmp_node == NULL)
          {
            if (use_node_num_ >= allocated_node_num_)
            {
              std::cerr << "Error: allocated_node_num is too small." << std::endl;
              return NO_PATH_FOUND;
            }
            PathNodePtr neighbor_node = path_node_pool_[use_node_num_];
            use_node_num_ += 1;
            neighbor_node->g_cost = tmp_g_cost;
            neighbor_node->position = neighbor_pos;
            neighbor_node->parent = current_node;
            neighbor_node->f_cost = neighbor_node->g_cost + lambda_ * getDiagonalHeu(neighbor_pos, end_pt);
            neighbor_node->node_state = IN_OPEN_LIST_;
            open_list_.push(neighbor_node);
            expanded_nodes_.insert(neighbor_node->position, neighbor_node);
          }
          else if (tmp_g_cost < tmp_node->g_cost)
          {
            tmp_node->g_cost = tmp_g_cost;
            tmp_node->parent = current_node;
            tmp_node->f_cost = tmp_node->g_cost + lambda_ * getDiagonalHeu(tmp_node->position, end_pt);
            // 代价被更新后重新压入优先队列，保证按最新的 f_cost 排序；
            // 队列中残留的旧条目在弹出时会被 IN_CLOSE_LIST_ 检查跳过。
            tmp_node->node_state = IN_OPEN_LIST_;
            open_list_.push(tmp_node);
          }
        }
  }

  std::cout << "no path found!" << std::endl;
  std::cout << "use_node_num: " << use_node_num_ << std::endl;
  return NO_PATH_FOUND;
}

double Astar::getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) //功能：计算两个三维点之间的欧几里得距离启发式函数
{
  return tie_breaker_ * (x1 - x2).norm();
}

double Astar::getDiagonalHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) //功能：计算两个三维点之间的对角线距离启发式函数
{
  double dx = std::abs(x1(0) - x2(0));
  double dy = std::abs(x1(1) - x2(1));
  double dz = std::abs(x1(2) - x2(2));
  // 面/棱/角邻接代价分别为 1 / √2 / √3 时，三维 26 邻域的精确最短距离：
  // 排序 dx >= dy >= dz 后，最短路径 = (dx-dy) 次面邻接 + (dy-dz) 次棱邻接 + dz 次角邻接。
  double max_xyz = std::max({ dx, dy, dz });
  double min_xyz = std::min({ dx, dy, dz });
  double mid_xyz = dx + dy + dz - max_xyz - min_xyz;
  double h = (max_xyz - mid_xyz) + std::sqrt(2.0) * (mid_xyz - min_xyz) +
             std::sqrt(3.0) * min_xyz;
  // 不再乘 tie_breaker_：任何 >1 的系数都会破坏可采纳性，无法保证最短路径。
  return h;
}

// Eigen::Vector3i Astar::posToIndex(Eigen::Vector3d pos) {
//   Eigen::Vector3i index;
//   index(0) =  std::floor((pos(0) - origin_(0)) * inv_resolution_); // fix bug
//   index(1) =  std::floor((pos(1) - origin_(1)) * inv_resolution_);
//   index(2) =  std::floor((pos(2) - origin_(2)) * inv_resolution_);
//   return index;
// }

void Astar::retrievePath(PathNodePtr end_node, std::vector<Eigen::Vector3d>& path)
{
  PathNodePtr current_node = end_node;
  while (current_node->parent != NULL)
  {
    path.push_back(current_node->position);
    current_node = current_node->parent;
  }
  path.push_back(current_node->position);
  std::reverse(path.begin(), path.end());
}

bool Astar::isSegmentCollisionFree(const Eigen::Vector3d& from, const Eigen::Vector3d& to)
{
  const double seg_len = (to - from).norm();
  const double step = std::max(0.5 * resolution_, 0.05);
  const int samples = std::max(2, static_cast<int>(std::ceil(seg_len / step)) + 1);
  for (int i = 0; i < samples; ++i)
  {
    const double ratio = static_cast<double>(i) / static_cast<double>(samples - 1);
    const Eigen::Vector3d pt = from + ratio * (to - from);
    if (!grid_map_->isInMap(pt) || grid_map_->getInflateOccupancy(pt) != 0)
    {
      return false;
    }
  }
  return true;
}

void Astar::setGridMap(GridMap::Ptr& grid_map)
{
  this->grid_map_ = grid_map; //将外部传入的网格地图对象赋值给A*算法类的成员变量grid_map_，以便在A*算法中使用该网格地图进行路径搜索和碰撞检测
}

void Astar::reset()
{
  this->expanded_nodes_.clear();
  this->close_list_.clear();
  this->path_.clear();

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  this->open_list_.swap(empty_queue);

  use_node_num_ = 0;
  for (int i = 0; i < allocated_node_num_; i++)
  {
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->g_cost = inf;
    node->f_cost = inf;
    node->node_state = NOT_EXPANDED_;
  }
}

}  // namespace path_searching
