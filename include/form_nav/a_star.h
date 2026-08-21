#ifndef A_STAR_H_
#define A_STAR_H_

#include <queue>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <unordered_map>

#include <form_nav/grid_map.h>

namespace path_searching
{

#define inf (1 << 30)
#define IN_OPEN_LIST_ 'a'
#define IN_CLOSE_LIST_ 'b'
#define NOT_EXPANDED_ 'c'

class PathNode
{
public:
  Eigen::Vector3d position;
  double g_cost, f_cost;
  char node_state;
  PathNode* parent;

  PathNode()
  {
    g_cost = inf;
    f_cost = inf;
    parent = NULL;
    node_state = NOT_EXPANDED_;
  }
  ~PathNode() {};
};
typedef PathNode* PathNodePtr;

class NodeComparator
{
public:
  bool operator()(const PathNodePtr& node1, const PathNodePtr& node2) const
  {
    return node1->f_cost > node2->f_cost;
  }
};

template <typename T>
struct hash_func : std::unary_function<T, size_t>
{
  size_t operator()(T const& x) const
  {
    size_t seed = 0;
    for (size_t i = 0; i < x.size(); ++i)
    {
      auto elem = *(x.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable
{
private:
  // don't use Eigen::Vector3i index as key, because different position may have the same index
  std::unordered_map<Eigen::Vector3d, PathNodePtr, hash_func<Eigen::Vector3d>> node_table_;

public:
  NodeHashTable() {};
  ~NodeHashTable() {};

  void insert(Eigen::Vector3d pos, PathNodePtr node)
  {
    node_table_.insert(std::make_pair(pos, node));
  }

  PathNodePtr find(Eigen::Vector3d pos)
  {
    auto it = node_table_.find(pos);
    if (it != node_table_.end())
    {
      return it->second;
    }
    else
    {
      return NULL;
    }
  }

  void erase(Eigen::Vector3d pos)
  {
    node_table_.erase(pos);
  }

  void clear()
  {
    node_table_.clear();
  }
};

class Astar
{
private:
  /* main data structure */
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_list_;
  NodeHashTable close_list_;
  NodeHashTable expanded_nodes_;
  std::vector<PathNodePtr> path_node_pool_;
  std::vector<Eigen::Vector3d> path_;

  /* main search parameters */
  int allocated_node_num_;  // pre-allocated the nodes num
  int use_node_num_;        // number of the nodes expanded
  double lambda_;           // weight for the heuristic term
  double tie_breaker_;      // enhance the search velocity
  double goal_tolerance_;   // 终点容差 (m)，与 kinoA* 的 goal_tolerance 语义对齐

  enum
  {
    REACH_END = 1,
    NO_PATH_FOUND = 2
  };

  /* main map parameters */
  Eigen::Vector3d origin_;
  Eigen::Vector3d map_size_;
  double resolution_;
  double inv_resolution_;
  GridMap::Ptr grid_map_; //定义一个指向GridMap对象的智能指针，表示A*算法所使用的网格地图

  /* helper function */
  // Eigen::Vector3i posToIndex(Eigen::Vector3d pos);
  double getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
  double getDiagonalHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
  void retrievePath(PathNodePtr end_node, std::vector<Eigen::Vector3d>& path);
  // 检查直线段是否无碰撞（从 from 到 to 按约半个分辨率采样）
  bool isSegmentCollisionFree(const Eigen::Vector3d& from, const Eigen::Vector3d& to);

public:
  void setParam(ros::NodeHandle& nh); //设置A*算法的参数，包括启发式函数权重、节点预分配数量等
  void init();
  void setGridMap(GridMap::Ptr& grid_map); //设置网格地图
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, std::vector<Eigen::Vector3d>& path);
  void reset();

  Astar() {};
  ~Astar();

  typedef std::shared_ptr<Astar> Ptr;
};

}  // namespace path_searching

#endif /* A_STAR_H_ */
