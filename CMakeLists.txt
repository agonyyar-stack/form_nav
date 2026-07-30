cmake_minimum_required(VERSION 3.0.2) # 至少用 CMake 3.0.2 
project(form_nav)                     # 定义这个 CMake 项目的名字为 form_nav

add_compile_options(-std=c++17)

# 查找读取包的配置，设置对应的变量和宏，告诉后面编译器和链接器该去哪儿找头文件、库文件
find_package(catkin REQUIRED COMPONENTS # 告诉 catkin 需要哪些 ROS 组件
  roscpp                                # 然后生成 ${catkin_INCLUDE_DIRS} 和 ${catkin_LIBRARIES}
  rospy                                 # 就会包含这些组件的头文件路径和链接库
  geometry_msgs      # 常用几何消息类型,包括 PoseStamped、TwistStamped、Point、Quaternion 等
  nav_msgs           # 如Odometry、Path、OccupancyGrid
  sensor_msgs        # 如PointCloud2、Image、LaserScan
  mavros_msgs        # 如无人机状态、遥控器输入、位置控制等
  std_msgs           # 如Header、Bool、Float32、String等，基础标准消息
  pcl_ros            # ROS 为 PCL 点云提供的桥接包，让 PCL 与 ROS PointCloud2、topic、TF 等协同工作
  pcl_conversions    # 常用于将 PCL 点云转为 ROS 消息，或从 ROS 消息转为 PCL 类型
  cv_bridge          # OpenCV 与 ROS sensor_msgs/Image 之间的桥
  message_filters    # 消息过滤和同步机制,常用于把多个话题（如深度图+位姿、点云+里程计）按时间戳对齐
  visualization_msgs # RViz 可视化消息
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(casadi REQUIRED)

add_message_files(
  FILES
  PredictedTrajectory.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# 包导出信息，描述当前包的信息
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs tf2 tf2_ros tf2_geometry_msgs
#  INCLUDE_DIRS include        #表示这个包的头文件在include/
#  LIBRARIES form_nav          #导出一个叫form_nav的库，会让其他 ROS 包的 find_package(catkin REQUIRED COMPONENTS form_nav) 生效
#  CATKIN_DEPENDS roscpp rospy #表示“如果别人依赖我，也要一起依赖这些 ROS 组件”
#  DEPENDS system_lib          #表示“如果别人依赖我，还要依赖这些系统库”
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
#  编译当前源文件时的头文件搜索路径
include_directories(      
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

# add_executable(cloud_process_node src/cloud_process_node.cpp src/cloud_process.cpp)
# target_link_libraries(cloud_process_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(obstacle_generator src/obstacle_generator.cpp)
# target_link_libraries(obstacle_generator ${catkin_LIBRARIES})

# add_executable(random_forest_generator src/random_forest_generator.cpp)
# target_link_libraries(random_forest_generator ${catkin_LIBRARIES})

add_executable(random_obs src/random_obs.cpp)
target_link_libraries(random_obs ${catkin_LIBRARIES})

add_executable(astar_node src/astar_node.cpp src/a_star.cpp src/grid_map.cpp src/raycast.cpp)
target_link_libraries(astar_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(uav_base src/uav_base.cpp)
target_link_libraries(uav_base ${catkin_LIBRARIES})

add_executable(multi_uav_static_tf src/multi_uav_static_tf.cpp)
target_link_libraries(multi_uav_static_tf ${catkin_LIBRARIES})

add_executable(odom_to_world src/odom_to_world.cpp)
target_link_libraries(odom_to_world ${catkin_LIBRARIES})

# 定义一个可执行程序目标planner_node，让CMake对这四个源文件分别编译，然后把它们链接成一个最终的可执行文件dwa_node
add_executable(dwa_node src/dwa_node.cpp src/dwa_planner.cpp src/sdf_map.cpp src/raycast.cpp)
# 把外部库也链接到这个可执行文件
target_link_libraries(dwa_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})

add_executable(kino_astar_node src/kino_astar_node.cpp src/kino_astar.cpp src/grid_map.cpp src/raycast.cpp)
target_link_libraries(kino_astar_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})

# 仅发布 nav_msgs::Path 路径位置信息
add_executable(kino_astar_path_node src/kino_astar_path_node.cpp src/kino_astar.cpp src/grid_map.cpp src/raycast.cpp)
target_link_libraries(kino_astar_path_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})

# MPC 编队追踪节点 (包含 ESDF CasADi 避障)
add_executable(mpc_node src/mpc_node.cpp src/mpc.cpp src/esdf_casadi.cpp src/sdf_map.cpp src/raycast.cpp)
target_link_libraries(mpc_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES} casadi)
add_dependencies(mpc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
