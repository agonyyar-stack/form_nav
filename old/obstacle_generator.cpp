#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cmath>
#include <random>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// 立方体障碍物结构体（中心坐标 + 边长）
struct CubeObstacle
{
    double cx, cy;   // 中心XY坐标
    double size;     // 立方体边长
    double z_min;    // 最低高度
    double z_max;    // 最高高度
};

class ObstacleCloudSimulator
{
public:
    ObstacleCloudSimulator(ros::NodeHandle& nh) 
        : nh_(nh), time_(0.0), dt_(0.1)
    {
        // 发布仿真点云话题
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_cloud", 10);
        
        // 1. 50m×50m区域（原点中心）随机生成静态立方体（不重叠+原点1.5米禁生）
        initStaticCubes();
    }

    void runOnce()
    {
        PointCloud::Ptr cloud(new PointCloud);
        cloud->header.frame_id = "map";
        cloud->height = 1;
        cloud->is_dense = true;

        // 生成所有静态立方体点云
        for (auto& cube : static_cubes_)
        {
            generateCubeCloud(cloud, cube);
        }

        // 生成10个动态立方体点云（固定轨迹、无碰撞）
        generateDynamicCubes(cloud);

        // 转换并发布点云
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp = ros::Time::now();
        pub_.publish(msg);

        time_ += dt_;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    double time_;
    double dt_;

    // 静态立方体列表
    std::vector<CubeObstacle> static_cubes_;

    // 50m×50m区域范围
    const double AREA_HALF = 25.0;
    // 立方体边长（静态+动态统一）
    const double CUBE_SIZE = 0.6;
    // 点云采样间距
    const double POINT_STEP = 0.1;
    // 原点禁生半径：1.5米
    const double ORIGIN_FORBID_RADIUS = 1.5;

    // =============================================
    // 初始化：50m×50m区域 随机不重叠静态立方体
    // 新增：原点为中心 半径1.5米内禁止生成
    // =============================================
    void initStaticCubes()
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        // 均匀分布：x:-25~25，y:-25~25
        std::uniform_real_distribution<> x_rand(-AREA_HALF + 1.0, AREA_HALF - 1.0);
        std::uniform_real_distribution<> y_rand(-AREA_HALF + 1.0, AREA_HALF - 1.0);

        // 生成100个静态立方体（不重叠）
        int cube_num = 100;
        for (int i = 0; i < cube_num; i++)
        {
            CubeObstacle cube;
            cube.size = CUBE_SIZE;
            cube.z_min = 0;
            cube.z_max = 2.0;

            // 循环采样，确保：不重叠 + 不在原点1.5米内
            bool overlap = false;
            do
            {
                overlap = false;
                cube.cx = x_rand(gen);
                cube.cy = y_rand(gen);

                // 条件1：禁止在原点1.5米范围内生成
                double dist_to_origin = hypot(cube.cx, cube.cy);
                if (dist_to_origin < ORIGIN_FORBID_RADIUS)
                {
                    overlap = true;
                    continue;
                }

                // 条件2：不与已有立方体重叠
                for (auto& exist : static_cubes_)
                {
                    double dist = hypot(cube.cx - exist.cx, cube.cy - exist.cy);
                    if (dist < CUBE_SIZE * 1.2) // 留安全余量
                    {
                        overlap = true;
                        break;
                    }
                }
            } while (overlap);

            static_cubes_.push_back(cube);
        }
    }

    // =============================================
    // 生成【单个立方体】点云（点间距严格0.1m）
    // =============================================
    void generateCubeCloud(PointCloud::Ptr& cloud, const CubeObstacle& cube)
    {
        double half_size = cube.size / 2.0;
        // X方向采样：0.1m步长
        for (double x = cube.cx - half_size; x <= cube.cx + half_size; x += POINT_STEP)
        {
            // Y方向采样：0.1m步长
            for (double y = cube.cy - half_size; y <= cube.cy + half_size; y += POINT_STEP)
            {
                // Z方向采样：0.1m步长
                for (double z = cube.z_min; z <= cube.z_max; z += POINT_STEP)
                {
                    cloud->push_back(pcl::PointXYZ(x, y, z));
                }
            }
        }
    }

    // =============================================
    // 生成10个动态障碍物（固定轨迹、无碰撞、不重叠、不进原点禁行区）
    // 轨迹类型：圆周/水平往复/竖直往复/椭圆/斜向运动，分布全区域
    // =============================================
    void generateDynamicCubes(PointCloud::Ptr& cloud)
    {
        CubeObstacle dyn_cube;
        dyn_cube.size = CUBE_SIZE;
        dyn_cube.z_min = 0.1;
        dyn_cube.z_max = 1.5;
        double speed = 0.15;

        // 1. 右上：大圆周运动
        dyn_cube.cx = 12.0 + 4.0 * cos(time_ * speed);
        dyn_cube.cy = 12.0 + 4.0 * sin(time_ * speed);
        generateCubeCloud(cloud, dyn_cube);

        // 2. 左上：水平长距离往复
        dyn_cube.cx = -14.0 + 5.0 * cos(time_ * speed);
        dyn_cube.cy = 10.0;
        generateCubeCloud(cloud, dyn_cube);

        // 3. 右下：竖直长距离往复
        dyn_cube.cx = 10.0;
        dyn_cube.cy = -14.0 + 5.0 * cos(time_ * speed);
        generateCubeCloud(cloud, dyn_cube);

        // 4. 左下：大椭圆运动
        dyn_cube.cx = -12.0 + 4.5 * cos(time_ * speed);
        dyn_cube.cy = -12.0 + 3.5 * sin(time_ * speed);
        generateCubeCloud(cloud, dyn_cube);

        // 5. 右侧：水平短往复
        dyn_cube.cx = 18.0 + 3.0 * cos(time_ * speed);
        dyn_cube.cy = 0.0;
        generateCubeCloud(cloud, dyn_cube);

        // 6. 左侧：竖直短往复
        dyn_cube.cx = -18.0;
        dyn_cube.cy = 0.0 + 3.0 * cos(time_ * speed);
        generateCubeCloud(cloud, dyn_cube);

        // 7. 上侧：斜向椭圆
        dyn_cube.cx = 0.0 + 5.0 * cos(time_ * speed);
        dyn_cube.cy = 18.0 + 2.5 * sin(time_ * speed);
        generateCubeCloud(cloud, dyn_cube);

        // 8. 下侧：斜向直线往复
        dyn_cube.cx = 0.0 + 4.0 * cos(time_ * speed);
        dyn_cube.cy = -18.0 + 4.0 * cos(time_ * speed);
        generateCubeCloud(cloud, dyn_cube);

        // 9. 右中：小圆周运动
        dyn_cube.cx = 8.0 + 2.0 * cos(time_ * speed);
        dyn_cube.cy = -6.0 + 2.0 * sin(time_ * speed);
        generateCubeCloud(cloud, dyn_cube);

        // 10. 左中：小椭圆运动
        dyn_cube.cx = -8.0 + 2.5 * cos(time_ * speed);
        dyn_cube.cy = 6.0 + 1.5 * sin(time_ * speed);
        generateCubeCloud(cloud, dyn_cube);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_cloud_simulator");
    ros::NodeHandle nh;
    ObstacleCloudSimulator sim(nh);

    ros::Rate rate(10);
    while (ros::ok())
    {
        sim.runOnce();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}