#ifndef ESDF_CASADI_H
#define ESDF_CASADI_H

#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <vector>
#include <memory>

/**
 * @brief 将 ESDF 地图的三线性插值过程表达为 CasADi MX 符号表达式，
 *        使障碍物距离能够参与求解器的自动求梯度过程。
 *
 * 核心思路：
 *   使用 CasADi 内置的 interpolant 函数对象构建 ESDF 网格上的三线性插值器。
 *   该 interpolant 以 Function 形式存在，接受 MX/SX 输入并返回 MX 输出，
 *   因此可以嵌入 Opti 优化问题的符号图中，IPOPT 可以对其自动求梯度。
 *
 * 用法:
 *   1. 构造时传入 ESDF 地图的元信息（分辨率、原点、体素数量、距离数据指针）
 *   2. 调用 getDistMX(pos_mx) 获取距离的 MX 表达式
 *   3. 当 ESDF 地图更新时，调用 updateMap() 重建 interpolant
 */
class EsdfCasadi {
public:
    EsdfCasadi()
        : data_ptr_(nullptr), Nx_(0), Ny_(0), Nz_(0),
          resolution_(1.0), resolution_inv_(1.0),
          origin_(Eigen::Vector3d::Zero()),
          interp_func_(nullptr) {}

    ~EsdfCasadi() = default;

    /**
     * @brief 用 ESDF 地图参数初始化并构建插值器
     * @param resolution   体素分辨率 (m)
     * @param origin       地图原点 (m)
     * @param voxel_num    体素数量 (Nx, Ny, Nz)
     * @param data_ptr     指向 distance_buffer_all_ 的指针
     */
    void init(double resolution, const Eigen::Vector3d& origin,
              const Eigen::Vector3i& voxel_num, const std::vector<double>* data_ptr);

    /**
     * @brief 更新底层数据并重建插值器（当 ESDF 地图在外部被重建后调用）
     */
    void updateMap();

    /**
     * @brief 更新底层数据指针（不触发重建，需手动调用 updateMap）
     */
    void setDataPtr(const std::vector<double>* data_ptr) {
        data_ptr_ = data_ptr;
    }

    /**
     * @brief 判断位置是否在地图范围内
     */
    bool isInMap(const Eigen::Vector3d& pos) const { Eigen::Vector3d max_bound = 
        origin_ + Eigen::Vector3d(Nx_ * resolution_, Ny_ * resolution_, Nz_ * resolution_);
        return (pos(0) >= origin_(0) && pos(0) <= max_bound(0) &&
                pos(1) >= origin_(1) && pos(1) <= max_bound(1) &&
                pos(2) >= origin_(2) && pos(2) <= max_bound(2));
    }

    /**
     * @brief 获取 ESDF 距离的 CasADi MX 表达式（三线性插值）
     * @param pos_mx  3x1 MX 位置向量 [x, y, z]^T (世界坐标系)
     * @return 标量 MX 距离值
     *
     * 内部调用预构建的 interpolant Function，该 Function 接受 MX 输入并返回 MX 输出。
     */
    casadi::MX getDistMX(const casadi::MX& pos_mx) const;

    /**
     * @brief 获取 ESDF 距离和梯度的 CasADi MX 表达式
     * @param pos_mx  3x1 MX 位置向量
     * @param grad_mx 输出 3x1 MX 梯度向量 (jacobian)
     * @return 标量 MX 距离值
     */
    casadi::MX getDistWithGradMX(const casadi::MX& pos_mx, casadi::MX& grad_mx) const;

    // 地图参数访问
    double getResolution() const { return resolution_; }
    const Eigen::Vector3d& getOrigin() const { return origin_; }
    Eigen::Vector3i getVoxelNum() const { return Eigen::Vector3i(Nx_, Ny_, Nz_); }
    bool isInitialized() const { return interp_func_ != nullptr; }

private:
    /// 重建 CasADi interpolant Function
    void buildInterpolant();

    // ESDF 地图参数
    const std::vector<double>* data_ptr_;
    int Nx_, Ny_, Nz_;
    double resolution_, resolution_inv_;
    Eigen::Vector3d origin_;

    // CasADi 三线性插值 Function
    // 输入: 3x1 [idx_x_float, idx_y_float, idx_z_float] (体素浮点索引)
    // 输出: 标量 距离值
    std::unique_ptr<casadi::Function> interp_func_;
};

#endif  // ESDF_CASADI_H