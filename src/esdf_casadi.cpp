#include "form_nav/esdf_casadi.h"
#include <iostream>

using namespace casadi;
// 分辨率，地图左后下世界坐标，三轴体素数量，指向 ESDF 距离数组的指针
void EsdfCasadi::init(double resolution, const Eigen::Vector3d& origin,
                       const Eigen::Vector3i& voxel_num, const std::vector<double>* data_ptr) {
    resolution_ = resolution;
    resolution_inv_ = 1.0 / resolution;
    origin_ = origin;
    Nx_ = voxel_num(0); //取voxel_num的第一个元素作为x方向的体素数量
    Ny_ = voxel_num(1);
    Nz_ = voxel_num(2);
    data_ptr_ = data_ptr;

    if (data_ptr_ != nullptr && Nx_ > 0 && Ny_ > 0 && Nz_ > 0) {
        buildInterpolant();
    }
}

void EsdfCasadi::updateMap() {
    if (data_ptr_ != nullptr && Nx_ > 0 && Ny_ > 0 && Nz_ > 0) {
        buildInterpolant();
    }
}
// 构建 CasADi 三线性插值器
void EsdfCasadi::buildInterpolant() {
    // 构建三轴 grid 向量（赋值体素索引）
    std::vector<double> grid_x(Nx_), grid_y(Ny_), grid_z(Nz_);
    for (int i = 0; i < Nx_; ++i) grid_x[i] = static_cast<double>(i); // static_cast为相同类型转换（如数值类型）
    for (int i = 0; i < Ny_; ++i) grid_y[i] = static_cast<double>(i);
    for (int i = 0; i < Nz_; ++i) grid_z[i] = static_cast<double>(i);

    // 将 ESDF 数据按 CasADi interpolant 的列优先顺序展平，x优先转为z优先
    std::vector<double> values_flat(Nx_ * Ny_ * Nz_);
    int idx = 0;
    for (int iz = 0; iz < Nz_; ++iz) {
        for (int iy = 0; iy < Ny_; ++iy) {
            for (int ix = 0; ix < Nx_; ++ix) {
                int addr = ix * Ny_ * Nz_ + iy * Nz_ + iz;
                values_flat[idx++] = (*data_ptr_)[addr];
            }
        }
    }

    // 创建 CasADi interpolant: 3 维线性插值
    std::vector<std::vector<double>> grid_vecs = {grid_x, grid_y, grid_z}; // x*3矩阵，等号后为初始化列表
    // 等价于std::unique_ptr<Function>( new Function(arg1, arg2, ...) )
    interp_func_ = std::make_unique<Function>(
        // interpolant(name, solver, vector<vector<double>> grid, vector<double> values)，casadi函数
        // solver = "bspline" (degree=1 → 等价线性插值，且支持 MX 符号求值/eval_sx)
        interpolant("esdf_interp", "bspline", grid_vecs, values_flat,
                    {{"degree", std::vector<int>{1, 1, 1}}})
    );

    std::cout << "[EsdfCasadi] Interpolant built: grid=" << Nx_ << "x" << Ny_ << "x" << Nz_
              << ", res=" << resolution_ << "m" << std::endl;
}

MX EsdfCasadi::getDistMX(const MX& pos_mx) const {
    if (!interp_func_ || data_ptr_ == nullptr) {
        // 无地图时返回一个很大的距离（视为远离障碍物）
        return MX(100.0);
    }

    // 提取位置分量的 MX
    MX px = pos_mx(0);
    MX py = pos_mx(1);
    MX pz = pos_mx(2);

    // 转为体素浮点索引: idx_float = (pos - origin) / resolution
    MX idx_x = (px - origin_(0)) * resolution_inv_;
    MX idx_y = (py - origin_(1)) * resolution_inv_;
    MX idx_z = (pz - origin_(2)) * resolution_inv_;

    // 调用 interpolant Function
    // 输入: 3x1 的 [float_idx_x, float_idx_y, float_idx_z]
    MX idx_in = MX::vertcat({idx_x, idx_y, idx_z});

    // 调用 interpolant，返回 interpolated distance
    // 返回类型为std::vector<MX>
    std::vector<MX> ret = (*interp_func_)({idx_in});
    return ret[0];
}

MX EsdfCasadi::getDistWithGradMX(const MX& pos_mx, MX& grad_mx) const {
    MX dist = getDistMX(pos_mx);

    // 使用 CasADi 的 jacobian 函数获取梯度
    // jacobian(dist, pos_mx) 返回 1x3 的梯度
    grad_mx = jacobian(dist, pos_mx).T();

    return dist;
}