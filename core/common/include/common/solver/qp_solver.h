/**
 * @file qp_solver.h
 * @brief 二次规划（QP）求解器高级封装
 *
 * 该文件在 OOQP 底层接口的基础上提供了更高层次的 QP 求解器封装。
 * 求解器将常见的轨迹优化问题转化为标准化形式并进行求解：
 *
 *   minimize    (Ax - b)' * S * (Ax - b)  +  x' * W * x
 *   subject to  Cx = c
 *               d <= Dx <= f
 *               l <= x <= u
 *
 * 其中：
 *   - 目标函数第一项 (Ax - b)' * S * (Ax - b)：使 Ax 逼近目标向量 b，
 *     S 为各维度权重矩阵
 *   - 目标函数第二项 x' * W * x：正则化项，使解向量 x 本身尽量小
 *   - Cx = c：等式约束（如起止点位置/速度连续性约束）
 *   - d <= Dx <= f：不等式约束（如加速度/曲率范围约束）
 *   - l <= x <= u：变量边界约束
 *
 * 该求解器被广泛用于轨迹平滑和速度规划中，将离散的轨迹点通过凸优化
 * 拟合为连续光滑的多项式样条曲线。
 *
 * 参考实现：
 *   https://github.com/ethz-asl/ooqp_eigen_interface
 *
 * @author EPSILON Autonomous Driving Team (adapted from ETHZ-ASL)
 * @date 2026
 */

#ifndef _CORE_COMMON_INC_COMMON_SOLVER_QP_SOLVER_H__
#define _CORE_COMMON_INC_COMMON_SOLVER_QP_SOLVER_H__

#include "common/basics/basics.h"

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/SparseCore>

namespace common {

/**
 * @class QuadraticProblem
 * @brief 二次规划问题求解器
 *
 * 提供两个重载的 solve 方法：
 *   - 完整版：包含等式约束、不等式约束和变量边界约束
 *   - 简化版：仅包含等式约束（适用于只需满足连续性条件的平滑问题）
 *
 * 内部通过调用 OoQpItf::solve 完成实际求解。
 */
class QuadraticProblem {
 public:
  /**
   * @brief 求解带完整约束的二次规划问题（完整版）
   *
   * 目标函数：minimize (Ax - b)' * S * (Ax - b) + x' * W * x
   * 约束条件：
   *   - Cx = c        （等式约束）
   *   - d <= Dx <= f  （不等式约束）
   *   - l <= x <= u   （变量边界约束）
   *
   * @param[in] A 投射矩阵（m x n），将优化变量 x 投射到目标空间
   * @param[in] S 目标空间各维度的对角权重矩阵（m x m）
   * @param[in] b 目标向量（m x 1），优化后 Ax 应尽可能接近 b
   * @param[in] W 优化变量的对角正则化权重矩阵（n x n），使解更平滑/更小
   * @param[in] C 等式约束矩阵（m_c x n），可为空矩阵
   * @param[in] c 等式约束右侧向量（m_c x 1）
   * @param[in] D 不等式约束矩阵（m_d x n），可为空矩阵
   * @param[in] d 不等式约束下界向量（m_d x 1）
   * @param[in] f 不等式约束上界向量（m_d x 1）
   * @param[in] l 变量下界向量（n x 1）
   * @param[in] u 变量上界向量（n x 1）
   * @param[out] x 输出的优化变量向量（n x 1）
   * @return true 求解成功，false 求解失败
   *
   * @note 内部将目标函数转换为标准 QP 形式：Q = A'SA + W, c = -2*A'Sb
   */
  static bool solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S,
                    const Eigen::VectorXd& b,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                    const Eigen::VectorXd& c,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& D,
                    const Eigen::VectorXd& d, const Eigen::VectorXd& f,
                    const Eigen::VectorXd& l, const Eigen::VectorXd& u,
                    Eigen::VectorXd& x);

  /**
   * @brief 求解仅带等式约束的二次规划问题（简化版）
   *
   * 目标函数：minimize (Ax - b)' * S * (Ax - b) + x' * W * x
   * 约束条件：Cx = c（仅等式约束）
   *
   * 该简化版本适用于仅需满足位置/速度连续性条件的轨迹平滑问题，
   * 求解速度更快、更稳定。
   *
   * @param[in] A 投射矩阵（m x n）
   * @param[in] S 目标空间权重矩阵（m x m）
   * @param[in] b 目标向量（m x 1）
   * @param[in] W 正则化权重矩阵（n x n）
   * @param[in] C 等式约束矩阵（m_c x n），可为空矩阵
   * @param[in] c 等式约束右侧向量（m_c x 1）
   * @param[out] x 输出的优化变量向量（n x 1）
   * @return true 求解成功，false 求解失败
   */
  static bool solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S,
                    const Eigen::VectorXd& b,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                    const Eigen::VectorXd& c, Eigen::VectorXd& x);
};

}  // namespace common

#endif
