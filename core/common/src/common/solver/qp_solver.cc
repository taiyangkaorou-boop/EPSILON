/**
 * @file qp_solver.cc
 * @brief 二次规划（QP）求解器包装器的实现文件
 *
 * 本文件实现了EPSILON自动驾驶规划系统中的 QuadraticProblem 类，
 * 它将带权重的加权最小二乘问题转化为标准QP问题，并调用OOQP求解器。
 *
 * ==================== QuadraticProblem 核心功能 ====================
 *
 * 本求解器解决以下形式的加权最小二乘 / Tikhonov正则化问题：
 *
 *   f(x) = (Ax - b)' * S * (Ax - b) + x' * W * x
 *
 * 其中：
 *   - S: 样本权重矩阵（对角矩阵），表示每个样本点的重要性
 *   - W: 正则化矩阵（对角矩阵），控制解的光滑度
 *
 * ==================== 转换为标准QP形式 ====================
 *
 * 展开目标函数：
 *   f(x) = x'A'SAx - 2x'A'Sb + b'Sb + x'Wx
 *        = x'(A'SA + W)x - 2x'A'Sb + b'Sb
 *
 * 忽略常数项 b'Sb，最小化 f(x) 等价于求解标准QP：
 *   min 1/2 * x' * Q * x + c' * x
 *   其中 Q = A' * S * A + W,  c = -A' * S * b
 *
 * 然后调用 OoQpItf::solve(Q, c, C_eq, c_eq, C_ineq, d, f, l, u, x)
 *
 * ==================== 两个重载版本 ====================
 *
 * 1. 完整版：包含等式约束 C*x=c、不等式约束 d<=D*x<=f、变量边界 l<=x<=u
 * 2. 简化版：仅有等式约束 C*x=c，无不等式约束和变量边界
 *    （内部将变量边界设为[-inf, +inf]，不等式约束设为空）
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/solver/qp_solver.h"

#include <stdexcept>

#include "common/solver/ooqp_interface.h"

namespace common {

/*
 * QuadraticProblem::solve - 完整版QP求解器
 *
 * 解决以下优化问题：
 *   min  (Ax - b)' * S * (Ax - b) + x' * W * x
 *   s.t. C * x = c           (等式约束)
 *        d <= D * x <= f      (不等式约束)
 *        l <= x <= u          (变量边界约束)
 *
 * 步骤：
 * 1. 构造Q = A' * S * A + W  (Hessian矩阵)
 *    - 注意 S 和 W 都是对角矩阵
 *    - S.toDenseMatrix().sparseView() 将稠密对角转为稀疏格式
 * 2. 构造c = -A' * S * b  (线性项系数)
 * 3. 调用 OoQpItf::solve 求解标准QP
 *
 * @param A 样本矩阵 (m×n)
 * @param S 样本权重对角矩阵 (m×m)
 * @param b 目标向量 (m维)
 * @param W 正则化权重对角矩阵 (n×n)
 * @param C 等式约束矩阵
 * @param c 等式约束右端向量
 * @param D 不等式约束矩阵
 * @param d 不等式约束下界向量
 * @param f 不等式约束上界向量
 * @param l 变量下界向量
 * @param u 变量上界向量
 * @param x 输出参数，最优解向量 (n维)
 * @return true 求解成功
 */
bool QuadraticProblem::solve(
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S,
    const Eigen::VectorXd& b,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
    const Eigen::VectorXd& c,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& D,
    const Eigen::VectorXd& d, const Eigen::VectorXd& f,
    const Eigen::VectorXd& l, const Eigen::VectorXd& u, Eigen::VectorXd& x) {
  int m = A.rows();  // 样本数量
  int n = A.cols();  // 变量数量
  x.setZero(n);
  assert(static_cast<int>(b.size()) == m);
  assert(static_cast<int>(S.rows()) == m);
  assert(static_cast<int>(W.rows()) == n);

  // 构造Hessian: Q = A' * S * A + W
  Eigen::SparseMatrix<double, Eigen::RowMajor> Q_temp;
  Q_temp = A.transpose() * S * A +
           (Eigen::SparseMatrix<double, Eigen::RowMajor>)W.toDenseMatrix()
               .sparseView();

  // 构造线性项: c_temp = -A' * S * b
  Eigen::VectorXd c_temp = -A.transpose() * S * b;

  // 调用OOQP求解器求解标准QP问题
  return OoQpItf::solve(Q_temp, c_temp, C, c, D, d, f, l, u, x);
}

/*
 * QuadraticProblem::solve - 简化版QP求解器
 *
 * 仅包含等式约束 C*x = c，无不等式约束和变量边界。
 *
 * 内部实现：
 *   - 变量边界设为 [-inf, +inf]（不活跃）
 *   - 不等式约束矩阵 D 为空（无不等式约束）
 *   - 然后调用完整版 solve 函数
 *
 * @param A 样本矩阵 (m×n)
 * @param S 样本权重对角矩阵
 * @param b 目标向量
 * @param W 正则化权重对角矩阵
 * @param C 等式约束矩阵
 * @param c 等式约束右端向量
 * @param x 输出参数，最优解向量
 * @return true 求解成功
 */
bool QuadraticProblem::solve(
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S,
    const Eigen::VectorXd& b,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
    const Eigen::VectorXd& c, Eigen::VectorXd& x) {
  int nx = A.cols();
  // 变量边界设为无穷大（不施加约束）
  Eigen::VectorXd u =
      std::numeric_limits<double>::max() * Eigen::VectorXd::Ones(nx);
  Eigen::VectorXd l = (-u.array()).matrix();
  // 空的不等式约束矩阵
  Eigen::SparseMatrix<double, Eigen::RowMajor> D;
  Eigen::VectorXd d, f;
  // 委托给完整版求解
  return solve(A, S, b, W, C, c, D, d, f, l, u, x);
}

}  // namespace common
