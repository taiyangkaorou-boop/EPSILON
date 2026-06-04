/**
 * @file ooqp_interface.h
 * @brief OOQP（Object-Oriented QP）二次规划求解器接口封装
 *
 * 该文件提供了与 OOQP 二次规划求解器的 Eigen 接口封装。
 * OOQP 是一个面向对象的 QP 求解器库，用于解决以下标准形式的二次规划问题：
 *
 *   minimize    1/2 * x'Qx + c'x
 *   subject to  Ax = b
 *               d <= Cx <= f
 *               l <= x <= u
 *
 * 其中：
 *   - Q 为对称半正定矩阵（n x n）
 *   - c 为线性项系数向量（n x 1）
 *   - A/C 为等式/不等式约束矩阵
 *   - b/d/f 为等式/不等式约束边界
 *   - l/u 为变量上下界
 *
 * 在本项目中，该求解器被用于轨迹平滑和速度优化中的凸优化问题求解，
 * 例如将粗糙的离散轨迹点拟合为光滑的多项式轨迹。
 *
 * 参考实现：
 *   https://github.com/ethz-asl/ooqp_eigen_interface
 *
 * @author EPSILON Autonomous Driving Team (adapted from ETHZ-ASL)
 * @date 2026
 */

#ifndef _CORE_COMMON_INC_COMMON_SOLVER_OOQP_INTERFACE_H__
#define _CORE_COMMON_INC_COMMON_SOLVER_OOQP_INTERFACE_H__

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/SparseCore>

#include <algorithm>  // std::max
#include <cmath>      // std::abs
#include <limits>     // std::numeric_limits

namespace common {

/**
 * @class OoQpItf
 * @brief OOQP 二次规划求解器接口
 *
 * 封装 OOQP 库的调用细节，提供统一的 solve 方法来解决带等式约束、
 * 不等式约束和变量边界约束的二次规划问题。
 */
class OoQpItf {
 public:
  /**
   * @brief 求解二次规划问题
   *
   * 求解 min 1/2 * x'Qx + c'x，约束条件为：
   *   - Ax = b        （等式约束）
   *   - d <= Cx <= f  （不等式约束）
   *   - l <= x <= u   （变量边界约束）
   *
   * @param[in] Q 对称半正定矩阵（n x n），二次项系数矩阵
   * @param[in] c 线性项系数向量（n x 1）
   * @param[in] A 等式约束矩阵（m_a x n），可为空矩阵
   * @param[in] b 等式约束右侧向量（m_a x 1）
   * @param[in] C 不等式约束矩阵（m_c x n），可为空矩阵
   * @param[in] d 不等式约束下界向量（m_c x 1）
   * @param[in] f 不等式约束上界向量（m_c x 1）
   * @param[in] l 变量下界向量（n x 1）
   * @param[in] u 变量上界向量（n x 1）
   * @param[out] x 求解得到的优化变量向量（n x 1）
   * @param ignoreUnknownError 是否忽略未知错误（默认 false）
   * @param verbose 是否打印详细求解信息（默认 false）
   * @return true 求解成功，false 求解失败
   *
   * @note Q 矩阵必须为对称半正定，否则求解可能不收敛
   * @note 所有约束矩阵使用 Eigen 稀疏矩阵格式以提高效率
   */
  static bool solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
                    const Eigen::VectorXd& c,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    const Eigen::VectorXd& b,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                    const Eigen::VectorXd& d, const Eigen::VectorXd& f,
                    const Eigen::VectorXd& l, const Eigen::VectorXd& u,
                    Eigen::VectorXd& x, const bool ignoreUnknownError = false,
                    const bool verbose = false);
 private:
  /**
   * @brief 确定哪些边界约束处于活跃状态
   *
   * 分析变量的上下界，判断哪些变量具有有限的上界或下界约束。
   * 具有 -inf 边界的约束视为无效。
   *
   * @param[in] l 变量下界向量（n x 1）
   * @param[in] u 变量上界向量（n x 1）
   * @param[out] useLowerLimit 各变量是否使用下界约束的标志向量
   * @param[out] useUpperLimit 各变量是否使用上界约束的标志向量
   * @param[out] lowerLimit 有效的下界值向量（无效项填充为 0）
   * @param[out] upperLimit 有效的上界值向量（无效项填充为 0）
   */
  static void generateLimits(
      const Eigen::VectorXd& l, const Eigen::VectorXd& u,
      Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit,
      Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit,
      Eigen::VectorXd& lowerLimit, Eigen::VectorXd& upperLimit);

  /// 调试辅助函数：打印二次规划问题的完整公式化描述
  static void printProblemFormulation(
      const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
      const Eigen::VectorXd& c,
      const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
      const Eigen::VectorXd& b,
      const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
      const Eigen::VectorXd& d, const Eigen::VectorXd& f,
      const Eigen::VectorXd& l, const Eigen::VectorXd& u);

  /// 调试辅助函数：打印变量边界约束的详细信息
  static void printLimits(
      const Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit,
      const Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit,
      const Eigen::VectorXd& lowerLimit, const Eigen::VectorXd& upperLimit);

  /// 调试辅助函数：打印求解状态和优化结果
  static void printSolution(const int status, const Eigen::VectorXd& x);
};

/**
 * @class NumericalUtil
 * @brief 数值计算工具类
 *
 * 提供数值比较和误差分析相关的辅助函数，用于处理浮点数计算中的精度问题。
 */
class NumericalUtil {
 public:
  /**
   * @brief 计算 max(|a|, |b|) * epsilon
   *
   * 用于相对误差比较中的缩放因子计算。
   *
   * @tparam ValueType_ 数值类型（float, double 等）
   * @param a 第一个数值
   * @param b 第二个数值
   * @param epsilon 精度因子（epsilon > 0）
   * @return max(|a|, |b|) * epsilon
   */
  template <typename ValueType_>
  static inline ValueType_ maxTimesEpsilon(const ValueType_ a,
                                           const ValueType_ b,
                                           const ValueType_ epsilon) {
    return std::max(std::abs(a), std::abs(b)) * epsilon;
  }

  /**
   * @brief 检查两个浮点数在相对误差范围内是否近似相等
   *
   * 判断条件：|a - b| <= max(|a|, |b|) * epsilon
   * 该方法的优点是比较精度随数值大小自适应调整。
   *
   * @tparam ValueType_ 数值类型（float, double 等）
   * @param[in] a 第一个待比较数值
   * @param[in] b 第二个待比较数值
   * @param[in] epsilon 相对误差阈值（可选项，默认为该数值类型的机器精度）
   * @return true 近似相等，false 不相等
   */
  template <typename ValueType_>
  static bool ApproximatelyEqual(
      const ValueType_ a, const ValueType_ b,
      ValueType_ epsilon = std::numeric_limits<ValueType_>::epsilon()) {
    return std::abs(a - b) <= maxTimesEpsilon(a, b, epsilon);
  }
};

}  // namespace common

#endif
