/**
 * @file bezier.h
 * @brief Bezier 样条曲线模板类及工具函数
 *
 * 本文件定义了 EPSILON 自动驾驶规划系统中的 Bezier 样条曲线类
 * BezierSpline 及其伴随的工具类 BezierUtils。
 *
 * Bezier 样条是一种使用控制点（control points）和 Bernstein 基函数
 * 定义的参数曲线。与普通多项式相比，Bezier 曲线具有以下显著优势：
 *   - 凸包性质（Convex Hull Property）：曲线完全包含在其控制点构成的
 *     凸包之内。这一性质在自动驾驶中极为重要——如果车道边界是凸多面体，
 *     那么只要保证控制点在边界内，整个曲线就一定在边界内。
 *     这使得 Bezier 曲线非常适合在空间走廊（corridor）约束下进行
 *     轨迹优化。
 *
 *   - 几何直观性：控制点的空间位置直接决定了曲线的整体形状，
 *     便于交互式调整和调试。
 *
 *   - 端点插值性质：曲线的起点等于第一个控制点，终点等于最后一个控制点，
 *     起点切向量方向沿第一个控制点到第二个控制点的连线方向。
 *
 * 本实现中 Bezier 样条的数学表示：
 *   B_j(t) = s_j * sum_{i=0}^{N_DEG} c_j^i * b_{N_DEG}^i((t - T_j) / s_j)
 *
 *   其中：
 *     - j 为段索引
 *     - c_j^i 为第 j 段第 i 个控制点（N_DIM 维空间中的点）
 *     - b_{N_DEG}^i 为 N_DEG 次 Bernstein 基函数
 *     - T_j 为第 j 段的时间偏移
 *     - s_j 为第 j 段的持续时间（缩放因子）
 *
 * 当前实现主要支持五次（N_DEG=5）Bezier 曲线，因为五次曲线可以在
 * 保证位置、速度、加速度连续性的同时提供 jerk 优化自由度。
 *
 * BezierUtils 类提供了：
 *   - GetBezierBasis: 计算 Bernstein 基函数及其各阶导数在给定参数处的值
 *   - GetBezierHessianMat: 计算 jerk/hessian 矩阵（用于 QP 优化中的能量最小化）
 */

#ifndef _CORE_COMMON_INC_COMMON_SPLINE_BEZIER_H__
#define _CORE_COMMON_INC_COMMON_SPLINE_BEZIER_H__

#include <assert.h>

#include <vector>

namespace common {

/// 前向声明，BezierSpline 需要使用 BezierUtils 进行求值
template <int N_DEG>
class BezierUtils;

/**
 * @class BezierSpline
 * @brief Bezier 样条曲线模板类
 *
 * 该类封装了一条由多个 Bezier 曲线段组成的分段参数化曲线。每段
 * 由 N_DEG+1 个 N_DIM 维控制点定义。由于 Bezier 曲线具有凸包性质，
 * 只要控制点位于安全区域内，整条曲线也必然位于安全区域内，
 * 这使得该类非常适合在受约束的空间走廊（corridor）中进行轨迹规划。
 *
 * @tparam N_DEG Bezier 曲线的次数（如 5 表示五次 Bezier 曲线）
 * @tparam N_DIM 控制点的空间维度（如 2 表示二维平面，3 表示空间+时间）
 *
 * 参数化说明：
 *   - vec_domain_ 是全局时间标尺（各段共享统一的参数轴）
 *   - 每段的局部归一化参数 t = (s - vec_domain_[idx]) / duration
 *     其中 duration = vec_domain_[idx+1] - vec_domain_[idx]，t ∈ [0, 1]
 */
template <int N_DEG, int N_DIM>
class BezierSpline {
 public:
  /// 默认构造函数，创建空的 Bezier 样条
  BezierSpline() {}

  /**
   * @brief 设置样条曲线的定义域划分并初始化控制点矩阵
   *
   * 为每段分配一个 (N_DEG+1) x N_DIM 的控制点矩阵，并初始化为零。
   *
   * @param vec_domain 定义域划分向量，长度 = 分段数 + 1，且严格单调递增
   */
  void set_vec_domain(const std::vector<decimal_t>& vec_domain) {
    assert(vec_domain.size() > 1);
    vec_domain_ = vec_domain;
    ctrl_pts_.resize(vec_domain.size() - 1);
    for (int j = 0; j < static_cast<int>(ctrl_pts_.size()); j++) {
      ctrl_pts_[j].setZero();  // 控制点矩阵初始化为零
    }
  }

  /**
   * @brief 设置第 segment_idx 段第 ctrl_pt_index 个控制点的值
   * @param segment_idx 段索引（0-based）
   * @param ctrl_pt_index 控制点在该段内的索引（0-based，0..N_DEG）
   * @param coeff 控制点的 N_DIM 维坐标
   */
  void set_coeff(const int segment_idx, const int ctrl_pt_index,
                 const Vecf<N_DIM>& coeff) {
    ctrl_pts_[segment_idx].row(ctrl_pt_index) = coeff;
  }

  /**
   * @brief 一次性设置所有段的控制点
   * @param pts 控制点向量，pts[j] 是第 j 段的 (N_DEG+1) x N_DIM 矩阵
   */
  void set_ctrl_pts(const vec_E<Matf<N_DEG + 1, N_DIM>>& pts) {
    ctrl_pts_ = pts;
  }

  /**
   * @brief 获取样条曲线的分段数
   * @return Bezier 曲线段的数量
   */
  int num_segments() const { return static_cast<int>(ctrl_pts_.size()); }

  /**
   * @brief 获取样条曲线的定义域划分向量
   * @return 包含所有时间断点的向量
   */
  std::vector<decimal_t> vec_domain() const { return vec_domain_; }

  /**
   * @brief 获取全部控制点（const 引用）
   * @return 所有段控制点矩阵的向量
   */
  vec_E<Matf<N_DEG + 1, N_DIM>> ctrl_pts() const { return ctrl_pts_; }

  /**
   * @brief 获取样条曲线参数域的起点
   * @return 第一个时间断点的值
   */
  decimal_t begin() const {
    if (vec_domain_.size() < 1) return 0.0;
    return vec_domain_.front();
  }

  /**
   * @brief 获取样条曲线参数域的终点
   * @return 最后一个时间断点的值
   */
  decimal_t end() const {
    if (vec_domain_.size() < 1) return 0.0;
    return vec_domain_.back();
  }

  /**
   * @brief 在指定参数处求值（带导数阶数）
   *
   * 求值过程：
   *   1. 确定 s 所属的分段 idx
   *   2. 计算局部归一化参数 normalized_s = (s - vec_domain_[idx]) / duration
   *   3. 调用 BezierUtils 计算 Bernstein 基函数在 normalized_s 处的 d 阶导数值
   *   4. 通过基函数向量与控制点矩阵的线性组合得到 d 阶导数值
   *
   * 链式法则缩放因子 pow(duration, 1 - d) 用于补偿归一化参数化引入的微分缩放。
   *
   * @param s 参数值（必须位于定义域内，否则返回 kIllegalInput）
   * @param d 求导阶数（0=位置, 1=一阶导数, ...）
   * @param ret 输出参数，N_DIM 维的求值结果向量
   * @return ErrorType 操作是否成功
   */
  ErrorType evaluate(const decimal_t s, const int d, Vecf<N_DIM>* ret) const {
    int num_pts = vec_domain_.size();
    if (num_pts < 1) return kIllegalInput;

    // 二分查找定位参数 s 所属的分段
    auto it = std::lower_bound(vec_domain_.begin(), vec_domain_.end(), s);
    int idx =
        std::min(std::max(static_cast<int>(it - vec_domain_.begin()) - 1, 0),
                 num_pts - 2);
    decimal_t h = s - vec_domain_[idx];
    if (s < vec_domain_[0]) {
      return kIllegalInput;  // Bezier 不支持外推
    } else {
      decimal_t duration = vec_domain_[idx + 1] - vec_domain_[idx];
      decimal_t normalized_s = h / duration;  // 归一化到 [0, 1]

      // 获得归一化 Bernstein 基函数在 normalized_s 处的 d 阶导数值
      Vecf<N_DEG + 1> basis =
          BezierUtils<N_DEG>::GetBezierBasis(d, normalized_s);

      // 通过链式法则补偿归一化缩放因子
      // 对于位置 (d=0): scale = duration，因为 dt/d(norm) = duration
      // 对于速度 (d=1): scale = 1，因为参数的分母 duration 消除
      // 对于加速度 (d=2): scale = 1/duration，以此类推
      Vecf<N_DIM> result =
          (pow(duration, 1 - d) * basis.transpose() * ctrl_pts_[idx])
              .transpose();
      *ret = result;
    }
    return kSuccess;
  }

  /**
   * @brief 打印 Bezier 样条控制点（调试用）
   *
   * 逐段打印每段的控制点矩阵。
   */
  void print() const {
    printf("Bezier control points.\n");
    for (int j = 0; j < static_cast<int>(ctrl_pts_.size()); j++) {
      printf("segment %d -->.\n", j);
      std::cout << ctrl_pts_[j] << std::endl;
    }
  }

 private:
  /// 控制点矩阵向量：ctrl_pts_[seg_idx] 是一个 (N_DEG+1) x N_DIM 矩阵
  /// 每行是一个控制点的 N_DIM 维坐标
  vec_E<Matf<N_DEG + 1, N_DIM>> ctrl_pts_;

  /// 定义域划分向量（全局时间标尺）
  std::vector<decimal_t> vec_domain_;
};

/**
 * @class BezierUtils
 * @brief Bezier 曲线的工具函数类（纯静态）
 *
 * 提供 Bezier 曲线的 Bernstein 基函数计算和相关的 Hessian 矩阵，
 * 主要用于 BezierSpline 的求值和 Bezier 优化问题中能量项的计算。
 *
 * @tparam N_DEG Bezier 曲线的次数，当前主要支持 N_DEG=5（五次曲线）
 */
template <int N_DEG>
class BezierUtils {
 public:
  /**
   * @brief 计算非缩放 Bezier 基函数的 Hessian 矩阵
   *
   * Hessian 矩阵 H_ij = integral (b_i^{(d)}(t) * b_j^{(d)}(t) dt)
   * 用于在 QP 优化中计算能量项（如 jerk 的积分平方值）。
   *
   * 当前支持：
   *   - N_DEG=5, derivative_degree=3 (jerk 能量 Hessian，6x6 矩阵）
   *
   * @param derivative_degree 导数的阶数（决定优化的是哪一阶导数的能量）
   * @return (N_DEG+1) x (N_DEG+1) 的 Hessian 矩阵
   */
  static MatNf<N_DEG + 1> GetBezierHessianMat(int derivative_degree) {
    MatNf<N_DEG + 1> hessian;
    switch (N_DEG) {
      case 5: {
        if (derivative_degree == 3) {
          // 五次 Bezier 曲线 jerk 能量 (d=3) 的 Hessian 矩阵
          // 该矩阵通过符号积分 B_i'''(t) * B_j'''(t) dt 得到
          hessian << 720.0, -1800.0, 1200.0, 0.0, 0.0, -120.0, -1800.0, 4800.0,
              -3600.0, 0.0, 600.0, 0.0, 1200.0, -3600.0, 3600.0, -1200.0, 0.0,
              0.0, 0.0, 0.0, -1200.0, 3600.0, -3600.0, 1200.0, 0.0, 600.0, 0.0,
              -3600.0, 4800.0, -1800.0, -120.0, 0.0, 0.0, 1200.0, -1800.0,
              720.0;
          break;
        } else {
          assert(false);  // 仅支持 jerk (d=3) 的 Hessian
        }
        break;
      }
      default:
        assert(false);  // 仅支持 N_DEG=5 的 Hessian 计算
    }
    return hessian;
  }

  /**
   * @brief 计算非缩放 Bernstein 基函数在归一化参数 t 处的 d 阶导数值
   *
   * 对于五次 (N_DEG=5) Bezier 曲线，六个 Bernstein 基函数为：
   *   b_0^5(t) = -(t-1)^5
   *   b_1^5(t) = 5 t (t-1)^4
   *   b_2^5(t) = -10 t^2 (t-1)^3
   *   b_3^5(t) = 10 t^3 (t-1)^2
   *   b_4^5(t) = -5 t^4 (t-1)
   *   b_5^5(t) = t^5
   *
   * 该函数返回它们在 [0,1] 范围内 t 处的 d 阶导数值组成的向量。
   * 乘以对应的控制点后即可得到曲线在 t 处的 d 阶导数值。
   *
   * @param derivative_degree 导数阶数（0=基函数本身, 1=一阶导, 2=二阶导, 3=三阶导）
   * @param t 归一化参数，范围 [0, 1]，超出会被 clamp
   * @return N_DEG+1 维基函数导数值向量
   */
  static Vecf<N_DEG + 1> GetBezierBasis(int derivative_degree, decimal_t t) {
    t = std::max(std::min(1.0, t), 0.0);  // 将 t 限制在 [0, 1]
    Vecf<N_DEG + 1> basis;
    switch (N_DEG) {
      case 5:
        if (derivative_degree == 0) {
          // 零阶：Bernstein 基函数 B_i^5(t)
          basis << -pow(t - 1, 5), 5 * t * pow(t - 1, 4),
              -10 * pow(t, 2) * pow(t - 1, 3), 10 * pow(t, 3) * pow(t - 1, 2),
              -5 * pow(t, 4) * (t - 1), pow(t, 5);
        } else if (derivative_degree == 1) {
          // 一阶：基函数的 dt 导数
          basis << -5 * pow(t - 1, 4),
              20 * t * pow(t - 1, 3) + 5 * pow(t - 1, 4),
              -20 * t * pow(t - 1, 3) - 30 * pow(t, 2) * pow(t - 1, 2),
              10 * pow(t, 3) * (2 * t - 2) + 30 * pow(t, 2) * pow(t - 1, 2),
              -20 * pow(t, 3) * (t - 1) - 5 * pow(t, 4), 5 * pow(t, 4);
        } else if (derivative_degree == 2) {
          // 二阶：基函数的 dt^2 导数
          basis << -20 * pow(t - 1, 3),
              60 * t * pow(t - 1, 2) + 40 * pow(t - 1, 3),
              -120 * t * pow(t - 1, 2) - 20 * pow(t - 1, 3) -
                  30 * t * t * (2 * t - 2),
              60 * t * pow(t - 1, 2) + 60 * t * t * (2 * t - 2) +
                  20 * t * t * t,
              -60 * t * t * (t - 1) - 40 * t * t * t, 20 * t * t * t;
        } else if (derivative_degree == 3) {
          // 三阶：基函数的 dt^3 导数（jerk）
          basis << -60 * pow(t - 1, 2),
              60 * t * (2 * t - 2) + 180 * pow(t - 1, 2),
              -180 * t * (2 * t - 2) - 180 * pow(t - 1, 2) - 60 * t * t,
              180 * t * (2 * t - 2) + 60 * pow(t - 1, 2) + 180 * t * t,
              -120 * t * (t - 1) - 180 * t * t, 60 * t * t;
        }
        break;
      default:
        printf("N_DEG %d, derivative_degree %d.\n", N_DEG, derivative_degree);
        assert(false);  // 仅支持 N_DEG=5
        break;
    }
    return basis;
  }
};

}  // namespace common

#endif
