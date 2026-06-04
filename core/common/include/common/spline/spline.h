/**
 * @file spline.h
 * @brief 分段多项式样条曲线模板类
 *
 * 本文件定义了 EPSILON 自动驾驶规划系统中的通用分段多项式样条曲线
 * Spline 类。该类是一个分段参数化曲线，由多个多项式段（PolynomialND）
 * 组成，每个段在其对应的定义域区间上是有效的。
 *
 * Spline 类的基本结构：
 *   - vec_domain_: 定义域划分向量 [d0, d1, d2, ..., d{n}]，其中 d{i} 是第 i 个断点
 *   - poly_: 多项式段向量，poly_[i] 在区间 [d{i}, d{i+1}] 上有效
 *   - 每条 spline 共有 n-1 个多项式段（n = vec_domain_.size()）
 *
 * 每个多项式段的参数化方式：对于参数 s ∈ [d{i}, d{i+1}]，定义 h = s - d{i}，
 * 则求值公式为 P_i(h)，其中 P_i 是第 i 段的 N_DIM 维多项式。
 * 这种局部参数化方式避免了远端参数引起的大数值问题。
 *
 * 外推行为：当求值参数 s 超出定义域时，Spline 使用最近的多项式段
 * 进行外推（而非截断或报错），这在某些鲁棒性需求的场景下是合理的。
 *
 * @tparam N_DEG 多项式的次数（如 5 表示五次多项式）
 * @tparam N_DIM 曲线的输出维度（如 2 表示二维位置曲线 (x, y)）
 */

#ifndef _CORE_COMMON_INC_COMMON_SPLINE_SPLINE_H__
#define _CORE_COMMON_INC_COMMON_SPLINE_SPLINE_H__

#include "common/spline/polynomial.h"

#include <assert.h>
#include <vector>

namespace common {

/**
 * @class Spline
 * @brief 分段多项式样条曲线模板类
 *
 * 该类封装了一条由多个多项式段组成的分段参数化曲线。每段在其局部定义
 * 区间上独立定义，并通过断点（breakpoints）连接组成完整的曲线域。
 * 相邻段之间在断点处的连续性由构造时保证（如通过 SplineGenerator 求解）。
 *
 * @tparam N_DEG 各段多项式的次数
 * @tparam N_DIM 曲线的输出维度（通常是 2，表示 (x(s), y(s))）
 */
template <int N_DEG, int N_DIM>
class Spline {
 public:
  /// 每个多项式段的类型别名：N_DIM 维度的 N_DEG 次多项式
  typedef PolynomialND<N_DEG, N_DIM> PolynomialType;

  /// 默认构造函数，生成空的样条曲线
  Spline() {}

  /**
   * @brief 设置样条曲线的定义域划分
   *
   * 定义域向量定义了样条的分段边界。例如，vec_domain = [0, 5, 10, 15]
   * 表示一个包含三个多项式段的三段样条，分别在 [0,5)、[5,10)、[10,15] 上有效。
   * 调用此方法后，poly_ 向量将被调整为 vec_domain.size() - 1 的大小。
   *
   * @param vec_domain 定义域划分向量，必须包含至少 2 个元素，且严格单调递增
   */
  void set_vec_domain(const std::vector<decimal_t>& vec_domain) {
    assert(vec_domain.size() > 1);
    vec_domain_ = vec_domain;
    poly_.resize(vec_domain_.size() - 1);
  }

  /**
   * @brief 获取样条曲线的分段数
   * @return 多项式段的数量（等于定义域断点数减 1）
   */
  int num_segments() const { return static_cast<int>(poly_.size()); }

  /**
   * @brief 获取样条曲线的定义域划分向量
   * @return 包含所有断点的向量（断点数为分段数 + 1）
   */
  std::vector<decimal_t> vec_domain() const { return vec_domain_; }

  /**
   * @brief 获取样条曲线参数域的起点
   * @return 第一个断点的值（即最小有效参数值）
   */
  decimal_t begin() const {
    if (vec_domain_.size() < 1) return 0.0;
    return vec_domain_.front();
  }

  /**
   * @brief 获取样条曲线参数域的终点
   * @return 最后一个断点的值（即最大有效参数值）
   */
  decimal_t end() const {
    if (vec_domain_.size() < 1) return 0.0;
    return vec_domain_.back();
  }

  /**
   * @brief 获取指定段和维度的多项式引用
   *
   * 通过该操作符可以直接访问第 n 段第 j 维的多项式对象，便于
   * 在构造阶段逐段逐维设置多项式系数。
   *
   * @param n 段索引（0-based，必须 < num_segments()）
   * @param j 维度索引（0-based，必须 < N_DIM）
   * @return 第 n 段第 j 维多形式的引用
   */
  Polynomial<N_DEG>& operator()(int n, int j) {
    assert(n < this->num_segments());
    assert(j < N_DIM);
    return poly_[n][j];
  }

  /**
   * @brief 在指定参数处求值（带导数阶数）
   *
   * 对于参数 s：
   *   1. 使用二分查找（lower_bound）确定 s 位于哪个定义域区间
   *   2. 计算局部参数 h = s - vec_domain_[idx]
   *   3. 调用对应段的多项式在 h 处求 d 阶导数
   *
   * 外推策略：当 s 超出定义域时，使用最近端点处的多项式段进行外推。
   *
   * @param s 参数值（局部分段参数的基准点是各自的断点）
   * @param d 求导阶数（0=位置, 1=一阶导数, 2=二阶导数, ...）
   * @param ret 输出参数，N_DIM 维的求值结果向量
   * @return ErrorType 操作是否成功（定义域为空时返回 kIllegalInput）
   */
  ErrorType evaluate(const decimal_t s, int d, Vecf<N_DIM>* ret) const {
    int num_pts = vec_domain_.size();
    if (num_pts < 1) return kIllegalInput;

    // 使用二分查找定位参数 s 所属的定义域区间
    auto it = std::lower_bound(vec_domain_.begin(), vec_domain_.end(), s);
    int idx = std::max(static_cast<int>(it - vec_domain_.begin()) - 1, 0);
    decimal_t h = s - vec_domain_[idx];  // 计算局部参数

    if (s < vec_domain_[0]) {
      // 外推：参数小于起点，使用第一段多项式外推
      poly_[0].evaluate(h, d, ret);
    } else if (s > vec_domain_[num_pts - 1]) {
      // 外推：参数大于终点，使用最后一段多项式外推
      h = s - vec_domain_[idx - 1];
      poly_[num_pts - 2].evaluate(h, d, ret);
    } else {
      // 正常内插：使用对应段的多项式求值
      poly_[idx].evaluate(h, d, ret);
    }
    return kSuccess;
  }

  /**
   * @brief 在指定参数处求值（0 阶导数，即位置值）
   *
   * 与 evaluate(s, 0, ret) 功能相同，但使用更高效的多项式求值路径
   * （Polynomial::evaluate(s) 而非 evaluate(s, 0)）。
   *
   * @param s 参数值
   * @param ret 输出参数，N_DIM 维的位置向量
   * @return ErrorType 操作是否成功
   */
  ErrorType evaluate(const decimal_t s, Vecf<N_DIM>* ret) const {
    int num_pts = vec_domain_.size();
    if (num_pts < 1) return kIllegalInput;

    auto it = std::lower_bound(vec_domain_.begin(), vec_domain_.end(), s);
    int idx = std::max(static_cast<int>(it - vec_domain_.begin()) - 1, 0);
    decimal_t h = s - vec_domain_[idx];

    if (s < vec_domain_[0]) {
      // 外推：参数小于起点
      poly_[0].evaluate(h, ret);
    } else if (s > vec_domain_[num_pts - 1]) {
      // 外推：参数大于终点
      h = s - vec_domain_[idx - 1];
      poly_[num_pts - 2].evaluate(h, ret);
    } else {
      // 正常内插
      poly_[idx].evaluate(h, ret);
    }
    return kSuccess;
  }

  /**
   * @brief 打印样条曲线的详细信息（调试用）
   *
   * 对每个分段输出：
   *   - 定义域区间
   *   - 多项式系数
   *   - 该段终点处的位置、速度、加速度、加加速度值
   */
  void print() const {
    int num_polys = static_cast<int>(poly_.size());
    for (int i = 0; i < num_polys; i++) {
      printf("vec domain (%lf, %lf).\n", vec_domain_[i], vec_domain_[i + 1]);
      poly_[i].print();
      Vecf<2> v;
      poly_[i].evaluate(vec_domain_[i + 1] - vec_domain_[i], 0, &v);
      printf("end pos: (%lf, %lf).\n", v[0], v[1]);
      poly_[i].evaluate(vec_domain_[i + 1] - vec_domain_[i], 1, &v);
      printf("end vel: (%lf, %lf).\n", v[0], v[1]);
      poly_[i].evaluate(vec_domain_[i + 1] - vec_domain_[i], 2, &v);
      printf("end acc: (%lf, %lf).\n", v[0], v[1]);
      poly_[i].evaluate(vec_domain_[i + 1] - vec_domain_[i], 3, &v);
      printf("end jerk: (%lf, %lf).\n", v[0], v[1]);
    }
  }

 private:
  /// 多项式段向量，poly_[i] 在区间 [vec_domain_[i], vec_domain_[i+1]] 上有效
  vec_E<PolynomialType> poly_;

  /// 定义域划分向量，vec_domain_[i] 是第 i 个断点的参数值
  std::vector<decimal_t> vec_domain_;
};

}  // namespace common

#endif
