/**
 * @file polynomial.h
 * @brief 一维多项式和 N 维多项式类定义
 *
 * 本文件定义了 EPSILON 自动驾驶规划系统中的核心数学工具——多项式类。
 * 包含两个模板类：
 *   - Polynomial<N_DEG>: 一维多项式，次数为 N_DEG
 *   - PolynomialND<N_DEG, N_DIM>: N 维多项式（即 N_DIM 个独立的一维多项式的组合）
 *
 * 多项式的系数存储策略（两种顺序）：
 *   1. coeff_（反向/物理顺序）:
 *      f(s) = coeff_(n) + coeff_(n-1)/1! * s + ... + coeff_(0)/n! * s^n
 *      这种顺序的系数具有直观的物理意义：
 *        - coeff_(n) = f(0)        （位置）
 *        - coeff_(n-1) = f'(0)     （速度）
 *        - coeff_(n-2) = f''(0)    （加速度）
 *      因此这种顺序便于直接设置和读取边界条件。
 *
 *   2. coeff_normal_order_（正向/数学顺序）:
 *      f(s) = coeff_normal_order_(0) + coeff_normal_order_(1) * s + ... + coeff_normal_order_(n) * s^n
 *      这种顺序与标准数学表示一致，且在使用 Horner 法则求值时效率更高
 *      （无需在循环内做除法）。
 *
 * 系统维护两种表示的一致性，任何系数的修改都会通过 update() 函数
 * 自动更新另一个表示。
 *
 * 核心功能：
 *   - evaluate(s): 在参数 s 处求值（使用 Horner 法则，高效）
 *   - evaluate(s, d): 在参数 s 处求 d 阶导数
 *   - J(s, d): 计算 d 阶导数的平方积分能量项（用于轨迹优化）
 *   - GetJerkOptimalConnection(...): 生成 jerk 最优（五次多项式）连接曲线
 *
 * 该类是 Spline 和 Lane 等上层类的基础数学组件。
 */

#ifndef _CORE_COMMON_INC_COMMON_SPLINE_POLYNOMIAL_H__
#define _CORE_COMMON_INC_COMMON_SPLINE_POLYNOMIAL_H__

#include "common/basics/basics.h"
#include "common/math/calculations.h"
#include "common/spline/lookup_table.h"

#include <assert.h>

namespace common {

/**
 * @class Polynomial
 * @brief 一维多项式类
 *
 * 该类表示一个 N_DEG 次的一维多项式，支持高效求值（含导数）、
 * 能量积分计算和 jerk 最优曲线生成。
 *
 * 系数以反向顺序存储（coeff_），使得低索引对应高阶导数，
 * 具有直观的物理含义。更新时自动同步正向顺序表示。
 *
 * @tparam N_DEG 多项式的次数（如 5 表示五次多项式）
 *
 * 参数化：
 *   f(s) = coeff_(n) + coeff_(n-1)/1! * s + ... + coeff_(0)/n! * s^n
 *
 * 例（五次多项式 N_DEG=5）：
 *   coeff_ = (5!*a5, 4!*a4, 3!*a3, 2!*a2, a1, a0)
 *   其中 a0=f(0), a1=f'(0), a2=f''(0), ...
 */
template <int N_DEG>
class Polynomial {
 public:
  /// N_DEG+1 维系数向量的类型别名
  typedef Vecf<N_DEG + 1> VecNf;

  /// 编译期判断是否需要 Eigen 内存对齐
  enum { NeedsToAlign = (sizeof(VecNf) % 16) == 0 };

  /// 默认构造函数，初始化为零多项式
  Polynomial() { set_zero(); }

  /**
   * @brief 通过系数向量构造多项式
   * @param coeff 反向顺序的系数向量（见类文档中的参数化说明）
   */
  Polynomial(const VecNf& coeff) : coeff_(coeff) { update(); }

  /**
   * @brief 获取多项式的系数向量（反向顺序）
   * @return N_DEG+1 维系数向量，coeff_(i) / (N_DEG-i)! 是 i 阶导数值
   */
  VecNf coeff() const { return coeff_; }

  /**
   * @brief 设置多项式的系数
   * @param coeff 反向顺序的系数向量，设置后自动更新正向顺序表示
   */
  void set_coeff(const VecNf& coeff) {
    coeff_ = coeff;
    update();
  }

  /**
   * @brief 更新正向顺序的系数表示
   *
   * 将反向顺序系数 coeff_ 转换为正向顺序系数 coeff_normal_order_：
   *   coeff_normal_order_[i] = coeff_[N_DEG - i] / i!
   *
   * 转换公式源自泰勒展开中阶乘因子的除法。
   */
  void update() {
    for (int i = 0; i < N_DEG + 1; i++) {
      coeff_normal_order_[i] = coeff_[N_DEG - i] / fac(i);
    }
  }

  /**
   * @brief 将多项式系数清零（设为零多项式）
   */
  void set_zero() {
    coeff_.setZero();
    coeff_normal_order_.setZero();
  }

  /**
   * @brief 在参数 s 处求 d 阶导数的值
   *
   * 使用 Horner 法则（秦九韶算法）进行高效求值，复杂度 O(N_DEG-d)。
   * 算法：p = coeff_(0)/fac(N_DEG-d)，然后迭代：
   *   p = p * s + coeff_(i) / fac(N_DEG - i - d)
   *
   * 注意：此路径使用反向系数 coeff_，在循环内需要除以阶乘。
   * evaluate(s) 无导数版本由于使用正向系数而无需除法，效率更高。
   *
   * @param s 参数值（自变量）
   * @param d 导数阶数（0=函数值, 1=一阶导数, ...）
   * @return 多项式在 s 处的 d 阶导数值
   */
  inline decimal_t evaluate(const decimal_t& s, const int& d) const {
    // 使用 Horner 法则（秦九韶算法）高效求值
    decimal_t p = coeff_(0) / fac(N_DEG - d);
    for (int i = 1; i <= N_DEG - d; i++) {
      p = (p * s + coeff_(i) / fac(N_DEG - i - d));
    }
    return p;
    // 经测试，以上算法通常比使用正向系数 + 阶乘乘法的路径更快
    // （d=1 时尤为明显，d 增大后优势减弱）
  }

  /**
   * @brief 在参数 s 处求函数值（0 阶导数）
   *
   * 此版本使用正向系数 coeff_normal_order_ 和 Horner 法则，
   * 无需在循环内做除法，因此比 evaluate(s, 0) 更快。
   *
   * @param s 参数值
   * @return 多项式在 s 处的函数值
   */
  inline decimal_t evaluate(const decimal_t& s) const {
    // 使用 Horner 法则高效求值（无需除法，比 evaluate(s, 0) 更快）
    decimal_t p = coeff_normal_order_[N_DEG];
    for (int i = 1; i <= N_DEG; i++) {
      p = (p * s + coeff_normal_order_[N_DEG - i]);
    }
    return p;
  }

  /**
   * @brief 计算 d 阶导数的平方积分（能量项）
   *
   * 在轨迹优化中，常用 jerk 的平方积分（d=3）或加速度的平方积分（d=2）
   * 作为光滑性度量。该函数返回从 0 到 s 的定积分值。
   *
   * 数学定义：J(s, d) = integral_{0}^{s} [f^(d)(tau)]^2 dtau
   *
   * 对于 d=3 (jerk 能量)：闭合形式是 s 的五次多项式
   *   积分项 = coeff_(0)^2 * s^5 / 20 + coeff_(0)*coeff_(1) * s^4 / 4
   *          + (coeff_(1)^2 + coeff_(0)*coeff_(2)) * s^3 / 3
   *          + coeff_(1)*coeff_(2) * s^2 + coeff_(2)^2 * s
   *
   * 对于 d=2 (加速度能量)：闭合形式是 s 的七次多项式
   *
   * @param s 积分上限（积分下限始终为 0）
   * @param d 导数阶数（2=加速度, 3=jerk）
   * @return 从 0 到 s 的 d 阶导数平方积分值
   */
  inline decimal_t J(decimal_t s, int d) const {
    if (d == 3) {
      // jerk 平方的积分（闭合解，s 的五次多项式）
      return coeff_(0) * coeff_(0) / 20.0 * pow(s, 5) +
             coeff_(0) * coeff_(1) / 4 * pow(s, 4) +
             (coeff_(1) * coeff_(1) + coeff_(0) * coeff_(2)) / 3 * pow(s, 3) +
             coeff_(1) * coeff_(2) * s * s + coeff_(2) * coeff_(2) * s;
    } else if (d == 2) {
      // 加速度平方的积分（闭合解，s 的七次多项式）
      return coeff_(0) * coeff_(0) / 252 * pow(s, 7) +
             coeff_(0) * coeff_(1) / 36 * pow(s, 6) +
             (coeff_(1) * coeff_(1) / 20 + coeff_(0) * coeff_(2) / 15) *
                 pow(s, 5) +
             (coeff_(0) * coeff_(3) / 12 + coeff_(1) * coeff_(2) / 4) *
                 pow(s, 4) +
             (coeff_(2) * coeff_(2) / 3 + coeff_(1) * coeff_(3) / 3) *
                 pow(s, 3) +
             coeff_(2) * coeff_(3) * s * s + coeff_(3) * coeff_(3) * s;
    } else {
      assert(false);  // 仅支持 d=2 和 d=3
    }
    return 0.0;
  }

  /**
   * @brief 生成满足起始和终止边界条件的 jerk 最优（minimum jerk）连接曲线
   *
   * 该函数在给定起点和终点位置、速度、加速度以及持续时间 S 的条件下，
   * 求解一条使得 jerk 平方积分最小的五次多项式。
   *
   * 求解过程：
   *   1. 构建边界条件向量 b = [p1, dp1, ddp1, p2, dp2, ddp2]
   *   2. 通过线性系统 A * c = b 求解多项式系数（归一化后的系数）
   *   3. 使用查表（LookUpCache）或实时计算（GetAInverse）来获取 A 矩阵的逆
   *
   * 注意：当 S < kEPS 时（接近零），处理静止轨迹的奇异性，
   *       此时直接设置常数/零系数。
   *
   * @param p1 起点位置值
   * @param dp1 起点一阶导数值（速度）
   * @param ddp1 起点二阶导数值（加速度）
   * @param p2 终点位置值
   * @param dp2 终点一阶导数值（速度）
   * @param ddp2 终点二阶导数值（加速度）
   * @param S 持续时间（> 0）
   *
   * @pre N_DEG >= 5（多项式必须至少为五次以提供六个自由度）
   */
  void GetJerkOptimalConnection(const decimal_t p1, const decimal_t dp1,
                                const decimal_t ddp1, const decimal_t p2,
                                const decimal_t dp2, const decimal_t ddp2,
                                const decimal_t S) {
    assert(N_DEG >= 5);

    // 边界条件向量
    Vecf<6> b;
    b << p1, dp1, ddp1, p2, dp2, ddp2;

    coeff_.setZero();

    // 处理 S=0（静止轨迹）的奇异性
    if (S < kEPS) {
      Vecf<6> c;
      c << 0.0, 0.0, 0.0, ddp1, dp1, p1;  // 常数加速度、常数速度分段
      coeff_.template segment<6>(N_DEG - 5) = c;
      return;
    }

    // 尝试从查表中获取预计算的 A 矩阵逆
    MatNf<6> A_inverse;
    if (!LookUpCache(S, &A_inverse)) {
      A_inverse = GetAInverse(S);  // 查表未命中，实时计算
    }

    // 求解系数 coeff = A_inverse * b
    auto coeff = A_inverse * b;

    // 将归一化系数转换为反向顺序的物理系数
    // coeff_(N_DEG-5) = c(0) * 5! 对应五阶项
    // coeff_(N_DEG-4) = c(1) * 4! 对应四阶项
    // ...
    // coeff_(N_DEG-0) = c(5)      对应零阶项
    coeff_[N_DEG - 5] = coeff(0) * fac(5);
    coeff_[N_DEG - 4] = coeff(1) * fac(4);
    coeff_[N_DEG - 3] = coeff(2) * fac(3);
    coeff_[N_DEG - 2] = coeff(3) * fac(2);
    coeff_[N_DEG - 1] = coeff(4) * fac(1);
    coeff_[N_DEG - 0] = coeff(5);

    update();  // 同步更新正向顺序系数
  }

  /**
   * @brief 从预计算的查表中获取 A 逆矩阵
   *
   * 由于 A 逆矩阵只依赖于时间 S，对于离散的 S 值可以预先计算并缓存，
   * 避免重复求解大型线性系统，显著提升轨迹优化效率。
   *
   * @param S 时间长度
   * @param A_inverse 输出参数，若查表命中则包含 A 矩阵的逆
   * @return true 查表命中并成功获取，false 查表未命中
   */
  bool LookUpCache(const decimal_t S, MatNf<6>* A_inverse) {
    auto it = kTableAInverse.find(S);
    if (it != kTableAInverse.end()) {
      *A_inverse = it->second;
      return true;
    } else {
      return false;
    }
  }

  /**
   * @brief 打印多项式系数（调试用）
   *
   * 以固定小数点格式、7 位精度输出反向顺序的系数向量。
   */
  void print() const {
    std::cout << std::fixed << std::setprecision(7) << coeff_.transpose()
              << std::endl;
  }

 private:
  /// 反向顺序系数向量：coeff_(0) 对应最高阶项（缩放后），coeff_(N_DEG) 对应零阶项
  VecNf coeff_;

  /// 正向顺序系数向量（标准多项式表示），由 coeff_ 通过 update() 自动维护
  VecNf coeff_normal_order_;

  /// Eigen 内存对齐（仅在 NeedsToAlign 为 true 时启用）
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
};

/**
 * @class PolynomialND
 * @brief N 维多项式类（N_DIM 个独立的一维多项式的组合）
 *
 * 该类将 N_DIM 个具有相同次数 N_DEG 的一维多项式组合在一起，
 * 用于表示多维空间中的参数化曲线。例如，N_DIM=2 时表示二维
 * 平面曲线 (x(s), y(s))。
 *
 * @tparam N_DEG 每个分量多项式的次数
 * @tparam N_DIM 空间维度（分量数）
 */
template <int N_DEG, int N_DIM>
class PolynomialND {
 public:
  /// 默认构造函数，所有分量多项式初始化为零
  PolynomialND() {}

  /**
   * @brief 通过多项式数组构造 N 维多项式
   * @param polys 长度为 N_DIM 的多项式数组，polys[i] 是第 i 维的分量
   */
  PolynomialND(const std::array<Polynomial<N_DEG>, N_DIM>& polys)
      : polys_(polys) {}

  /**
   * @brief 获取第 j 维分量多项式的引用
   * @param j 维度索引（0 <= j < N_DIM），例如 j=0 对应 x 分量
   * @return 第 j 维的一维多项式引用
   */
  Polynomial<N_DEG>& operator[](int j) {
    assert(j < N_DIM);
    return polys_[j];
  }

  /**
   * @brief 在参数 s 处求 d 阶导数的值（N_DIM 维向量输出）
   *
   * 逐维调用 Polynomial::evaluate(s, d) 并组装为向量。
   *
   * @param s 参数值
   * @param d 导数阶数
   * @param vec 输出参数，N_DIM 维导数向量
   */
  inline void evaluate(const decimal_t s, int d, Vecf<N_DIM>* vec) const {
    for (int i = 0; i < N_DIM; i++) {
      (*vec)[i] = polys_[i].evaluate(s, d);
    }
  }

  /**
   * @brief 在参数 s 处求函数值（零阶导数）
   *
   * 使用 Polynomial::evaluate(s) 的无导数高效路径。
   *
   * @param s 参数值
   * @param vec 输出参数，N_DIM 维位置向量
   */
  inline void evaluate(const decimal_t s, Vecf<N_DIM>* vec) const {
    for (int i = 0; i < N_DIM; i++) {
      (*vec)[i] = polys_[i].evaluate(s);
    }
  }

  /**
   * @brief 打印 N 维多项式的所有分量系数（调试用）
   */
  void print() const {
    for (int i = 0; i < N_DIM; i++) polys_[i].print();
  }

 private:
  /// N_DIM 个一维多项式的数组
  std::array<Polynomial<N_DEG>, N_DIM> polys_;
};

}  // namespace common

#endif
