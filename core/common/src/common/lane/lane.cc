/**
 * @file lane.cc
 * @brief 车道类（Lane）的实现文件
 *
 * 本文件实现了EPSILON自动驾驶规划系统中的核心车道几何类型。
 * Lane类封装了一条车道的参考曲线（以样条曲线表示），支持多种
 * 沿弧长的几何查询操作，是Frenet坐标系变换和路径规划的基础。
 *
 * ==================== Lane类核心功能 ====================
 *
 * 1. 几何查询（沿弧长参数化）：
 *    - GetPositionByArcLength: 获取弧长s处的笛卡尔位置
 *    - GetTangentVectorByArcLength: 获取弧长s处的单位切向量
 *    - GetNormalVectorByArcLength: 获取弧长s处的单位法向量
 *    - GetOrientationByArcLength: 获取弧长s处的朝向角
 *    - GetCurvatureByArcLength: 获取弧长s处的曲率及曲率导数
 *    - GetDerivativeByArcLength: 获取弧长s处各阶导数
 *
 * 2. 逆向查询（笛卡尔 → 弧长投影）：
 *    - GetArcLengthByVecPosition: 将笛卡尔坐标投影到车道参考线上
 *      采用两阶段方法：二分法粗搜索 + 牛顿法精搜索
 *
 * 3. 参数验证：
 *    - CheckInputArcLength: 检查输入的弧长是否在有效定义域内
 *
 * ==================== 投影算法详解 ====================
 *
 * GetArcLengthByVecPosition 两阶段方法：
 *
 * 第一阶段（二分法粗搜索）：
 *   将弧长定义域 [s_min, s_max] 对分为三个点 s1, s2, s3，
 *   比较输入点到三个点对应位置的距离，取距离最小的点，
 *   然后在该点附近缩小搜索区间。最多迭代 kMaxCnt=4 次。
 *   如果最短距离已经小于阈值 kMaxDistSquare=900，则退出循环。
 *
 * 第二阶段（牛顿法精搜索）：
 *   使用第一阶段得到的初始估计 s0，进行牛顿迭代：
 *   目标函数：f(s) = (p(s) - target) dot p'(s)
 *   其中 p(s) 是弧长s处的参考点位置，p'(s) 是切线方向。
 *   牛顿更新：s_{k+1} = s_k - f(s_k) / f'(s_k)
 *   收敛条件：|dx| < epsilon (1e-3) 或超过最大迭代次数 kMaxIter=8
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/lane/lane.h"

namespace common {

/*
 * GetCurvatureByArcLength - 获取弧长s处的曲率和曲率导数
 *
 * 对于平面曲线 r(s) = (x(s), y(s))，曲率计算公式为：
 *   kappa = (x' * y'' - y' * x'') / (x'^2 + y'^2)^(3/2)
 *
 * 曲率导数 dkappa/ds 由链式法则展开计算。
 *
 * @param arc_length 输入的弧长参数s
 * @param curvature 输出参数，曲率 kappa(s)
 * @param curvature_derivative 输出参数，曲率导数 dkappa/ds
 * @return kSuccess 计算成功；kIllegalInput 弧长越界或非二维
 */
ErrorType Lane::GetCurvatureByArcLength(const decimal_t& arc_length,
                                        decimal_t* curvature,
                                        decimal_t* curvature_derivative) const {
  if (CheckInputArcLength(arc_length) != kSuccess || LaneDim != 2) {
    return kIllegalInput;
  }

  // 获取参考曲线在弧长s处的一阶导数（速度向量/切向量）和二阶导数（加速度向量）
  Vecf<LaneDim> vel, acc, jrk;
  GetDerivativeByArcLength(arc_length, 1, &vel);   // 一阶导: (x', y')
  GetDerivativeByArcLength(arc_length, 2, &acc);   // 二阶导: (x'', y'')
  GetDerivativeByArcLength(arc_length, 3, &jrk);   // 三阶导: (x''', y''')

  /*
   * 曲率计算 (使用二维叉积):
   *   kappa = (x' * y'' - y' * x'') / |r'|^3
   */
  decimal_t c0 = vel[0] * acc[1] - vel[1] * acc[0];  // 分子: 叉积 x'*y'' - y'*x''
  decimal_t c1 = vel.norm();                          // 分母: |r'| = sqrt(x'^2 + y'^2)
  *curvature = c0 / (c1 * c1 * c1);

  /*
   * 曲率导数计算（通过商法则求导）:
   *   dkappa/ds = (c0' * c1^3 - 3 * c0 * c1^2 * c1') / c1^6
   */
  *curvature_derivative =
      ((acc[0] * acc[1] + vel[0] * jrk[1] - acc[1] * acc[0] - vel[1] * jrk[0]) /
           c1 * c1 * c1 -
       3 * c0 * (vel[0] * acc[0] + vel[1] * acc[1]) / (c1 * c1 * c1 * c1 * c1));
  return kSuccess;
}

/*
 * GetCurvatureByArcLength - 获取弧长s处的曲率（不计算曲率导数）
 *
 * 这是上面函数的简化版本，仅计算曲率，不计算曲率导数。
 *
 * @param arc_length 输入的弧长参数s
 * @param curvature 输出参数，曲率 kappa(s)
 * @return kSuccess；kIllegalInput 弧长越界或非二维
 */
ErrorType Lane::GetCurvatureByArcLength(const decimal_t& arc_length,
                                        decimal_t* curvature) const {
  if (CheckInputArcLength(arc_length) != kSuccess || LaneDim != 2) {
    return kIllegalInput;
  }

  Vecf<LaneDim> vel, acc;
  GetDerivativeByArcLength(arc_length, 1, &vel);
  GetDerivativeByArcLength(arc_length, 2, &acc);

  decimal_t c0 = vel[0] * acc[1] - vel[1] * acc[0];  // 分子: x'*y'' - y'*x''
  decimal_t c1 = vel.norm();
  *curvature = c0 / (c1 * c1 * c1);
  return kSuccess;
}

/*
 * GetDerivativeByArcLength - 获取弧长s处位置样条的d阶导数
 *
 * 委托给内部的位置样条对象（position_spline_）进行求值。
 *
 * @param arc_length 弧长参数s
 * @param d 导数阶数（0=位置, 1=切线, 2=曲率相关, ...）
 * @param derivative 输出参数，d阶导数向量
 * @return 样条求值的结果状态
 */
ErrorType Lane::GetDerivativeByArcLength(const decimal_t arc_length,
                                         const int d,
                                         Vecf<LaneDim>* derivative) const {
  return position_spline_.evaluate(arc_length, d, derivative);
}

/*
 * GetPositionByArcLength - 获取弧长s处的笛卡尔坐标位置
 *
 * 委托给位置样条的0阶导数求值。
 *
 * @param arc_length 弧长参数s
 * @param derivative 输出参数，笛卡尔位置坐标
 * @return 样条求值的结果状态
 */
ErrorType Lane::GetPositionByArcLength(const decimal_t arc_length,
                                       Vecf<LaneDim>* derivative) const {
  return position_spline_.evaluate(arc_length, derivative);
}

/*
 * GetTangentVectorByArcLength - 获取弧长s处的单位切向量
 *
 * 切向量 = 位置样条的一阶导数归一化结果。
 * 即：t(s) = r'(s) / |r'(s)|
 *
 * @param arc_length 弧长参数s
 * @param tangent_vector 输出参数，单位切向量
 * @return kSuccess；kIllegalInput 弧长越界；kWrongStatus 切线为零向量（异常）
 */
ErrorType Lane::GetTangentVectorByArcLength(
    const decimal_t arc_length, Vecf<LaneDim>* tangent_vector) const {
  if (CheckInputArcLength(arc_length) != kSuccess) {
    return kIllegalInput;
  }

  Vecf<LaneDim> vel;
  GetDerivativeByArcLength(arc_length, 1, &vel);

  if (vel.norm() < kEPS) {
    return kWrongStatus;  // 切线为零向量，无法归一化
  }

  *tangent_vector = vel / vel.norm();
  return kSuccess;
}

/*
 * GetNormalVectorByArcLength - 获取弧长s处的单位法向量
 *
 * 法向量 = 将切向量逆时针旋转90度（即左侧法向量）。
 * 2D中：(t_x, t_y) → (-t_y, t_x)
 *
 * @param arc_length 弧长参数s
 * @param normal_vector 输出参数，单位法向量（指向左侧）
 * @return kSuccess；kIllegalInput 弧长越界；kWrongStatus 切线为零向量
 */
ErrorType Lane::GetNormalVectorByArcLength(const decimal_t arc_length,
                                           Vecf<LaneDim>* normal_vector) const {
  if (CheckInputArcLength(arc_length) != kSuccess) {
    return kIllegalInput;
  }

  Vecf<LaneDim> vel;
  GetDerivativeByArcLength(arc_length, 1, &vel);

  if (vel.norm() < kEPS) {
    return kWrongStatus;
  }

  Vecf<LaneDim> tangent_vector = vel / vel.norm();
  *normal_vector = rotate_vector_2d(tangent_vector, M_PI / 2.0);
  return kSuccess;
}

/*
 * GetOrientationByArcLength - 获取弧长s处的参考线朝向角
 *
 * 朝向角 = atan2(tangent_y, tangent_x)
 *
 * @param arc_length 弧长参数s
 * @param angle 输出参数，朝向角（弧度，范围(-pi, pi]）
 * @return kSuccess；kIllegalInput 弧长越界；kWrongStatus 切线为零向量
 */
ErrorType Lane::GetOrientationByArcLength(const decimal_t arc_length,
                                          decimal_t* angle) const {
  if (CheckInputArcLength(arc_length) != kSuccess) {
    return kIllegalInput;
  }

  Vecf<LaneDim> vel;
  GetDerivativeByArcLength(arc_length, 1, &vel);

  if (vel.norm() < kEPS) {
    return kWrongStatus;
  }

  Vecf<LaneDim> tangent_vector = vel / vel.norm();
  *angle = vec2d_to_angle(tangent_vector);
  return kSuccess;
}

/*
 * GetArcLengthByVecPosition - 将笛卡尔坐标投影到车道参考线上
 *
 * 这是逆向查询的核心函数，通过两阶段方法计算给定笛卡尔坐标对应的弧长。
 *
 * 【算法】两阶段方法：
 *
 * 第一阶段（二分法粗搜索，最多4次迭代）：
 *   在弧长定义域 [s_min, s_max] 中取三个等距点 s1, s2, s3，
 *   计算对应位置与目标位置的距离 d1, d2, d3。
 *   取距离最小的点，将搜索区间缩小到该点附近。
 *   当最短距离 < kMaxDistSquare=900 时提前退出粗搜索。
 *   此阶段的目的是为牛顿法提供一个足够好的初始估计。
 *
 * 第二阶段（牛顿法精搜索）：
 *   使用粗搜索的初始估计，调用 GetArcLengthByVecPositionWithInitialGuess
 *   进行牛顿迭代精确求解。
 *
 * @param vec_position 输入的笛卡尔坐标
 * @param arc_length 输出参数，投影后对应的弧长s
 * @return kSuccess；kWrongStatus 参考线无效
 */
ErrorType Lane::GetArcLengthByVecPosition(const Vecf<LaneDim>& vec_position,
                                          decimal_t* arc_length) const {
  if (!IsValid()) {
    return kWrongStatus;
  }

  static constexpr int kMaxCnt = 4;               // 二分法最大迭代次数
  static constexpr decimal_t kMaxDistSquare = 900.0;  // 距离平方阈值（对应30米距离）

  const decimal_t val_lb = position_spline_.begin();  // 弧长下界
  const decimal_t val_ub = position_spline_.end();    // 弧长上界
  decimal_t step = (val_ub - val_lb) * 0.5;        // 初始步长

  // 定义域中三个均匀分布的点
  decimal_t s1 = val_lb;           // 左端点
  decimal_t s2 = val_lb + step;    // 中点
  decimal_t s3 = val_ub;           // 右端点
  decimal_t initial_guess = s2;    // 默认初始估计为中点

  // 计算三点的位置和到目标位置的距离
  Vecf<LaneDim> start_pos, mid_pos, final_pos;
  position_spline_.evaluate(s1, &start_pos);
  position_spline_.evaluate(s2, &mid_pos);
  position_spline_.evaluate(s3, &final_pos);

  decimal_t d1 = (start_pos - vec_position).squaredNorm();
  decimal_t d2 = (mid_pos - vec_position).squaredNorm();
  decimal_t d3 = (final_pos - vec_position).squaredNorm();

  // 阶段一: 二分法粗搜索
  for (int i = 0; i < kMaxCnt; ++i) {
    decimal_t min_dis = std::min(std::min(d1, d2), d3);
    if (min_dis < kMaxDistSquare) {
      // 距离足够近了，退出粗搜索
      if (min_dis == d1) {
        initial_guess = s1;
      } else if (min_dis == d2) {
        initial_guess = s2;
      } else if (min_dis == d3) {
        initial_guess = s3;
      } else {
        assert(false);
      }
      break;
    }
    step *= 0.5;  // 步长减半
    if (min_dis == d1) {
      // 最近点在左端，向左收缩搜索区间
      initial_guess = s1;
      s3 = s2;
      s2 = s1 + step;
      position_spline_.evaluate(s2, &mid_pos);
      position_spline_.evaluate(s3, &final_pos);
      d2 = (mid_pos - vec_position).squaredNorm();
      d3 = (final_pos - vec_position).squaredNorm();
    } else if (min_dis == d2) {
      // 最近点在中点，向中心收缩
      initial_guess = s2;
      s1 = s2 - step;
      s3 = s2 + step;
      position_spline_.evaluate(s1, &start_pos);
      position_spline_.evaluate(s3, &final_pos);
      d1 = (start_pos - vec_position).squaredNorm();
      d3 = (final_pos - vec_position).squaredNorm();
    } else if (min_dis == d3) {
      // 最近点在右端，向右收缩搜索区间
      initial_guess = s3;
      s1 = s2;
      s2 = s3 - step;
      position_spline_.evaluate(s1, &start_pos);
      position_spline_.evaluate(s2, &mid_pos);
      d1 = (start_pos - vec_position).squaredNorm();
      d2 = (mid_pos - vec_position).squaredNorm();
    } else {
      printf(
          "[Lane]GetArcLengthByVecPosition - d1: %lf, d2: %lf, d3: %lf, "
          "min_dis: %lf\n",
          d1, d2, d3, min_dis);
      assert(false);
    }
  }

  // 阶段二: 牛顿法精搜索
  GetArcLengthByVecPositionWithInitialGuess(vec_position, initial_guess,
                                            arc_length);

  return kSuccess;
}

/*
 * GetArcLengthByVecPositionWithInitialGuess - 使用初始估计进行牛顿法精搜索
 *
 * 目标：找到弧长s使得到目标位置的距离最小。
 * 即最小化 d(s) = |p(s) - target| 关于s的值。
 *
 * 牛顿法求解：
 *   目标函数: f(s) = (p(s) - target) dot p'(s) = 0  （距离的梯度为零）
 *   导数: f'(s) = p'(s) dot p'(s) + (p(s) - target) dot p''(s)
 *   更新: s_{k+1} = s_k - f(s_k) / f'(s_k)
 *
 * 收敛条件：|ds| < epsilon (1e-3) 或超过最大迭代次数 kMaxIter=8
 *
 * @param vec_position 目标位置的笛卡尔坐标
 * @param initial_guess 初始弧长估计s0
 * @param arc_length 输出参数，精搜索后的弧长s
 * @return kSuccess；kWrongStatus 参考线无效
 */
ErrorType Lane::GetArcLengthByVecPositionWithInitialGuess(
    const Vecf<LaneDim>& vec_position, const decimal_t& initial_guess,
    decimal_t* arc_length) const {
  if (!IsValid()) {
    return kWrongStatus;
  }

  const decimal_t val_lb = position_spline_.begin();
  const decimal_t val_ub = position_spline_.end();

  static constexpr decimal_t epsilon = 1e-3;  // 收敛阈值
  static constexpr int kMaxIter = 8;           // 最大迭代次数
  decimal_t x = std::min(std::max(initial_guess, val_lb), val_ub);  // 钳位到定义域
  Vecf<LaneDim> p, dp, ddp, tmp_vec;

  for (int i = 0; i < kMaxIter; ++i) {
    // 获取当前位置及其一阶、二阶导数
    position_spline_.evaluate(x, 0, &p);    // 位置 p(s)
    position_spline_.evaluate(x, 1, &dp);   // 切线 p'(s)
    position_spline_.evaluate(x, 2, &ddp);  // 二阶导数 p''(s)

    // f(s) = (p(s) - target) dot p'(s)
    tmp_vec = p - vec_position;
    double f_1 = tmp_vec.dot(dp);
    // f'(s) = p'(s) dot p'(s) + (p(s) - target) dot p''(s)
    double f_2 = dp.dot(dp) + tmp_vec.dot(ddp);
    double dx = -f_1 / f_2;  // 牛顿更新步长

    if (std::fabs(dx) < epsilon) {
      break;  // 已收敛
    }

    if (x + dx > val_ub) {
      x = val_ub;  // 超出上界，钳位到上界
      break;
    } else if (x + dx < val_lb) {
      x = val_lb;  // 超出下界，钳位到下界
      break;
    }

    x += dx;  // 正常更新
  }

  *arc_length = x;

  return kSuccess;
}

/*
 * CheckInputArcLength - 检查输入的弧长是否在有效范围内
 *
 * 验证弧长值在位置样条的定义域 [begin, end] 内。
 * 使用 kEPS 容差允许微小的超出。
 *
 * @param arc_length 待检查的弧长值
 * @return kSuccess 有效；kWrongStatus 参考线无效；kIllegalInput 弧长越界
 */
ErrorType Lane::CheckInputArcLength(const decimal_t arc_length) const {
  if (!IsValid()) {
    printf("[CheckInputArcLength]Quering invalid lane.\n");
    return kWrongStatus;
  }
  if (arc_length < position_spline_.begin() - kEPS ||
      arc_length > position_spline_.end() + kEPS) {
    return kIllegalInput;
  }
  return kSuccess;
}

}  // namespace common
