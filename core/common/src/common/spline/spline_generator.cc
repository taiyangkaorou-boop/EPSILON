/**
 * @file spline_generator.cc
 * @brief 样条曲线生成器（SplineGenerator）的实现文件
 *
 * 本文件实现了EPSILON自动驾驶规划系统中样条曲线生成的核心算法。
 * SplineGenerator 是一个模板类，支持任意阶数和维度的样条曲线生成。
 *
 * ==================== 核心功能 ====================
 *
 * 1. GetCubicSplineBySampleInterpolation - 三次样条精确插值
 *    对输入采样点进行C2连续的三次样条插值，自动对稀疏采样区间
 *    进行加密以保证后续逆向查询的精度。
 *
 * 2. GetQuinticSplineBySampleFitting - 五次样条平滑拟合
 *    使用分段五次多项式对采样点进行带正则化的加权最小二乘拟合，
 *    通过QP求解器保证各分段之间在位置、速度、加速度、加加速度
 *    (snap/jerk)上的连续性。
 *
 * 3. GetSplineFromStateVec / GetSplineFromFreeStateVec - 从状态向量生成样条
 *    使用加加速度最优连接（Jerk-Optimal Connection），
 *    在相邻状态点之间生成最小加加速度变化的多项式曲线段。
 *
 * 4. GetBezierSplineUsingCorridor - 基于时空语义走廊的贝塞尔样条生成
 *    在给定的时空约束立方体中优化生成贝塞尔曲线，
 *    保证在位置、速度、加速度上的界约束，并最小化高阶导数能量。
 *
 * ==================== 五次样条拟合的数学原理 ====================
 *
 * 目标: min ||S^(1/2) * (A * x - b)||^2 + ||W^(1/2) * x||^2
 * 约束: C * x = c
 *
 * 展开为: min x' * (A' * S * A + W) * x - 2 * x' * A' * S * b + b' * S * b
 * 等价于: min 0.5 * x' * Q * x + c' * x
 * 其中 Q = A' * S * A + W,  c = -A' * S * b
 *
 * 这被转化为标准二次规划问题并通过 OOQP 求解器求解。
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/spline/spline_generator.h"

#include "common/solver/ooqp_interface.h"

namespace common {

/*
 * GetCubicSplineBySampleInterpolation - 通过三次样条插值生成样条
 *
 * 算法流程：
 * 1. 检查输入合法性（至少3个采样点，参数数量匹配，阶数>=3）
 * 2. 稀疏参数加密：如果相邻采样点之间的弧长间距超过阈值(7.0m)，
 *    则在中间插入线性插值点以保证后续弧长逆向查询的精度
 * 3. 对每个维度独立使用 tk::spline 进行三次样条插值
 * 4. 将插值得到的多项式系数填入 Spline 对象
 *
 * 系数转换说明：
 *   tk::spline 返回标准多项式系数（泰勒级数形式）
 *   Spline 类使用除以阶乘的形式（方便直接求导）
 *   因此需要乘以相应的阶乘 coeff[N_DEG - d] = cubic_fitting.get_coeff(n, d) * fac(d)
 *
 * @param samples 采样点序列（每个点为N_DIM维向量）
 * @param para 对应的弧长参数序列
 * @param spline 输出参数，生成的三次样条
 * @return kSuccess；kWrongStatus 输入不合法；kIllegalInput 参数不匹配
 */
template <int N_DEG, int N_DIM>
ErrorType SplineGenerator<N_DEG, N_DIM>::GetCubicSplineBySampleInterpolation(
    const vec_Vecf<N_DIM>& samples, const std::vector<decimal_t>& para,
    SplineType* spline) {
  if (samples.size() < 3) {
    return kWrongStatus;
  }
  if (samples.size() != para.size()) return kIllegalInput;
  if (N_DEG < 3) {
    printf(
        "[CubicSplineBySampleInterpolation]Currently we just support "
        "interpolation >= cubic(3).\n");
    return kWrongStatus;
  }

  SplineType cubic_spline;
  /*
   * 稀疏参数加密：
   * GetArcLengthByVecPosition使用快速实现，
   * 但可能受局部最小值问题影响。
   * 为减少局部最小值风险，需要足够密的参数化。
   * 建议 delta_para < 7.0m
   */
  const decimal_t para_delta_threshold = 7.0;
  int num_samples = static_cast<int>(samples.size());
  vec_Vecf<N_DIM> interpolated_samples{samples[0]};
  std::vector<decimal_t> interpolated_para{para[0]};

  for (int i = 1; i < num_samples; i++) {
    decimal_t dis = para[i] - para[i - 1];
    if (dis > para_delta_threshold) {
      // 间距过大，在中间插入线性插值点
      int n_inserted = static_cast<int>(dis / para_delta_threshold) + 1;
      decimal_t delta_para = dis / n_inserted;
      for (int n = 0; n < n_inserted; n++) {
        decimal_t para_insert = para[i - 1] + (n + 1) * delta_para;
        decimal_t partion = (n + 1) * delta_para / dis;  // 插值比例 [0, 1]
        if (para_insert < para[i]) {
          interpolated_para.push_back(para_insert);
          Vecf<N_DIM> sample_insert;
          // 线性插值：在相邻采样点之间按比例插入
          for (int d = 0; d < N_DIM; d++) {
            sample_insert[d] = (samples[i][d] - samples[i - 1][d]) * partion +
                               samples[i - 1][d];
          }
          interpolated_samples.push_back(sample_insert);
        }
      }
    }
    interpolated_para.push_back(para[i]);
    interpolated_samples.push_back(samples[i]);
  }

  // 设置样条的定义域
  cubic_spline.set_vec_domain(interpolated_para);
  // 对每个维度独立进行三次样条插值
  for (int i = 0; i < N_DIM; i++) {
    std::vector<decimal_t> X, Y;
    for (int s = 0; s < static_cast<int>(interpolated_samples.size()); s++) {
      X.push_back(interpolated_para[s]);
      Y.push_back(interpolated_samples[s][i]);
    }

    // 使用第三方库 tk::spline 进行三次样条插值
    tk::spline cubic_fitting;
    cubic_fitting.set_points(X, Y);

    // 将插值系数填入 Spline 对象
    for (int n = 0; n < cubic_fitting.num_pts() - 1; n++) {
      Vecf<N_DEG + 1> coeff = Vecf<N_DEG + 1>::Zero();
      for (int d = 0; d <= N_DEG; d++) {
        if (d <= 3) {
          // 泰勒系数 → 阶乘系数：乘以 fac(d)
          coeff[N_DEG - d] = cubic_fitting.get_coeff(n, d) * fac(d);
        } else {
          coeff[N_DEG - d] = 0.0;  // 高阶项系数为0（因为是三次样条）
        }
      }
      cubic_spline(n, i).set_coeff(coeff);
    }
  }
  (*spline) = std::move(cubic_spline);
  return kSuccess;
}

/*
 * GetQuinticSplineBySampleFitting - 通过五次样条拟合生成样条
 *
 * 使用分段五次多项式对采样点进行带正则化的加权最小二乘拟合。
 *
 * 【问题表述】求解以下二次规划问题：
 *
 *   min  || S^(1/2) * (A * x - b) ||^2 + || W^(1/2) * x ||^2
 *   s.t. C * x = c
 *
 * 其中：
 *   - x: 待优化的多项式系数向量（所有分段所有维度的系数拼接为一个向量）
 *   - A: 多项式求值矩阵（A * x 给出所有采样点处的拟合值）
 *   - b: 采样点的真实值
 *   - S: 采样点权重矩阵（当前为单位矩阵 = 所有点等权重）
 *   - W: 正则化权重矩阵（对高阶项系数施加惩罚，使曲线更光滑）
 *   - C * x = c: 分段间的连续性约束（位置、速度、加速度、加加速度连续）
 *
 * 【正则化策略】
 *   仅对前3阶导数（位置、速度、加速度）的系数施加正则化，
 *   权重逐阶递减 (regulator / 10^j)，使得对高阶变化的惩罚更重。
 *
 * @param samples 采样点序列
 * @param para 弧长参数序列
 * @param breaks 分段断点（定义每个五次多项式的区间）
 * @param regulator 正则化系数（越大越光滑，越小越拟合）
 * @param spline 输出参数，生成的五次样条
 * @return kSuccess 拟合成功；kWrongStatus 输入不合法或求解失败
 */
template <int N_DEG, int N_DIM>
ErrorType SplineGenerator<N_DEG, N_DIM>::GetQuinticSplineBySampleFitting(
    const vec_Vecf<N_DIM>& samples, const std::vector<decimal_t>& para,
    const Eigen::ArrayXf& breaks, const decimal_t regulator,
    SplineType* spline) {
  if (N_DEG < 5) {
    printf("[QuinticSpline]Degree not support.\n");
    return kWrongStatus;
  }
  if (breaks.size() < 2) {
    printf("[QuinticSpline]Cannot support less than two segment.\n");
    return kWrongStatus;
  }
  if (samples.size() < 3) {
    return kWrongStatus;
  }

  int num_segments = breaks.size() - 1;      // 分段数量
  int num_samples = static_cast<int>(samples.size());  // 采样点总数
  int num_order = N_DEG + 1;                 // 每个分段多项式的系数数量 (=6)

  /*
   * 重新组织采样点并计算每个分段内的相对时间
   * 将全局弧长参数转化为每个分段内的局部时间 t ∈ [0, duration]
   */
  std::vector<std::vector<double>> all_samples;
  std::vector<decimal_t> durations;
  std::vector<int> num_samples_hist{0};
  {
    int idx_low = 0;
    for (int n = 0; n < num_segments; n++) {
      std::vector<double> sub_samples;
      auto lbd = breaks[n];
      auto ubd = breaks[n + 1];
      // 找到属于当前分段的所有采样点
      auto upper_it = std::upper_bound(para.begin(), para.end(), ubd);
      int idx_up = static_cast<int>(upper_it - para.begin());
      for (int i = idx_low; i < idx_up; i++) {
        sub_samples.push_back(para[i] - lbd);  // 转换为相对弧长
      }
      all_samples.push_back(sub_samples);
      durations.push_back(ubd - lbd);  // 分段长度
      num_samples_hist.push_back(idx_up);  // 累积采样点数（用于索引计算）
      idx_low = idx_up;
    }
  }

  // ========== 步骤1: 构造稀疏矩阵 A（求值矩阵）==========
  // A[size: N_DIM*num_samples x N_DIM*num_segments*num_order]
  // A(i, j) = t^(k) / k!  — 即多项式在t处求值的基函数值
  Eigen::SparseMatrix<double, Eigen::RowMajor> A(
      N_DIM * num_samples, N_DIM * num_segments * num_order);
  A.reserve(Eigen::VectorXi::Constant(N_DIM * num_samples, num_order));
  {
    int idx, idy;
    double val;
    for (int n = 0; n < num_segments; n++) {
      int num_sub_samples = static_cast<int>(all_samples[n].size());
      for (int i = 0; i < num_sub_samples; i++) {
        auto deltat = all_samples[n][i];  // 分段内的相对弧长
        for (int j = 0; j < num_order; j++) {
          // 基函数值: t^(N_DEG-j) / (N_DEG-j)!
          val = pow(deltat, num_order - j - 1) / fac(num_order - j - 1);
          for (int d = 0; d < N_DIM; d++) {
            idx = d * num_samples + num_samples_hist[n] + i;
            idy = d * num_segments * num_order + n * num_order + j;
            A.insert(idx, idy) = val;
          }
        }
      }
    }
  }

  // ========== 步骤2: 构造向量 b（采样点的真实值）==========
  // b 按照 (d0_samples, d1_samples, ...) 的顺序排列
  Eigen::VectorXd b;
  b.resize(N_DIM * num_samples);
  {
    for (int i = 0; i < num_samples; i++) {
      for (int d = 0; d < N_DIM; d++) {
        b(d * num_samples + i) = samples[i][d];
      }
    }
  }

  // ========== 步骤3: 构造权重矩阵 S（采样点权重）==========
  // 当前所有点等权重（单位矩阵）
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> S(N_DIM * num_samples);
  S.diagonal() = Eigen::VectorXd::Ones(N_DIM * num_samples);

  // ========== 步骤4: 构造正则化矩阵 W ==========
  // 对前 regulate_top_degree=3 阶的系数施加正则化
  // 权重逐阶递减: regulator / 10^j
  // 这鼓励使用低阶项来表示曲线，避免高阶分量过大
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> W(N_DIM * num_segments *
                                                  num_order);
  Eigen::VectorXd weight(N_DIM * num_segments * num_order);
  weight.setZero();
  int regulate_top_degree = 3;
  for (int n = 0; n < num_segments; n++) {
    for (int j = 0; j < num_order; j++) {
      for (int d = 0; d < N_DIM; d++) {
        if (j < regulate_top_degree)
          weight(d * num_segments * num_order + n * num_order + j) =
              regulator / pow(10, j);
      }
    }
  }
  W.diagonal() = weight;

  // ========== 步骤5: 构造连续性约束矩阵 C ==========
  // C * x = c，其中 c = 0，确保各分段在连接处满足：
  //   位置连续 (c=0)
  //   速度连续 (c=1)  — 一阶导数连续
  //   加速度连续 (c=2)  — 二阶导数连续
  //   加加速度连续 (c=3)  — 三阶导数连续（snap/jerk）
  int num_connections = num_segments - 1;
  int num_continuity = 4;
  Eigen::SparseMatrix<double, Eigen::RowMajor> C(
      N_DIM * num_connections * num_continuity,
      N_DIM * num_segments * num_order);
  C.reserve(Eigen::VectorXi::Constant(N_DIM * num_connections * num_continuity,
                                      2 * num_order));
  {
    int idx, idy_l, idy_r;
    double val_l, val_r;
    for (int n = 0; n < num_connections; n++) {
      for (int c = 0; c < num_continuity; c++) {
        for (int j = 0; j < num_order; j++) {
          if (j <= num_order - 1 - c) {
            auto T = durations[n];
            // 左分段在T处的c阶导数 - 右分段在0处的c阶导数 = 0
            val_l = pow(T, num_order - j - 1 - c) / fac(num_order - j - 1 - c);
            val_r =
                -pow(0.0, num_order - j - 1 - c) / fac(num_order - j - 1 - c);
            for (int d = 0; d < N_DIM; d++) {
              idx =
                  d * num_connections * num_continuity + n * num_continuity + c;
              idy_l = d * num_segments * num_order + n * num_order + j;
              idy_r = d * num_segments * num_order + (n + 1) * num_order + j;
              C.insert(idx, idy_l) = val_l;
              C.insert(idx, idy_r) = val_r;
            }
          }
        }
      }
    }
  }

  // 连续性约束的右端向量c = 0（所有导数差为0）
  Eigen::VectorXd c;
  c.resize(N_DIM * num_connections * num_continuity);
  c.setZero();

  // ========== 步骤6: 求解QP问题 ==========
  Eigen::VectorXd x(N_DIM * num_segments * num_order);
  if (QuadraticProblem::solve(A, S, b, W, C, c, x)) {
    SplineType quintic_spline;
    std::vector<double> vec_domain;
    for (int i = 0; i < breaks.size(); i++) vec_domain.push_back(breaks[i]);
    quintic_spline.set_vec_domain(vec_domain);
    // 将求解得到的系数填入样条对象
    for (int d = 0; d < N_DIM; d++) {
      for (int n = 0; n < num_segments; n++) {
        Vecf<N_DEG + 1> coeff = Vecf<N_DEG + 1>::Zero();
        for (int i = 0; i <= N_DEG; i++) {
          coeff(i) = x[d * num_segments * num_order + n * num_order + i];
        }
        quintic_spline(n, d).set_coeff(coeff);
      }
    }
    (*spline) = std::move(quintic_spline);
  } else {
    printf("solver failed.\n");
    return kWrongStatus;
  }

  return kSuccess;
}

/*
 * GetWaypointsFromPositionSamples - 从位置采样点生成路点向量
 *
 * 为每个采样点创建一个 Waypoint 对象，标记为固定位置约束。
 *
 * @param samples 采样点序列
 * @param para 弧长参数序列
 * @param waypoints 输出参数，生成的路点向量
 * @return kSuccess；kIllegalInput 参数不匹配
 */
template <int N_DEG, int N_DIM>
ErrorType SplineGenerator<N_DEG, N_DIM>::GetWaypointsFromPositionSamples(
    const vec_Vecf<N_DIM>& samples, const std::vector<decimal_t>& para,
    vec_E<Waypoint<N_DIM>>* waypoints) {
  int num_samples = static_cast<int>(samples.size());
  int num_paras = static_cast<int>(para.size());
  if (num_samples != num_paras) {
    return kIllegalInput;
  }

  waypoints->clear();
  waypoints->reserve(num_samples);
  for (int i = 0; i < num_samples; i++) {
    Waypoint<N_DIM> wp;
    wp.pos = samples[i];
    wp.fix_pos = true;   // 标记位置为固定约束
    wp.t = para[i];
    wp.stamped = true;
    waypoints->push_back(wp);
  }

  return kSuccess;
}

/*
 * GetSplineFromStateVec - 从状态向量生成样条（使用加加速度最优连接）
 *
 * 对于相邻两个状态点，使用五阶多项式进行加加速度最优(Jerk-Optimal)
 * 连接。五阶多项式可以同时满足起止点的位置、速度、加速度约束，
 * 且使得加加速度（jerk）的平方积分最小化。
 *
 * 流程：
 * 1. 将 State 转换为 FreeState（笛卡尔分量形式）
 * 2. 仅对前两个维度（x, y）构造连接多项式
 * 3. 其余维度（如有）设为零
 *
 * @param para 弧长参数序列（对应每个状态点的时间/弧长）
 * @param state_vec 状态向量（自行车模型状态）
 * @param spline 输出参数，生成的样条
 * @return kSuccess；kWrongStatus 阶数不足或参数不足
 */
template <int N_DEG, int N_DIM>
ErrorType SplineGenerator<N_DEG, N_DIM>::GetSplineFromStateVec(
    const std::vector<decimal_t>& para, const vec_E<State>& state_vec,
    SplineType* spline) {
  if (N_DEG < 5) {
    printf(
        "[GetSplineFromStateVec]To fit state vec we need at least 5 "
        "degree.\n");
    return kWrongStatus;
  }
  if (para.size() < 2) {
    printf("[GetSplineFromStateVec]failed to get spline from state vec.\n");
    return kWrongStatus;
  }

  assert(para.size() == state_vec.size());
  spline->set_vec_domain(para);
  int num_pts = static_cast<int>(state_vec.size());
  for (int i = 0; i < num_pts - 1; i++) {
    FreeState s0, s1;
    GetFreeStateFromState(state_vec[i], &s0);      // 转换为自由状态
    GetFreeStateFromState(state_vec[i + 1], &s1);
    for (int d = 0; d < N_DIM; d++) {
      if (d <= 1) {
        // 对x和y维度构造加加速度最优连接
        (*spline)(i, d).GetJerkOptimalConnection(
            s0.position[d], s0.velocity[d], s0.acceleration[d],
            s1.position[d], s1.velocity[d], s1.acceleration[d],
            para[i + 1] - para[i]);
      } else {
        (*spline)(i, d).set_zero();  // 其他维度清零
      }
    }
  }
  return kSuccess;
}

/*
 * GetSplineFromFreeStateVec - 从自由状态向量生成样条（加加速度最优连接）
 *
 * 与 GetSplineFromStateVec 功能相同，但输入已经是 FreeState 格式，
 * 无需先进行状态转换。
 *
 * @param para 弧长参数序列
 * @param free_state_vec 自由状态向量
 * @param spline 输出参数，生成的样条
 * @return kSuccess；kWrongStatus 阶数不足或参数不足
 */
template <int N_DEG, int N_DIM>
ErrorType SplineGenerator<N_DEG, N_DIM>::GetSplineFromFreeStateVec(
    const std::vector<decimal_t>& para, const vec_E<FreeState>& free_state_vec,
    SplineType* spline) {
  if (N_DEG < 5) {
    printf(
        "[GetSplineFromStateVec]To fit state vec we need at least 5 "
        "degree.\n");
    return kWrongStatus;
  }
  if (para.size() < 2) {
    printf("[GetSplineFromStateVec]failed to get spline from state vec.\n");
    return kWrongStatus;
  }
  assert(para.size() == free_state_vec.size());
  spline->set_vec_domain(para);
  int num_pts = static_cast<int>(free_state_vec.size());

  for (int i = 0; i < num_pts - 1; i++) {
    FreeState s0 = free_state_vec[i];
    FreeState s1 = free_state_vec[i + 1];
    for (int d = 0; d < N_DIM; d++) {
      if (d <= 1) {
        (*spline)(i, d).GetJerkOptimalConnection(
            s0.position[d], s0.velocity[d], s0.acceleration[d],
            s1.position[d], s1.velocity[d], s1.acceleration[d],
            para[i + 1] - para[i]);
      } else {
        (*spline)(i, d).set_zero();
      }
    }
  }
  return kSuccess;
}

/*
 * GetBezierSplineUsingCorridor - 基于时空语义走廊生成贝塞尔样条（带参考路径）
 *
 * 在给定的时空约束立方体（cubes）中优化生成贝塞尔曲线。
 *
 * 【问题表述】求解以下二次规划问题：
 *
 *   min  energy(x) + weight_proximity * proximity_term(x)
 *   s.t. A_eq * x = b_eq       (等式约束: 连续性 + 起止状态)
 *        lbd <= C_ineq * x <= ubd  (不等式约束: 位置/速度/加速度界)
 *
 * 【能量项 energy(x)】
 *   x' * H * x，其中 H 为贝塞尔曲线的三阶导数（加加速度）能量矩阵。
 *   最小化加加速度能量使曲线平滑。
 *
 * 【接近项 proximity_term(x)】
 *   ||贝塞尔曲线在参考采样点处的值 - 参考点值||^2。
 *   引导曲线尽量靠近给定的参考路径。
 *
 * 【等式约束】
 *   - 连接处的位置、速度、加速度、加加速度连续性
 *   - 起始和结束状态约束（位置、速度、加速度）
 *
 * 【不等式约束】
 *   - 位置界约束（每个控制点处）
 *   - 速度界约束（贝塞尔速度由相邻控制点差值表达）
 *   - 加速度界约束（贝塞尔加速度由三个相邻控制点表达）
 *
 * @param cubes 时空语义立方体序列，定义每个分段的时空约束（上下界）
 * @param start_constraints 起始状态约束 [位置, 速度, 加速度]
 * @param end_constraints 结束状态约束 [位置, 速度, 加速度]
 * @param ref_stamps 参考路径的时间戳
 * @param ref_points 参考路径点
 * @param weight_proximity 接近项的权重系数
 * @param bezier_spline 输出参数，生成的贝塞尔样条
 * @return kSuccess；kWrongStatus 求解失败
 */
template <int N_DEG, int N_DIM>
ErrorType SplineGenerator<N_DEG, N_DIM>::GetBezierSplineUsingCorridor(
    const vec_E<SpatioTemporalSemanticCubeNd<N_DIM>>& cubes,
    const vec_E<Vecf<N_DIM>>& start_constraints,
    const vec_E<Vecf<N_DIM>>& end_constraints,
    const std::vector<decimal_t>& ref_stamps,
    const vec_E<Vecf<N_DIM>>& ref_points, const decimal_t& weight_proximity,
    BezierSplineType* bezier_spline) {
  int num_segments = static_cast<int>(cubes.size());
  int num_order = N_DEG + 1;         // 每个分段多项式系数数量
  int derivative_degree = 3;          // 能量最小化中使用三阶导数（jerk能量）

  // ========== 阶段一: 构造目标函数 ==========
  int total_num_vals = N_DIM * num_segments * num_order;
  // Q矩阵: 贝塞尔曲线的加加速度能量矩阵 (Hessian)
  Eigen::SparseMatrix<double, Eigen::RowMajor> Q(total_num_vals,
                                                 total_num_vals);
  Q.reserve(
      Eigen::VectorXi::Constant(N_DIM * num_segments * num_order, num_order));
  {
    MatNf<N_DEG + 1> hessian =
        BezierUtils<N_DEG>::GetBezierHessianMat(derivative_degree);
    int idx, idy;
    decimal_t val;
    for (int n = 0; n < num_segments; n++) {
      decimal_t duration = cubes[n].t_ub - cubes[n].t_lb;
      for (int d = 0; d < N_DIM; d++) {
        for (int j = 0; j < num_order; j++) {
          for (int k = 0; k < num_order; k++) {
            idx = d * num_segments * num_order + n * num_order + j;
            idy = d * num_segments * num_order + n * num_order + k;
            val = hessian(j, k) / pow(duration, 2 * derivative_degree - 3);
            Q.insert(idx, idy) = val;
          }
        }
      }
    }
  }

  // c向量: 目标函数的线性项（初始为空）
  Eigen::VectorXd c;
  c.resize(total_num_vals);
  c.setZero();

  // ========== 接近项 (Proximity Term) 的构造 ==========
  // P矩阵和c向量来自最小化 ||贝塞尔值 - 参考值||^2
  Eigen::SparseMatrix<double, Eigen::RowMajor> P(total_num_vals,
                                                 total_num_vals);
  P.reserve(
      Eigen::VectorXi::Constant(N_DIM * num_segments * num_order, num_order));

  if (!ref_stamps.empty()) {
    int idx, idy;
    int num_ref_samples = ref_stamps.size();
    for (int i = 0; i < num_ref_samples; i++) {
      // 跳过不在任何立方体范围内的参考点
      if (ref_stamps[i] < cubes[0].t_lb ||
          ref_stamps[i] > cubes[num_segments - 1].t_ub)
        continue;

      // 找到参考点所属的立方体分段
      int n;
      for (n = 0; n < num_segments; n++) {
        if (cubes[n].t_ub > ref_stamps[i]) {
          break;
        }
      }
      n = std::min(num_segments - 1, n);
      decimal_t s = cubes[n].t_ub - cubes[n].t_lb;  // 分段长度
      decimal_t t = ref_stamps[i] - cubes[n].t_lb;   // 分段内相对时间
      for (int d = 0; d < N_DIM; d++) {
        for (int j = 0; j < num_order; j++) {
          idx = d * num_segments * num_order + n * num_order + j;
          // 构造线性项: -2 * ref * basis_j(t)
          c[idx] += -2 * ref_points[i][d] * s * nchoosek(N_DEG, j) *
                    pow(t / s, j) * pow(1 - t / s, N_DEG - j);
          for (int k = 0; k < num_order; k++) {
            idy = d * num_segments * num_order + n * num_order + k;
            // 构造二次型项: basis_j(t) * basis_k(t) * s^2
            P.coeffRef(idx, idy) += s * s * nchoosek(N_DEG, j) *
                                    nchoosek(N_DEG, k) * pow(t / s, j + k) *
                                    pow(1 - t / s, 2 * N_DEG - j - k);
          }
        }
      }
    }
  }

  // 将接近项的贡献合并到目标函数中
  P = P * weight_proximity;
  c = c * weight_proximity;
  Q = 2 * (Q + P);  // 最终Q矩阵 = 2 * (能量Hessian + 接近项Hessian)

  // ========== 阶段二: 构造等式约束 ==========
  int num_continuity = 3;  // 位置、速度、加速度连续 (含加加速度共4阶)
  int num_connections = num_segments - 1;
  int num_continuity_constraints = N_DIM * num_connections * num_continuity;
  int num_start_eq_constraints =
      static_cast<int>(start_constraints.size()) * N_DIM;
  int num_end_eq_constraints =
      static_cast<int>(end_constraints.size()) * N_DIM;
  int total_num_eq_constraints = num_continuity_constraints +
                                 num_start_eq_constraints +
                                 num_end_eq_constraints;
  Eigen::SparseMatrix<double, Eigen::RowMajor> A(
      total_num_eq_constraints, N_DIM * num_segments * num_order);
  A.reserve(Eigen::VectorXi::Constant(total_num_eq_constraints, 2 * num_order));

  Eigen::VectorXd b;
  b.resize(total_num_eq_constraints);
  b.setZero();

  int idx, idy;
  decimal_t val;
  {
    // ---- 连续性约束 ----
    for (int n = 0; n < num_connections; n++) {
      decimal_t duration_l = cubes[n].t_ub - cubes[n].t_lb;
      decimal_t duration_r = cubes[n + 1].t_ub - cubes[n + 1].t_lb;
      for (int c = 0; c < num_continuity; c++) {
        decimal_t scale_l = pow(duration_l, 1 - c);
        decimal_t scale_r = pow(duration_r, 1 - c);
        for (int d = 0; d < N_DIM; d++) {
          idx = d * num_connections * num_continuity + n * num_continuity + c;
          if (c == 0) {
            // 位置连续: 左分段的最后一个控制点 = 右分段的第一个控制点
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 0;
            val = 1.0 * scale_r;
            A.insert(idx, idy) = -val;
          } else if (c == 1) {
            // 速度连续: N_DEG * (p_N - p_{N-1}) * scale 应相等
            // 左分段末尾速度 end
            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
            val = -1.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * scale_l;
            A.insert(idx, idy) = val;
            // 右分段起始速度 begin
            idy = d * num_segments * num_order + (n + 1) * num_order;
            val = -1.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 1;
            val = 1.0 * scale_r;
            A.insert(idx, idy) = -val;
          } else if (c == 2) {
            // 加速度连续: N_DEG*(N_DEG-1) * (p_N - 2*p_{N-1} + p_{N-2}) * scale
            // 左分段末尾加速度 end
            idy = d * num_segments * num_order + n * num_order + N_DEG - 2;
            val = 1.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
            val = -2.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * scale_l;
            A.insert(idx, idy) = val;
            // 右分段起始加速度 begin
            idy = d * num_segments * num_order + (n + 1) * num_order;
            val = 1.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 1;
            val = -2.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 2;
            val = 1.0 * scale_r;
            A.insert(idx, idy) = -val;
          } else if (c == 3) {
            // 加加速度(Jerk)连续
            // 左分段末尾jerk end
            idy = d * num_segments * num_order + n * num_order + N_DEG - 3;
            val = -1.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG - 2;
            val = 3.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
            val = -3.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * scale_l;
            A.insert(idx, idy) = val;
            // 右分段起始jerk begin
            idy = d * num_segments * num_order + (n + 1) * num_order;
            val = -1.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 1;
            val = 3.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 2;
            val = -3.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 3;
            val = 1.0 * scale_r;
            A.insert(idx, idy) = -val;
          }
        }
      }
    }

    // ---- 起始状态约束 ----
    {
      int num_order_constraint_start =
          static_cast<int>(start_constraints.size());
      decimal_t duration = cubes[0].t_ub - cubes[0].t_lb;
      decimal_t scale;
      int n = 0;
      for (int j = 0; j < num_order_constraint_start; j++) {
        scale = pow(duration, 1 - j);
        for (int d = 0; d < N_DIM; d++) {
          idx = num_continuity_constraints + d * num_order_constraint_start + j;
          if (j == 0) {
            // 位置约束: p_0 = 设定值
            idy = d * num_segments * num_order + n * num_order + 0;
            val = 1.0 * scale;
            A.insert(idx, idy) = val;
            b[idx] = start_constraints[j][d];
          } else if (j == 1) {
            // 速度约束: N_DEG * (p_1 - p_0) = 设定值
            idy = d * num_segments * num_order + n * num_order + 0;
            val = -1.0 * N_DEG * scale;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + 1;
            val = 1.0 * N_DEG * scale;
            A.insert(idx, idy) = val;
            b[idx] = start_constraints[j][d];
          } else if (j == 2) {
            // 加速度约束: N_DEG*(N_DEG-1) * (p_2 - 2*p_1 + p_0) = 设定值
            idy = d * num_segments * num_order + n * num_order + 0;
            val = 1.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + 1;
            val = -2.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + 2;
            val = 1.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;
            b[idx] = start_constraints[j][d];
          }
        }
      }
    }
    // ---- 结束状态约束 ----
    {
      int num_order_constraint_end = static_cast<int>(end_constraints.size());
      decimal_t duration =
          cubes[num_segments - 1].t_ub - cubes[num_segments - 1].t_lb;
      decimal_t scale;
      int n = num_segments - 1;
      int accu_eq_cons_idx =
          num_continuity_constraints + num_start_eq_constraints;
      for (int j = 0; j < num_order_constraint_end; j++) {
        scale = pow(duration, 1 - j);
        for (int d = 0; d < N_DIM; d++) {
          idx = accu_eq_cons_idx++;
          if (j == 0) {
            // 位置约束: p_N = 设定值
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * scale;
            A.insert(idx, idy) = val;
            b[idx] = end_constraints[j][d];
          } else if (j == 1) {
            // 速度约束
            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
            val = -1.0 * N_DEG * scale;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * N_DEG * scale;
            A.insert(idx, idy) = val;
            b[idx] = end_constraints[j][d];
          } else if (j == 2) {
            // 加速度约束
            idy = d * num_segments * num_order + n * num_order + N_DEG - 2;
            val = 1.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
            val = -2.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;
            b[idx] = end_constraints[j][d];
          }
        }
      }
    }
  }

  // ========== 阶段三: 构造不等式约束 ==========
  // 每个立方体提供位置、速度、加速度的上下界
  int total_num_ineq = 0;
  for (int i = 0; i < num_segments; i++) {
    total_num_ineq += (static_cast<int>(cubes[i].p_ub.size())) * num_order;
    total_num_ineq +=
        (static_cast<int>(cubes[i].v_ub.size())) * (num_order - 1);
    total_num_ineq +=
        (static_cast<int>(cubes[i].a_ub.size())) * (num_order - 2);
  }
  Eigen::VectorXd lbd;
  Eigen::VectorXd ubd;
  Eigen::SparseMatrix<double, Eigen::RowMajor> C(total_num_ineq,
                                                 total_num_vals);
  C.reserve(Eigen::VectorXi::Constant(total_num_ineq, 3));
  lbd.setZero(total_num_ineq);
  ubd.setZero(total_num_ineq);
  {
    int accu_num_ineq = 0;
    for (int n = 0; n < num_segments; n++) {
      decimal_t duration = cubes[n].t_ub - cubes[n].t_lb;
      decimal_t scale;
      for (int d = 0; d < N_DIM; d++) {
        // 位置界约束: 每个控制点都要在位置上下界内
        scale = pow(duration, 1 - 0);
        for (int j = 0; j < num_order; j++) {
          idx = accu_num_ineq;
          idy = d * num_segments * num_order + n * num_order + j;
          val = scale;
          C.insert(idx, idy) = val;
          lbd[idx] = cubes[n].p_lb[d];
          ubd[idx] = cubes[n].p_ub[d];
          accu_num_ineq++;
        }
        // 速度界约束: N_DEG * (p_{j+1} - p_j) 在速度上下界内
        scale = pow(duration, 1 - 1);
        for (int j = 0; j < num_order - 1; j++) {
          idx = accu_num_ineq;
          idy = d * num_segments * num_order + n * num_order + j;
          val = -N_DEG * scale;
          C.insert(idx, idy) = val;
          idy = d * num_segments * num_order + n * num_order + (j + 1);
          val = N_DEG * scale;
          C.insert(idx, idy) = val;
          lbd[idx] = cubes[n].v_lb[d];
          ubd[idx] = cubes[n].v_ub[d];
          accu_num_ineq++;
        }
        // 加速度界约束: N_DEG*(N_DEG-1) * (p_{j+2} - 2*p_{j+1} + p_j) 在上下界内
        scale = pow(duration, 1 - 2);
        for (int j = 0; j < num_order - 2; j++) {
          idx = accu_num_ineq;
          idy = d * num_segments * num_order + n * num_order + j;
          val = N_DEG * (N_DEG - 1) * scale;
          C.insert(idx, idy) = val;
          idy = d * num_segments * num_order + n * num_order + (j + 1);
          val = -2.0 * N_DEG * (N_DEG - 1) * scale;
          C.insert(idx, idy) = val;
          idy = d * num_segments * num_order + n * num_order + (j + 2);
          val = N_DEG * (N_DEG - 1) * scale;
          C.insert(idx, idy) = val;
          lbd[idx] = cubes[n].a_lb[d];
          ubd[idx] = cubes[n].a_ub[d];
          accu_num_ineq++;
        }
      }
    }
  }

  // 控制点的默认界（不做额外约束）
  Eigen::VectorXd u = std::numeric_limits<double>::max() *
                      Eigen::VectorXd::Ones(total_num_vals);
  Eigen::VectorXd l = (-u.array()).matrix();

  // ========== 阶段四: 求解QP问题 ==========
  Eigen::VectorXd x;
  x.setZero(total_num_vals);
  if (!OoQpItf::solve(Q, c, A, b, C, lbd, ubd, l, u, x, true, false)) {
    printf("[GetBezierSplineUsingCorridor]Solver error.\n");
    return kWrongStatus;
  }

  // ========== 阶段五: 将解填充到贝塞尔样条结构中 ==========
  std::vector<decimal_t> vec_domain;
  vec_domain.push_back(cubes.front().t_lb);
  for (int n = 0; n < num_segments; n++) {
    vec_domain.push_back(cubes[n].t_ub);
  }
  bezier_spline->set_vec_domain(vec_domain);
  for (int n = 0; n < num_segments; n++) {
    for (int j = 0; j < num_order; j++) {
      Vecf<N_DIM> coeff;
      for (int d = 0; d < N_DIM; d++)
        coeff[d] = x[d * num_segments * num_order + n * num_order + j];
      bezier_spline->set_coeff(n, j, coeff);
    }
  }
  return kSuccess;
}

/*
 * GetBezierSplineUsingCorridor - 基于时空语义走廊生成贝塞尔样条（无参考路径）
 *
 * 与上一个重载版本功能相同，但不包含接近项（proximity term），
 * 仅最小化加加速度能量，并在走廊约束内优化。
 *
 * 适用于参考路径不可用或不需要引导曲线靠近特定路径的场景。
 *
 * @param cubes 时空语义立方体序列
 * @param start_constraints 起始状态约束
 * @param end_constraints 结束状态约束
 * @param bezier_spline 输出参数，生成的贝塞尔样条
 * @return kSuccess；kWrongStatus 求解失败
 */
template <int N_DEG, int N_DIM>
ErrorType SplineGenerator<N_DEG, N_DIM>::GetBezierSplineUsingCorridor(
    const vec_E<SpatioTemporalSemanticCubeNd<N_DIM>>& cubes,
    const vec_E<Vecf<N_DIM>>& start_constraints,
    const vec_E<Vecf<N_DIM>>& end_constraints,
    BezierSplineType* bezier_spline) {
  int num_segments = static_cast<int>(cubes.size());
  int num_order = N_DEG + 1;
  int derivative_degree = 3;

  // 构造Q矩阵（加加速度能量Hessian）
  int total_num_vals = N_DIM * num_segments * num_order;
  Eigen::SparseMatrix<double, Eigen::RowMajor> Q(total_num_vals,
                                                 total_num_vals);
  Q.reserve(
      Eigen::VectorXi::Constant(N_DIM * num_segments * num_order, num_order));
  {
    MatNf<N_DEG + 1> hessian =
        BezierUtils<N_DEG>::GetBezierHessianMat(derivative_degree);
    int idx, idy;
    decimal_t val;
    for (int n = 0; n < num_segments; n++) {
      decimal_t duration = cubes[n].t_ub - cubes[n].t_lb;
      for (int d = 0; d < N_DIM; d++) {
        for (int j = 0; j < num_order; j++) {
          for (int k = 0; k < num_order; k++) {
            idx = d * num_segments * num_order + n * num_order + j;
            idy = d * num_segments * num_order + n * num_order + k;
            val = hessian(j, k) / pow(duration, 2 * derivative_degree - 3);
            Q.insert(idx, idy) = val;
          }
        }
      }
    }
  }

  Eigen::VectorXd c;
  c.resize(total_num_vals);
  c.setZero();

  // 等式约束部分与带参考路径版本相同
  int num_continuity = 3;
  int num_connections = num_segments - 1;
  int num_continuity_constraints = N_DIM * num_connections * num_continuity;
  int num_start_eq_constraints =
      static_cast<int>(start_constraints.size()) * N_DIM;
  int num_end_eq_constraints =
      static_cast<int>(end_constraints.size()) * N_DIM;
  int total_num_eq_constraints = num_continuity_constraints +
                                 num_start_eq_constraints +
                                 num_end_eq_constraints;

  // [以上等式约束构造逻辑与带参考路径版本相同，此处省略详细注释]

  Eigen::SparseMatrix<double, Eigen::RowMajor> A(
      total_num_eq_constraints, N_DIM * num_segments * num_order);
  A.reserve(Eigen::VectorXi::Constant(total_num_eq_constraints, 2 * num_order));

  Eigen::VectorXd b;
  b.resize(total_num_eq_constraints);
  b.setZero();

  int idx, idy;
  decimal_t val;
  {
    for (int n = 0; n < num_connections; n++) {
      decimal_t duration_l = cubes[n].t_ub - cubes[n].t_lb;
      decimal_t duration_r = cubes[n + 1].t_ub - cubes[n + 1].t_lb;
      for (int c = 0; c < num_continuity; c++) {
        decimal_t scale_l = pow(duration_l, 1 - c);
        decimal_t scale_r = pow(duration_r, 1 - c);
        for (int d = 0; d < N_DIM; d++) {
          idx = d * num_connections * num_continuity + n * num_continuity + c;
          if (c == 0) {
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 0;
            val = 1.0 * scale_r;
            A.insert(idx, idy) = -val;
          } else if (c == 1) {
            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
            val = -1.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + (n + 1) * num_order;
            val = -1.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 1;
            val = 1.0 * scale_r;
            A.insert(idx, idy) = -val;
          } else if (c == 2) {
            idy = d * num_segments * num_order + n * num_order + N_DEG - 2;
            val = 1.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
            val = -2.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + (n + 1) * num_order;
            val = 1.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 1;
            val = -2.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 2;
            val = 1.0 * scale_r;
            A.insert(idx, idy) = -val;
          } else if (c == 3) {
            idy = d * num_segments * num_order + n * num_order + N_DEG - 3;
            val = -1.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG - 2;
            val = 3.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
            val = -3.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + (n + 1) * num_order;
            val = -1.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 1;
            val = 3.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 2;
            val = -3.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 3;
            val = 1.0 * scale_r;
            A.insert(idx, idy) = -val;
          }
        }
      }
    }

    {
      int num_order_constraint_start =
          static_cast<int>(start_constraints.size());
      decimal_t duration = cubes[0].t_ub - cubes[0].t_lb;
      decimal_t scale;
      int n = 0;
      for (int j = 0; j < num_order_constraint_start; j++) {
        scale = pow(duration, 1 - j);
        for (int d = 0; d < N_DIM; d++) {
          idx = num_continuity_constraints + d * num_order_constraint_start + j;
          if (j == 0) {
            idy = d * num_segments * num_order + n * num_order + 0;
            val = 1.0 * scale;
            A.insert(idx, idy) = val;
            b[idx] = start_constraints[j][d];
          } else if (j == 1) {
            idy = d * num_segments * num_order + n * num_order + 0;
            val = -1.0 * N_DEG * scale;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + 1;
            val = 1.0 * N_DEG * scale;
            A.insert(idx, idy) = val;
            b[idx] = start_constraints[j][d];
          } else if (j == 2) {
            idy = d * num_segments * num_order + n * num_order + 0;
            val = 1.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + 1;
            val = -2.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + 2;
            val = 1.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;
            b[idx] = start_constraints[j][d];
          }
        }
      }
    }
    {
      int num_order_constraint_end = static_cast<int>(end_constraints.size());
      decimal_t duration =
          cubes[num_segments - 1].t_ub - cubes[num_segments - 1].t_lb;
      decimal_t scale;
      int n = num_segments - 1;
      int accu_eq_cons_idx =
          num_continuity_constraints + num_start_eq_constraints;
      for (int j = 0; j < num_order_constraint_end; j++) {
        scale = pow(duration, 1 - j);
        for (int d = 0; d < N_DIM; d++) {
          idx = accu_eq_cons_idx++;
          if (j == 0) {
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * scale;
            A.insert(idx, idy) = val;
            b[idx] = end_constraints[j][d];
          } else if (j == 1) {
            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
            val = -1.0 * N_DEG * scale;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * N_DEG * scale;
            A.insert(idx, idy) = val;
            b[idx] = end_constraints[j][d];
          } else if (j == 2) {
            idy = d * num_segments * num_order + n * num_order + N_DEG - 2;
            val = 1.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
            val = -2.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;
            b[idx] = end_constraints[j][d];
          }
        }
      }
    }
  }

  // 不等式约束
  int total_num_ineq = 0;
  for (int i = 0; i < num_segments; i++) {
    total_num_ineq += (static_cast<int>(cubes[i].p_ub.size())) * num_order;
    total_num_ineq +=
        (static_cast<int>(cubes[i].v_ub.size())) * (num_order - 1);
    total_num_ineq +=
        (static_cast<int>(cubes[i].a_ub.size())) * (num_order - 2);
  }
  Eigen::VectorXd lbd;
  Eigen::VectorXd ubd;
  Eigen::SparseMatrix<double, Eigen::RowMajor> C(total_num_ineq,
                                                 total_num_vals);
  C.reserve(Eigen::VectorXi::Constant(total_num_ineq, 3));
  lbd.setZero(total_num_ineq);
  ubd.setZero(total_num_ineq);
  {
    int accu_num_ineq = 0;
    for (int n = 0; n < num_segments; n++) {
      decimal_t duration = cubes[n].t_ub - cubes[n].t_lb;
      decimal_t scale;
      for (int d = 0; d < N_DIM; d++) {
        scale = pow(duration, 1 - 0);
        for (int j = 0; j < num_order; j++) {
          idx = accu_num_ineq;
          idy = d * num_segments * num_order + n * num_order + j;
          val = scale;
          C.insert(idx, idy) = val;
          lbd[idx] = cubes[n].p_lb[d];
          ubd[idx] = cubes[n].p_ub[d];
          accu_num_ineq++;
        }
        scale = pow(duration, 1 - 1);
        for (int j = 0; j < num_order - 1; j++) {
          idx = accu_num_ineq;
          idy = d * num_segments * num_order + n * num_order + j;
          val = -N_DEG * scale;
          C.insert(idx, idy) = val;
          idy = d * num_segments * num_order + n * num_order + (j + 1);
          val = N_DEG * scale;
          C.insert(idx, idy) = val;
          lbd[idx] = cubes[n].v_lb[d];
          ubd[idx] = cubes[n].v_ub[d];
          accu_num_ineq++;
        }
        scale = pow(duration, 1 - 2);
        for (int j = 0; j < num_order - 2; j++) {
          idx = accu_num_ineq;
          idy = d * num_segments * num_order + n * num_order + j;
          val = N_DEG * (N_DEG - 1) * scale;
          C.insert(idx, idy) = val;
          idy = d * num_segments * num_order + n * num_order + (j + 1);
          val = -2.0 * N_DEG * (N_DEG - 1) * scale;
          C.insert(idx, idy) = val;
          idy = d * num_segments * num_order + n * num_order + (j + 2);
          val = N_DEG * (N_DEG - 1) * scale;
          C.insert(idx, idy) = val;
          lbd[idx] = cubes[n].a_lb[d];
          ubd[idx] = cubes[n].a_ub[d];
          accu_num_ineq++;
        }
      }
    }
  }

  Eigen::VectorXd u = std::numeric_limits<double>::max() *
                      Eigen::VectorXd::Ones(total_num_vals);
  Eigen::VectorXd l = (-u.array()).matrix();

  // 求解
  Eigen::VectorXd x;
  x.setZero(total_num_vals);
  if (!OoQpItf::solve(Q, c, A, b, C, lbd, ubd, l, u, x, true, false)) {
    printf("[GetBezierSplineUsingCorridor]Solver error.\n");
    return kWrongStatus;
  }

  std::vector<decimal_t> vec_domain;
  vec_domain.push_back(cubes.front().t_lb);
  for (int n = 0; n < num_segments; n++) {
    vec_domain.push_back(cubes[n].t_ub);
  }
  bezier_spline->set_vec_domain(vec_domain);
  for (int n = 0; n < num_segments; n++) {
    for (int j = 0; j < num_order; j++) {
      Vecf<N_DIM> coeff;
      for (int d = 0; d < N_DIM; d++)
        coeff[d] = x[d * num_segments * num_order + n * num_order + j];
      bezier_spline->set_coeff(n, j, coeff);
    }
  }
  return kSuccess;
}

/*
 * 显式模板实例化：
 * - SplineGenerator<5, 2>: 五次样条 × 二维（用于车道参考线、路径规划）
 * - SplineGenerator<5, 1>: 五次样条 × 一维（用于速度剖面规划）
 */
template class SplineGenerator<5, 2>;
template class SplineGenerator<5, 1>;

}  // namespace common
