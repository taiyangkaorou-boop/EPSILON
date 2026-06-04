/**
 * @file state_transformer.cc
 * @brief 笛卡尔坐标系与Frenet坐标系之间的状态变换器实现
 *
 * 本文件实现了 StateTransformer 类，它是EPSILON自动驾驶规划系统中最核心的
 * 坐标变换模块，负责在笛卡尔坐标系（Cartesian）和Frenet坐标系之间进行
 * 双向状态转换。
 *
 * ==================== Frenet坐标系简介 ====================
 *
 * Frenet坐标系是一种沿参考路径/车道定义的曲线坐标系：
 * - s: 沿参考路径的纵向距离（弧长参数，类似"沿车道走了多远"）
 * - d (或 l): 相对于参考路径的横向偏移（类似"偏离车道中心多远"）
 *
 * Frenet坐标系的优势：
 * 1. 将二维规划问题解耦为纵向(s方向)和横向(d方向)两个一维问题
 * 2. 便于表达"沿道路"的运动约束（如速度限制沿s方向有意义）
 * 3. 简化了路径规划中的碰撞检测和车道变更逻辑
 *
 * ==================== 转换公式（2D平面） ====================
 *
 * 记参考曲线为 r(s) = (x_r(s), y_r(s))
 * 切线方向: t_r(s) = r'(s) / |r'(s)|
 * 法线方向: n_r(s) = rotate_90_ccw(t_r(s))
 * 曲率: kappa(s)
 *
 * Frenet → Cartesian:
 *   位置: p = r(s) + d * n_r(s)
 *   速度: v = s_dot * (1 - kappa*d) / cos(delta_theta)
 *   朝向: theta = delta_theta + theta_r(s)
 *   其中 delta_theta = atan(d_dot / (1 - kappa*d))
 *   加速度和曲率有更复杂的非线性映射
 *
 * Cartesian → Frenet:
 *   s: 通过求解最近点问题得到（牛顿法/二分法）
 *   d = (p - r(s)) dot n_r(s)
 *   s_dot = v * cos(delta_theta) / (1 - kappa*d)
 *   d_dot = (1 - kappa*d) * tan(delta_theta)
 *   其中 delta_theta = theta - theta_r(s)
 *
 * ==================== 使用注意事项 ====================
 *
 * 1. 变换的有效性条件：1 - kappa * d > 0（即曲率半径必须大于横向偏移）
 *    当此条件不满足时，Frenet坐标出现奇异性
 * 2. 投影容差：当前实现使用 step_tolerance=0.5m 来检测投影偏差
 * 3. Cartesian → Frenet 的 s 坐标通过求解最小距离问题获得，
 *    使用二分法粗搜索 + 牛顿法精搜索的两阶段方法
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/state/state_transformer.h"

namespace common {

/*
 * GetStateFromFrenetState - 将Frenet状态转换为笛卡尔状态
 *
 * 这是从Frenet坐标系到笛卡尔坐标系的前向变换。
 *
 * 算法流程：
 * 1. 验证参考车道是否有效，以及Frenet状态是否可用
 * 2. 获取弧长s处的参考曲线曲率 kappa 和曲率导数 dkappa
 * 3. 检查奇异性条件 1 - kappa*d > 0（分母不能为0或负）
 * 4. 获取弧长s处的参考线位置 r(s) 和切线方向 t_r(s)
 * 5. 计算法线方向 n_r(s)（切线逆时针旋转90度）
 * 6. 通过非线性变换计算笛卡尔系下的位置、速度、朝向、曲率和加速度
 *
 * 关键变换公式：
 * 横向偏角: delta_theta = atan2(d_dot, 1 - kappa*d)
 * 位置: p = r(s) + d * n_r(s)           (法线方向叠加)
 * 速度: v = s_dot * (1 - kappa*d) / cos(delta_theta)
 * 朝向: theta = delta_theta + theta_r
 * 曲率: 由Frenet横向加速度和参考曲率复合计算
 * 加速度: 由Frenet纵向加速度和参考线曲率/切向变化复合计算
 *
 * @param fs 输入的Frenet状态（包含s, s_dot, s_ddot, d, d_dot, d_ddot）
 * @param s 输出参数，转换后的笛卡尔状态
 * @return kSuccess 转换成功；kIllegalInput 参考线无效或状态不可用；
 *         kWrongStatus 曲率获取失败或奇异性
 */
ErrorType StateTransformer::GetStateFromFrenetState(const FrenetState& fs,
                                                    State* s) const {
  if (!lane_.IsValid()) {
    printf("[StateFromFrenetState]Err: lane not valid.\n");
    return kIllegalInput;
  }
  if (!fs.is_ds_usable) {
    // 高速轨迹中可能出现 vs=0 的情况，此时ds不可用
    return kIllegalInput;
  }

  if (LaneDim != 2) {
    printf("[StateFromFrenetState]Err: cannot support non-plane now.\n");
    return kIllegalInput;
  }

  // 步骤1: 获取参考曲线上弧长 s 处的曲率和曲率导数
  decimal_t curvature, curvature_derivative;
  if (lane_.GetCurvatureByArcLength(fs.vec_s[0], &curvature,
                                    &curvature_derivative) != kSuccess) {
    return kWrongStatus;
  }

  // 步骤2: 检查奇异性条件 1 - kappa * d > 0
  // 当此条件不满足时，Frenet坐标变换退化（如高速大横向偏移的情况）
  decimal_t one_minus_curd = 1 - curvature * fs.vec_ds[0];
  if (one_minus_curd < kEPS) {
    return kWrongStatus;
  }

  // 步骤3: 获取弧长 s 处的参考曲线位置（笛卡尔坐标）
  Vecf<LaneDim> lane_pos;
  if (lane_.GetPositionByArcLength(fs.vec_s[0], &lane_pos) != kSuccess) {
    return kWrongStatus;
  }

  // 步骤4: 获取弧长 s 处的切线方向（单位向量）
  Vecf<LaneDim> vec_tangent;
  if (lane_.GetTangentVectorByArcLength(fs.vec_s[0], &vec_tangent) !=
      kSuccess) {
    return kWrongStatus;
  }

  // 步骤5: 计算参考线的朝向角、法线方向和横向偏角
  decimal_t lane_orientation = vec2d_to_angle(vec_tangent);
  Vecf<LaneDim> vec_normal(-vec_tangent[1], vec_tangent[0]);  // 逆时针旋转90度
  decimal_t tan_delta_theta = fs.vec_ds[1] / one_minus_curd;
  decimal_t delta_theta = atan2(fs.vec_ds[1], one_minus_curd);
  decimal_t cn_delta_theta = cos(delta_theta);

  // 步骤6: 计算笛卡尔坐标系下的状态量
  s->vec_position = vec_normal * fs.vec_ds[0] + lane_pos;  // 位置 = r(s) + d * n
  s->velocity = fs.vec_s[1] * one_minus_curd / cn_delta_theta;
  s->angle = normalize_angle(delta_theta + lane_orientation);

  // 步骤7: 计算笛卡尔曲率
  // 公式包含：横向加速度项、曲率导数项、速度耦合项
  decimal_t lhs = (fs.vec_ds[2] + (curvature_derivative * fs.vec_ds[0] +
                                   curvature * fs.vec_ds[1]) *
                                      tan_delta_theta) *
                  cn_delta_theta * cn_delta_theta / one_minus_curd;
  s->curvature = (lhs + curvature) * cn_delta_theta / one_minus_curd;

  // 步骤8: 计算横向偏角的变化率（用于加速度计算）
  decimal_t delta_theta_derivative = 1.0 /
                                     (1 + tan_delta_theta * tan_delta_theta) *
                                     (fs.vec_ds[2] * one_minus_curd +
                                      curvature * fs.vec_ds[1] * fs.vec_ds[1]) /
                                     pow(one_minus_curd, 2);

  // 步骤9: 计算笛卡尔加速度
  // 包含：Frenet纵向加速度分量、向心加速度分量、曲率导数耦合项
  s->acceleration =
      fs.vec_s[2] * one_minus_curd / cn_delta_theta +
      fs.vec_s[1] * fs.vec_s[1] / cn_delta_theta *
          (one_minus_curd * tan_delta_theta * delta_theta_derivative -
           (curvature_derivative * fs.vec_ds[0] + curvature * fs.vec_ds[1]));
  s->time_stamp = fs.time_stamp;
  return kSuccess;
}

/*
 * GetFrenetStateVectorFromStates - 批量将笛卡尔状态向量转换为Frenet状态向量
 *
 * @param state_vec 输入的笛卡尔状态向量
 * @param fs_vec 输出参数，Frenet状态向量
 * @return kSuccess 全部转换成功；kWrongStatus 任一个转换失败
 */
ErrorType StateTransformer::GetFrenetStateVectorFromStates(
    const vec_E<State> state_vec, vec_E<FrenetState>* fs_vec) const {
  fs_vec->clear();
  fs_vec->reserve(state_vec.size());
  FrenetState fs;
  for (const auto& state : state_vec) {
    if (GetFrenetStateFromState(state, &fs) == kSuccess) {
      fs_vec->push_back(fs);
    } else {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

/*
 * GetStateVectorFromFrenetStates - 批量将Frenet状态向量转换为笛卡尔状态向量
 *
 * @param fs_vec 输入的Frenet状态向量
 * @param state_vec 输出参数，笛卡尔状态向量
 * @return kSuccess 全部转换成功；kWrongStatus 任一个转换失败
 */
ErrorType StateTransformer::GetStateVectorFromFrenetStates(
    const vec_E<FrenetState>& fs_vec, vec_E<State>* state_vec) const {
  State s;
  state_vec->clear();
  state_vec->reserve(fs_vec.size());
  for (auto& fs : fs_vec) {
    if (GetStateFromFrenetState(fs, &s) == kSuccess) {
      state_vec->push_back(s);
    } else {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

/*
 * GetFrenetStateFromState - 将笛卡尔状态转换为Frenet状态
 *
 * 这是从笛卡尔坐标系到Frenet坐标系的逆向变换。
 *
 * 算法流程（两阶段方法）：
 *
 * 【阶段一：投影】找到笛卡尔坐标在参考曲线上最近点的弧长 s
 *   - 调用 lane_.GetArcLengthByVecPosition() 进行二分法+牛顿法求解
 *   - 获取该弧长处的参考位置 r(s)、切线 t_r(s)、法线 n_r(s) 和曲率 kappa(s)
 *   - 验证投影距离偏差是否在容差范围内（0.5米）
 *
 * 【阶段二：状态变换】
 *   - 横向偏移: d = (p - r(s)) dot n_r(s)
 *   - 奇异性检查: 1 - kappa*d > 0
 *   - 朝向偏差: delta_theta = normalize(theta - theta_r)
 *   - 横向速度: d_dot = (1 - kappa*d) * tan(delta_theta)
 *   - 纵向速度: s_dot = v * cos(delta_theta) / (1 - kappa*d)
 *   - 横向加速度 d_ddot 和纵向加速度 s_ddot 由非线性微分方程计算
 *
 * @param s 输入的笛卡尔状态（位置、速度、朝向、曲率、加速度）
 * @param fs 输出参数，转换后的Frenet状态
 * @return kSuccess 转换成功；kIllegalInput 参考线无效；
 *         kWrongStatus 弧长计算失败、曲率获取失败或投影偏差过大
 */
ErrorType StateTransformer::GetFrenetStateFromState(const State& s,
                                                    FrenetState* fs) const {
  if (!lane_.IsValid()) {
    return kIllegalInput;
  }

  // 步骤1: 通过求解最小距离问题找到弧长 s（投影操作）
  decimal_t arc_length;
  if (lane_.GetArcLengthByVecPosition(s.vec_position, &arc_length) !=
      kSuccess) {
    return kWrongStatus;
  }

  // 步骤2: 获取弧长 s 处的参考曲线曲率
  decimal_t curvature, curvature_derivative;
  if (lane_.GetCurvatureByArcLength(arc_length, &curvature,
                                    &curvature_derivative) != kSuccess) {
    return kWrongStatus;
  }

  // 步骤3: 获取弧长 s 处的参考曲线位置
  Vecf<2> lane_position;
  if (lane_.GetPositionByArcLength(arc_length, &lane_position) != kSuccess) {
    return kWrongStatus;
  }

  // 步骤4: 获取弧长 s 处的切线方向和法线方向
  Vecf<2> lane_tangent_vec;
  if (lane_.GetTangentVectorByArcLength(arc_length, &lane_tangent_vec) !=
      kSuccess) {
    return kWrongStatus;
  }
  decimal_t lane_orientation = vec2d_to_angle(lane_tangent_vec);
  Vecf<2> lane_normal_vec = Vecf<2>(-lane_tangent_vec[1], lane_tangent_vec[0]);

  // 步骤5: 验证投影质量
  // 检查投影位置到实际位置沿切线方向的偏差是否在容差内
  const decimal_t step_tolerance = 0.5;
  if (fabs((s.vec_position - lane_position).dot(lane_tangent_vec)) >
      step_tolerance) {
    // 投影偏差过大，可能因为：参考曲线密度不足或局部最小值问题
    return kWrongStatus;
  }

  // 步骤6: 计算横向偏移 d = (p - r(s)) dot n
  decimal_t d = (s.vec_position - lane_position).dot(lane_normal_vec);
  decimal_t one_minus_curd = 1 - curvature * d;
  if (one_minus_curd < kEPS) {
    // 奇异性：1 - kappa*d <= 0，Frenet变换不可行
    return kWrongStatus;
  }
  decimal_t delta_theta = normalize_angle(s.angle - lane_orientation);

  // 步骤7: 计算Frenet坐标系下的运动学量
  decimal_t cn_delta_theta = cos(delta_theta);
  decimal_t tan_delta_theta = tan(delta_theta);

  // 横向速度: d_dot = (1 - kappa*d) * tan(delta_theta)
  decimal_t ds = one_minus_curd * tan_delta_theta;

  // 横向加速度: d_ddot，由曲率变化和向心加速度复合计算
  decimal_t dss =
      -(curvature_derivative * d + curvature * ds) * tan_delta_theta +
      one_minus_curd / pow(cn_delta_theta, 2) *
          (s.curvature * one_minus_curd / cn_delta_theta - curvature);

  // 纵向速度: s_dot = v * cos(delta_theta) / (1 - kappa*d)
  decimal_t sp = s.velocity * cn_delta_theta / one_minus_curd;

  // 横向偏角导数 delta_theta_dot（用于纵向加速度计算）
  decimal_t delta_theta_derivative =
      1 / (1 + pow(tan_delta_theta, 2)) *
      (dss * one_minus_curd + curvature * pow(ds, 2)) / pow(one_minus_curd, 2);

  // 纵向加速度: s_ddot，由笛卡尔加速度转换而来
  decimal_t spp =
      (s.acceleration -
       pow(sp, 2) / cn_delta_theta *
           (one_minus_curd * tan_delta_theta * delta_theta_derivative -
            (curvature_derivative * d + curvature * ds))) *
      cn_delta_theta / one_minus_curd;

  // 步骤8: 加载Frenet状态并设置时间戳
  // 注意：使用 kInitWithDs 模式表示横向状态是相对于弧长s的参数化（而非时间）
  fs->Load(Vecf<3>(arc_length, sp, spp), Vecf<3>(d, ds, dss),
           FrenetState::kInitWithDs);
  fs->time_stamp = s.time_stamp;
  return kSuccess;
}

/*
 * GetFrenetPointFromPoint - 将笛卡尔坐标点转换为Frenet坐标点
 *
 * 这是一个简化版的转换，仅处理位置信息（不考虑速度、加速度等动态量）。
 *
 * @param s 输入的笛卡尔坐标点 (x, y)
 * @param fs 输出参数，Frenet坐标点 (s, d)
 * @return kSuccess；kIllegalInput 参考线无效；
 *         kWrongStatus 弧长计算或位置获取失败
 */
ErrorType StateTransformer::GetFrenetPointFromPoint(const Vec2f& s,
                                                    Vec2f* fs) const {
  if (!lane_.IsValid()) return kIllegalInput;

  // 找到参考曲线上最近点的弧长
  decimal_t arc_length;
  if (lane_.GetArcLengthByVecPosition(s, &arc_length) != kSuccess) {
    return kWrongStatus;
  }

  // 获取弧长处参考位置和法线方向
  Vecf<2> lane_position;
  if (lane_.GetPositionByArcLength(arc_length, &lane_position) != kSuccess) {
    return kWrongStatus;
  }

  Vecf<2> lane_tangent_vec;
  if (lane_.GetTangentVectorByArcLength(arc_length, &lane_tangent_vec) !=
      kSuccess) {
    return kWrongStatus;
  }
  Vecf<2> lane_normal_vec = Vecf<2>(-lane_tangent_vec[1], lane_tangent_vec[0]);

  // 计算横向偏移 d
  decimal_t d = (s - lane_position).dot(lane_normal_vec);

  (*fs)(0) = arc_length;  // s坐标：沿参考曲线的弧长
  (*fs)(1) = d;          // d坐标：横向偏移

  return kSuccess;
}

/*
 * GetFrenetPointVectorFromPoints - 批量将笛卡尔点向量转换为Frenet点向量
 *
 * @param pts 输入的笛卡尔点向量
 * @param fps 输出参数，Frenet点向量
 * @return kSuccess 全部成功；kWrongStatus 任一个失败
 */
ErrorType StateTransformer::GetFrenetPointVectorFromPoints(
    const vec_E<Vec2f>& pts, vec_E<Vec2f>* fps) const {
  fps->clear();
  fps->reserve(pts.size());
  Vec2f fp;
  for (const auto& pt : pts) {
    if (GetFrenetPointFromPoint(pt, &fp) == kSuccess) {
      fps->push_back(fp);
    } else {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

}  // namespace common
