/**
 * @file basics.h
 * @author HKUST Aerial Robotics Group
 * @brief EPSILON 自动驾驶决策规划系统 — 基础类型定义头文件
 *
 * @details
 * 本文件是 EPSILON 系统中最底层的类型定义模块，被 common/ 下几乎所有其他模块所依赖。
 * 主要提供以下基础设施：
 *   - 统一的浮点精度类型 decimal_t（默认 double）
 *   - 基于 Eigen 库的向量/矩阵类型别名体系（Vecf、Matf、vec_E 等）
 *   - 系统级数学常量（圆周率、无穷大、不同精度的数值容差）
 *   - 全局错误码枚举 ErrorType
 *   - 无效 ID 常量（用于表示未初始化的 agent/lane 标识）
 *   - backward-cpp 栈回溯支持（用于崩溃调试）
 *
 * @note 本文件通过 Eigen::aligned_allocator 解决了 Eigen 定长向量在 STL 容器中的
 *       内存对齐问题，是整个代码库类型安全的基础保证。
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _CORE_COMMON_INC_BASICS_BASICS_H_
#define _CORE_COMMON_INC_BASICS_BASICS_H_

#include "common/basics/macros.h"
#include "common/basics/tic_toc.h"

#include <math.h>
#include <stdio.h>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

/// @brief 启用 backward-cpp 的 unwind 功能，用于解析栈回溯中的函数名
#define BACKWARD_HAS_UNWIND 1
/// @brief 启用 backward-cpp 的 DWARF 调试信息支持
#define BACKWARD_HAS_DW 1
#include "backward.hpp"

/**
 * @enum ErrorType
 * @brief 系统全局错误码枚举
 *
 * @details
 * 定义了 EPSILON 中所有模块统一使用的错误返回码。
 * 所有可能失败的函数都应返回 ErrorType，调用方通过检查返回值是否为 kSuccess 来判断成功。
 *
 * @note kWrongStatus 通常表示系统/模块处于不正确的状态（例如在未初始化时被调用），
 *       kIllegalInput 表示输入参数不合法（越界、空指针、格式错误等），
 *       kUnknown 用于非预期错误兜底。
 */
enum ErrorType {
  kSuccess = 0,       ///< 操作成功完成
  kWrongStatus,       ///< 系统/模块处于错误状态（如未初始化）
  kIllegalInput,      ///< 输入参数非法（越界、空指针、格式错误等）
  kUnknown            ///< 未知错误（兜底类型）
};

/**
 * @brief EPSILON 系统中统一的浮点精度类型
 *
 * @details
 * 默认为 double 精度。所有核心计算（坐标变换、碰撞检测、轨迹优化等）
 * 均使用此类型，保证全系统数值精度一致。如需切换精度级别，仅需修改此处。
 */
using decimal_t = double;

// ============================================================================
// Eigen 类型别名体系 — 向量和矩阵的便捷命名
// ============================================================================

/**
 * @brief 定长浮点列向量模板
 * @tparam N 向量维度
 * @note 底层为 Eigen::Matrix<decimal_t, N, 1>，是 EPSILON 所有二维/三维坐标向量的基础类型
 */
template <int N>
using Vecf = Eigen::Matrix<decimal_t, N, 1>;

/// @brief 定长整数列向量模板
template <int N>
using Veci = Eigen::Matrix<int, N, 1>;

/// @brief M×N 浮点矩阵模板
template <int M, int N>
using Matf = Eigen::Matrix<decimal_t, M, N>;

/// @brief M×N uint8_t 矩阵模板（常用于栅格地图数据）
template <int M, int N>
using Mati8 = Eigen::Matrix<uint8_t, M, N>;

/// @brief M×N 整数矩阵模板
template <int M, int N>
using Mati = Eigen::Matrix<int, M, N>;

/// @brief N×N 浮点方阵模板
template <int N>
using MatNf = Matf<N, N>;

/// @brief 动态行数×N列的浮点矩阵模板（常用于轨迹采样点集合）
template <int N>
using MatDNf = Eigen::Matrix<decimal_t, Eigen::Dynamic, N>;

/// @brief 动态大小浮点矩阵（最通用的矩阵类型）
using MatDf = Matf<Eigen::Dynamic, Eigen::Dynamic>;
/// @brief 动态大小整数矩阵
using MatDi = Mati<Eigen::Dynamic, Eigen::Dynamic>;
/// @brief 动态大小 uint8_t 矩阵（用于图像/栅格数据）
using MatDi8 = Mati8<Eigen::Dynamic, Eigen::Dynamic>;

/// @brief 2×2 浮点矩阵
using Mat2f = Matf<2, 2>;
/// @brief 3×3 浮点矩阵（常用于二维旋转+平移齐次变换）
using Mat3f = Matf<3, 3>;
/// @brief 4×4 浮点矩阵（常用于三维齐次变换）
using Mat4f = Matf<4, 4>;

/// @brief 二维浮点列向量，EPSILON 中最常用的坐标/方向/速度类型
using Vec2f = Vecf<2>;
/// @brief 三维浮点列向量，常用于 (x, y, yaw) 或 (x, y, z)
using Vec3f = Vecf<3>;
/// @brief 四维浮点列向量
using Vec4f = Vecf<4>;

/// @brief 二维整数列向量（栅格坐标）
using Vec2i = Veci<2>;
/// @brief 三维整数列向量（体素坐标 / (x, y, t) 等）
using Vec3i = Veci<3>;
/// @brief 四维整数列向量
using Vec4i = Veci<4>;

/**
 * @brief Eigen 对齐的 STL vector 模板
 *
 * @details
 * Eigen 库的定长向量/矩阵类型（如 Vec2f）在内存中需要按 16 字节对齐。
 * 若直接使用 std::vector<Vec2f> 可能在 SIMD 指令操作时导致段错误。
 * vec_E 通过 Eigen::aligned_allocator 解决此问题，是 EPSILON 中所有
 * Eigen 对象容器的标准类型。
 *
 * @tparam T 元素类型（通常为 Eigen 定长类型）
 */
template <typename T>
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;

/// @brief N维浮点向量的 Eigen 对齐容器
template <int N>
using vec_Vecf = vec_E<Vecf<N>>;

// ============================================================================
// 系统级数学常量
// ============================================================================

/// @brief 粗精度数值容差（1e-1），用于大范围判断（如是否跨越车道线）
const decimal_t kBigEPS = 1e-1;

/// @brief 标准数值容差（1e-6），用于一般浮点比较
const decimal_t kEPS = 1e-6;

/// @brief 精细数值容差（1e-10），用于高精度计算（如碰撞检测中的 SAT 分离轴判断）
const decimal_t kSmallEPS = 1e-10;

/// @brief 圆周率 π = acos(-1.0)
const decimal_t kPi = acos(-1.0);

/// @brief 系统表示"无穷大"的数值（1e20），用于初始化最远距离/代价上限
const decimal_t kInf = 1e20;

// ============================================================================
// 无效 ID 常量
// ============================================================================

/// @brief 无效智能体 ID，表示未初始化或非法车辆标识
const int kInvalidAgentId = -1;
/// @brief 无效车道 ID，表示未初始化或非法车道标识
const int kInvalidLaneId = -1;

#endif  // CORE_COMMON_INC_BASICS_BASICS_H
