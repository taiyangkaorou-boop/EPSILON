/**
 * @file ros_adapter.h
 * @author EPSILON Autonomous Driving Group
 * @brief ROS适配器——订阅ROS话题，将物理仿真器数据传入语义地图管道
 *
 * @details
 * RosAdapter 是 EPSILON 系统与 ROS 通信层的桥梁，负责：
 *
 * 1. 话题订阅：
 *    - "arena_info" 话题（vehicle_msgs::msg::ArenaInfo）：包含完整的单帧仿真数据
 *      （车道网络 + 车辆集合 + 障碍物集合），用于单帧模式
 *    - "arena_info_static" 话题（vehicle_msgs::msg::ArenaInfoStatic）：
 *      静态场景数据（车道网络 + 障碍物），帧间一般不变
 *    - "arena_info_dynamic" 话题（vehicle_msgs::msg::ArenaInfoDynamic）：
 *      动态实体数据（车辆状态），每帧更新
 *
 * 2. 双模数据处理：
 *    - 单帧模式：直接使用 ArenaInfo 消息，每帧都包含完整的 static + dynamic 数据
 *    - 分离模式：ArenaInfoStatic 先到达并缓存（get_arena_info_static_=true），
 *      ArenaInfoDynamic 到达后与缓存的静态数据组合进行渲染
 *    分离模式的目的是减少带宽——静态场景数据不需要每帧重复发送
 *
 * 3. 数据流路径：
 *    ROS消息 -> vehicle_msgs::Decoder (反序列化) -> DataRenderer::Render ->
 *    SemanticMapManager::UpdateSemanticMap -> 所有规划器
 *
 * 4. 回调绑定（BindMapUpdateCallback）：
 *    允许外部模块注册回调函数，在每次语义地图更新完成后触发，
 *    通常用于触发规划器的 RunOnce 执行
 *
 * @version 0.1
 * @date 2019-03-20
 */

#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_ROS_ADAPTER_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_ROS_ADAPTER_H_

#include <assert.h>

#include <functional>
#include <iostream>
#include <vector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "rclcpp/rclcpp.hpp"
#include "semantic_map_manager/data_renderer.h"
#include "vehicle_msgs/decoder.h"

#include "vehicle_msgs/msg/arena_info.hpp"
#include "vehicle_msgs/msg/arena_info_static.hpp"
#include "vehicle_msgs/msg/arena_info_dynamic.hpp"

namespace semantic_map_manager {

/// @class RosAdapter
/// @brief ROS通信适配器——将仿真器发布的ArenaInfo消息转换为语义地图内部数据
///
/// 该类是仿真器与规划栈之间的关键桥梁，负责：
/// - 订阅3类ROS话题（完整/静态/动态）
/// - 通过 vehicle_msgs::Decoder 反序列化ROS消息到内部数据结构
/// - 调用 DataRenderer 进行数据渲染和预处理
/// - 触发 SemanticMapManager 的核心更新流程
/// - 支持外部模块注册地图更新后的回调函数
class RosAdapter {
 public:
  using GridMap2D = common::GridMapND<uint8_t, 2>;

  /// @brief 构造函数——初始化订阅器和 DataRenderer
  /// @param node ROS2节点共享指针
  /// @param ptr_smm 语义地图管理器指针（数据最终注入目标）
  RosAdapter(std::shared_ptr<rclcpp::Node> node, SemanticMapManager* ptr_smm)
      : node_(node), p_smm_(ptr_smm), p_data_renderer_(new DataRenderer(ptr_smm)) {
    Init();
  }

  ~RosAdapter() {
    delete p_data_renderer_;
  }

  /// @brief 绑定地图更新回调函数
  ///
  /// 注册的回调函数在每次 UpdateSemanticMap 完成后被调用，
  /// 函数签名：int(const SemanticMapManager&)
  /// 典型用途：触发 planner->RunOnce() 使规划器在数据就绪后立即执行
  ///
  /// @param fn 回调函数对象
  void BindMapUpdateCallback(std::function<int(const SemanticMapManager&)> fn);

  void Init();  // Make Init public

 private:
  // ======================== ROS 话题回调函数 ========================

  /// @brief ArenaInfo（单帧完整数据）回调
  /// @details 反序列化完整仿真数据（车道+车辆+障碍物）并直接渲染
  void ArenaInfoCallback(const vehicle_msgs::msg::ArenaInfo::SharedPtr msg);

  /// @brief ArenaInfoStatic（静态场景数据）回调
  /// @details 仅缓存静态数据（车道网络+障碍物），设置 get_arena_info_static_ 标志
  void ArenaInfoStaticCallback(const vehicle_msgs::msg::ArenaInfoStatic::SharedPtr msg);

  /// @brief ArenaInfoDynamic（动态实体数据）回调
  /// @details 仅在收到静态数据后才进行渲染（通过 get_arena_info_static_ 标志保护）
  ///          ——分离模式下动态数据到达才会触发完整的更新管线
  void ArenaInfoDynamicCallback(const vehicle_msgs::msg::ArenaInfoDynamic::SharedPtr msg);

  // ======================== ROS 通信成员 ========================

  std::shared_ptr<rclcpp::Node> node_;  ///< ROS2节点共享指针

  /// @brief ArenaInfo 完整数据订阅器（单帧模式）
  rclcpp::Subscription<vehicle_msgs::msg::ArenaInfo>::SharedPtr arena_info_sub_;
  /// @brief ArenaInfoStatic 静态场景订阅器（分离模式的静态部分）
  rclcpp::Subscription<vehicle_msgs::msg::ArenaInfoStatic>::SharedPtr arena_info_static_sub_;
  /// @brief ArenaInfoDynamic 动态实体订阅器（分离模式的动态部分）
  rclcpp::Subscription<vehicle_msgs::msg::ArenaInfoDynamic>::SharedPtr arena_info_dynamic_sub_;

  // ======================== 数据缓存 ========================

  common::Vehicle ego_vehicle_;       ///< 自车信息缓存
  common::VehicleSet vehicle_set_;    ///< 车辆集合缓存
  common::LaneNet lane_net_;          ///< 车道网络缓存
  common::ObstacleSet obstacle_set_;  ///< 障碍物集合缓存

  // ======================== 数据处理组件 ========================

  DataRenderer* p_data_renderer_;            ///< 数据渲染器（数据预处理）
  SemanticMapManager* p_smm_;                ///< 语义地图管理器（数据最终注入目标）

  bool get_arena_info_static_ = false;       ///< 是否已收到静态场景数据（分离模式标志）

  bool has_callback_binded_ = false;         ///< 是否已绑定地图更新回调
  std::function<int(const SemanticMapManager&)> private_callback_fn_;  ///< 绑定的回调函数
};

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_ROS_ADAPTER_H_
