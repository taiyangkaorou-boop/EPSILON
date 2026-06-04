/**
 * @file ros_adapter.cc
 * @author EPSILON Autonomous Driving Group
 * @brief RosAdapter 类实现——ROS消息订阅与数据适配
 *
 * @details
 * 该文件实现了 RosAdapter 的核心功能：
 *   - Init()：订阅3类 ArenaInfo 话题（完整/静态/动态）
 *   - ArenaInfoCallback()：单帧模式回调——完整数据直接渲染
 *   - ArenaInfoStaticCallback()：分离模式静态回调——缓存车道和障碍物
 *   - ArenaInfoDynamicCallback()：分离模式动态回调——与缓存静态数据组合后渲染
 *   - BindMapUpdateCallback()：注册地图更新后的回调（触发规划器执行）
 */

#include "semantic_map_manager/ros_adapter.h"

namespace semantic_map_manager {

/// @brief 初始化 ROS 订阅器
///
/// 订阅3类话题：
///   - "arena_info"（队列深度2）：完整单帧数据（ArenaInfo），包含 lane_net + vehicle_set + obstacle_set
///   - "arena_info_static"（队列深度2）：静态场景数据（ArenaInfoStatic），帧间不变
///   - "arena_info_dynamic"（队列深度2）：动态数据（ArenaInfoDynamic），每帧更新
///
/// @note 队列深度为2，使用 std::bind 绑定成员函数回调
void RosAdapter::Init() {
  // 与物理仿真器的通信通道——订阅3类数据话题
  arena_info_sub_ = node_->create_subscription<vehicle_msgs::msg::ArenaInfo>(
      "arena_info", 2, std::bind(&RosAdapter::ArenaInfoCallback, this, std::placeholders::_1));
  arena_info_static_sub_ = node_->create_subscription<vehicle_msgs::msg::ArenaInfoStatic>(
      "arena_info_static", 2, std::bind(&RosAdapter::ArenaInfoStaticCallback, this, std::placeholders::_1));
  arena_info_dynamic_sub_ = node_->create_subscription<vehicle_msgs::msg::ArenaInfoDynamic>(
      "arena_info_dynamic", 2, std::bind(&RosAdapter::ArenaInfoDynamicCallback, this, std::placeholders::_1));
}

/// @brief ArenaInfo（单帧完整数据）回调
///
/// 处理流程：
///   1. vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfo 反序列化消息
///   2. DataRenderer::Render 渲染数据并注入语义地图管理器
///   3. 若绑定了回调函数，则触发 private_callback_fn_（通常启动规划器执行）
///
/// @note 单帧模式下每帧都包含完整的 static+dynamic 数据，无需缓存
void RosAdapter::ArenaInfoCallback(const vehicle_msgs::msg::ArenaInfo::SharedPtr msg) {
  rclcpp::Time time_stamp;
  // 从 ROS 消息中解码出车道网络、车辆集合、障碍物集合
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfo(
      *msg, &time_stamp, &lane_net_, &vehicle_set_, &obstacle_set_);
  // 渲染处理并推送到语义地图管理器
  p_data_renderer_->Render(time_stamp.seconds(), lane_net_, vehicle_set_, obstacle_set_);
  // 触发外部回调（通常用于驱动规划器执行）
  if (has_callback_binded_) {
    private_callback_fn_(*p_smm_);
  }
}

/// @brief ArenaInfoStatic（静态场景数据）回调
///
/// 分离模式下，静态数据先到达并被缓存：
///   - 反序列化车道网络和障碍物集合存储在 lane_net_ 和 obstacle_set_ 中
///   - 设置 get_arena_info_static_ = true 标志
///   - 不触发渲染（等待动态数据到达）
///
/// @note 静态场景数据（车道布局、障碍物位置）帧间通常不变，
///        分离发送可以大幅减少网络带宽
void RosAdapter::ArenaInfoStaticCallback(const vehicle_msgs::msg::ArenaInfoStatic::SharedPtr msg) {
  rclcpp::Time time_stamp;
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfoStatic(
      *msg, &time_stamp, &lane_net_, &obstacle_set_);
  get_arena_info_static_ = true;  // 标记已收到静态数据
}

/// @brief ArenaInfoDynamic（动态实体数据）回调
///
/// 分离模式下，动态数据到达时与缓存的静态数据组合：
///   - 反序列化车辆集合存储在 vehicle_set_ 中
///   - 仅当 get_arena_info_static_ 为 true（已收到静态数据）时才触发渲染
///   - 渲染完成后同样触发外部回调
///
/// @note 必须等待静态数据先到达，否则无法构建完整的场景
void RosAdapter::ArenaInfoDynamicCallback(const vehicle_msgs::msg::ArenaInfoDynamic::SharedPtr msg) {
  rclcpp::Time time_stamp;
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfoDynamic(*msg, &time_stamp, &vehicle_set_);

  // 仅当静态数据已就绪时才触发渲染
  if (get_arena_info_static_) {
    p_data_renderer_->Render(time_stamp.seconds(), lane_net_, vehicle_set_, obstacle_set_);
    if (has_callback_binded_) {
      private_callback_fn_(*p_smm_);
    }
  }
}

/// @brief 绑定地图更新后的回调函数
///
/// 注册的回调函数签名：int(const SemanticMapManager&)
/// 在每次 SemanticMapManager::UpdateSemanticMap 完成后被调用。
///
/// 典型使用场景：
///   - 规划器注册自己的 RunOnce 方法作为回调
///   - 当新数据到达并完成语义地图更新后，规划器自动执行一步规划
///
/// @param fn 回调函数对象
void RosAdapter::BindMapUpdateCallback(std::function<int(const SemanticMapManager&)> fn) {
  private_callback_fn_ = std::bind(fn, std::placeholders::_1);
  has_callback_binded_ = true;
}

}  // namespace semantic_map_manager
