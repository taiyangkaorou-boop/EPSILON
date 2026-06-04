/**
 * @file decoder.h
 * @brief ROS 消息解码器 —— 将 vehicle_msgs ROS 消息转换为 common 命名空间的 C++ 数据结构
 *
 * @version 0.1
 * @date 2019
 *
 * 本模块实现了从 ROS 消息类型到 C++ 内部数据结构的单向转换。
 * 所有方法均为静态方法，无需实例化即可调用。
 *
 * 数据转换方向：ROS 消息 → C++ 内部数据结构
 *
 * 转换覆盖的数据类型：
 *   - 车辆状态：State（位置、速度、加速度、曲率、转向角）和 FreeState（自由状态）
 *   - 车辆定义：Vehicle、VehicleParam、VehicleSet
 *   - 车道网络：LaneRaw、LaneNet
 *   - 障碍物：CircleObstacle、PolygonObstacle、ObstacleSet
 *   - 竞技场信息：ArenaInfo、ArenaInfoStatic、ArenaInfoDynamic
 *   - 控制信号：VehicleControlSignal（加速度、转向速率、开环标志）
 *   - 几何类型：Circle、Polygon
 *
 * 典型使用场景：
 *   - 订阅控制信号：ROS 回调接收 vehicle_msgs::msg::ControlSignal →
 *     Decoder::GetControlSignalFromRosControlSignal() → common::VehicleControlSignal
 *   - 接收场景数据：ROS 回调接收 vehicle_msgs::msg::ArenaInfoDynamic →
 *     Decoder::GetSimulatorDataFromRosArenaInfoDynamic() → common::VehicleSet
 *
 * 与 Encoder（encoder.h）互为逆操作：
 *   Encoder: C++ 结构 → ROS 消息
 *   Decoder: ROS 消息 → C++ 结构
 */
#ifndef _VEHICLE_MSGS_INC_VEHICLE_MSGS_DECODER_H__
#define _VEHICLE_MSGS_INC_VEHICLE_MSGS_DECODER_H__

#include <memory>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/free_state.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"

#include "vehicle_msgs/msg/arena_info.hpp"
#include "vehicle_msgs/msg/arena_info_dynamic.hpp"
#include "vehicle_msgs/msg/arena_info_static.hpp"
#include "vehicle_msgs/msg/circle.hpp"
#include "vehicle_msgs/msg/circle_obstacle.hpp"
#include "vehicle_msgs/msg/control_signal.hpp"
#include "vehicle_msgs/msg/free_state.hpp"
#include "vehicle_msgs/msg/lane.hpp"
#include "vehicle_msgs/msg/lane_net.hpp"
#include "vehicle_msgs/msg/obstacle_set.hpp"
#include "vehicle_msgs/msg/occupancy_grid_float.hpp"
#include "vehicle_msgs/msg/occupancy_grid_u_int8.hpp"
#include "vehicle_msgs/msg/polygon_obstacle.hpp"
#include "vehicle_msgs/msg/state.hpp"
#include "vehicle_msgs/msg/vehicle.hpp"
#include "vehicle_msgs/msg/vehicle_param.hpp"
#include "vehicle_msgs/msg/vehicle_set.hpp"

#include "rclcpp/rclcpp.hpp"

namespace vehicle_msgs {

/// @brief ROS 消息解码器（静态工具类）
///
/// 提供一系列静态方法，将 ROS 格式的车辆消息解码为 EPSILON 内部通用数据结构。
/// 所有方法不维护内部状态，线程安全（前提是输入参数不跨线程共享）。
///
/// 命名约定：Get{TargetType}FromRos{SourceType}()
///   例如：GetStateFromRosStateMsg() —— 从 ros State 消息解码为 common::State
class Decoder {
 public:
  /// @brief 将 ROS FreeState 消息解码为 common::FreeState
  /// @param in_state ROS FreeState 消息
  /// @param state [输出] 解码后的自由状态（x, y, vx, vy, ax, ay, angle）
  /// @return kSuccess
  ///
  /// FreeState 是一种无约束的状态表示，不包含曲率和转向角信息。
  /// 时间戳从 ROS 消息头中提取，转换为秒（double）。
  static ErrorType GetFreeStateMsgFromRosFreeState(
      const vehicle_msgs::msg::FreeState &in_state, common::FreeState *state) {
    state->time_stamp = rclcpp::Time(in_state.header.stamp).seconds();
    state->position[0] = in_state.pos.x;
    state->position[1] = in_state.pos.y;
    state->velocity[0] = in_state.vel.x;
    state->velocity[1] = in_state.vel.y;
    state->acceleration[0] = in_state.acc.x;
    state->acceleration[1] = in_state.acc.y;
    state->angle = in_state.angle;
    return kSuccess;
  }

  /// @brief 将 ROS State 消息解码为 common::State
  /// @param in_state ROS State 消息
  /// @param state [输出] 解码后的车辆状态
  ///       包含：时间戳、位置、朝向角、曲率、速度、加速度、转向角
  /// @return kSuccess
  static ErrorType GetStateFromRosStateMsg(const vehicle_msgs::msg::State &in_state,
                                           common::State *state) {
    state->time_stamp = rclcpp::Time(in_state.header.stamp).seconds();
    state->vec_position[0] = in_state.vec_position.x;
    state->vec_position[1] = in_state.vec_position.y;
    state->angle = in_state.angle;
    state->curvature = in_state.curvature;
    state->velocity = in_state.velocity;
    state->acceleration = in_state.acceleration;
    state->steer = in_state.steer;
    return kSuccess;
  }

  /// @brief 将 ROS VehicleSet 消息解码为 common::VehicleSet
  /// @param msg ROS VehicleSet 消息（包含 vehicles[] 数组）
  /// @param p_vehicle_set [输出] 解码后的车辆集合
  /// @return kSuccess
  ///
  /// 遍历 msg.vehicles 数组，逐个解码并插入以 ID 为键的 map 中。
  static ErrorType GetVehicleSetFromRosVehicleSet(
      const vehicle_msgs::msg::VehicleSet &msg, common::VehicleSet *p_vehicle_set) {
    p_vehicle_set->vehicles.clear();
    for (int i = 0; i < (int)msg.vehicles.size(); ++i) {
      common::Vehicle vehicle;
      GetVehicleFromRosVehicle(msg.vehicles[i], &vehicle);
      p_vehicle_set->vehicles.insert(
          std::pair<int, common::Vehicle>(vehicle.id(), vehicle));
    }
    return kSuccess;
  }

  /// @brief 将 ROS Vehicle 消息解码为 common::Vehicle
  /// @param msg ROS Vehicle 消息
  /// @param p_vehicle [输出] 解码后的车辆对象
  /// @return kSuccess
  ///
  /// 分别解码车辆的 ID、子类型、类型、参数和状态。
  static ErrorType GetVehicleFromRosVehicle(const vehicle_msgs::msg::Vehicle &msg,
                                            common::Vehicle *p_vehicle) {
    p_vehicle->set_id(msg.id.data);
    p_vehicle->set_subclass(msg.subclass.data);
    p_vehicle->set_type(msg.type.data);
    common::VehicleParam param;
    GetVehicleParamFromRosVehicleParam(msg.param, &param);
    common::State state;
    GetStateFromRosStateMsg(msg.state, &state);
    p_vehicle->set_param(param);
    p_vehicle->set_state(state);
    return kSuccess;
  }

  /// @brief 将 ROS VehicleParam 消息解码为 common::VehicleParam
  /// @param msg ROS 车辆参数消息
  /// @param p_vehicle_param [输出] 解码后的车辆物理参数
  /// @return kSuccess
  ///
  /// 包含的字段：车宽、车长、轴距、前后悬长度、
  /// 最大转向角、最大纵向/横向加速度、质心到后轴距离 d_cr。
  static ErrorType GetVehicleParamFromRosVehicleParam(
      const vehicle_msgs::msg::VehicleParam &msg,
      common::VehicleParam *p_vehicle_param) {
    p_vehicle_param->set_width(msg.width);
    p_vehicle_param->set_length(msg.length);
    p_vehicle_param->set_wheel_base(msg.wheel_base);
    p_vehicle_param->set_front_suspension(msg.front_suspension);
    p_vehicle_param->set_rear_suspension(msg.rear_suspension);
    p_vehicle_param->set_max_steering_angle(msg.max_steering_angle);
    p_vehicle_param->set_max_longitudinal_acc(msg.max_longitudinal_acc);
    p_vehicle_param->set_max_lateral_acc(msg.max_lateral_acc);
    p_vehicle_param->set_d_cr(msg.d_cr);
    return kSuccess;
  }

  /// @brief 将 ROS LaneNet 消息解码为 common::LaneNet
  /// @param msg ROS 车道网络消息
  /// @param p_lane_net [输出] 解码后的车道网络
  /// @return kSuccess
  ///
  /// 遍历所有车道消息并逐条解码，以车道 ID 为键存入 map。
  static ErrorType GetLaneNetFromRosLaneNet(const vehicle_msgs::msg::LaneNet &msg,
                                            common::LaneNet *p_lane_net) {
    p_lane_net->lane_set.clear();
    for (const auto &lane_msg : msg.lanes) {
      common::LaneRaw lane_raw;
      GetLaneRawFromRosLane(lane_msg, &lane_raw);
      p_lane_net->lane_set.insert(
          std::pair<int, common::LaneRaw>(lane_raw.id, lane_raw));
    }
    return kSuccess;
  }

  /// @brief 将 ROS Lane 消息解码为 common::LaneRaw
  /// @param msg ROS 车道消息
  /// @param p_lane [输出] 解码后的车道原始数据
  /// @return kSuccess
  ///
  /// 解码字段包括：车道 ID、方向、拓扑连接关系（child/father/left/right）、
  /// 变道权限、行为类型、长度、中心线采样点序列、起点和终点坐标。
  static ErrorType GetLaneRawFromRosLane(const vehicle_msgs::msg::Lane &msg,
                                         common::LaneRaw *p_lane) {
    p_lane->id = msg.id;
    p_lane->dir = msg.dir;

    p_lane->child_id = msg.child_id;
    p_lane->father_id = msg.father_id;
    p_lane->l_lane_id = msg.l_lane_id;
    p_lane->l_change_avbl = msg.l_change_avbl;
    p_lane->r_lane_id = msg.r_lane_id;
    p_lane->r_change_avbl = msg.r_change_avbl;
    p_lane->behavior = msg.behavior;
    p_lane->length = msg.length;

    // 提取起点和终点坐标
    p_lane->start_point(0) = msg.start_point.x;
    p_lane->start_point(1) = msg.start_point.y;
    p_lane->final_point(0) = msg.final_point.x;
    p_lane->final_point(1) = msg.final_point.y;
    // 提取中心线采样点序列
    for (const auto pt : msg.points) {
      p_lane->lane_points.push_back(Vec2f(pt.x, pt.y));
    }
    return kSuccess;
  }

  /// @brief 将 ROS ObstacleSet 消息解码为 common::ObstacleSet
  /// @param msg ROS 障碍物集合消息
  /// @param obstacle_set [输出] 解码后的障碍物集合
  /// @return kSuccess
  ///
  /// 分别处理 obs_circle（圆形障碍物）和 obs_polygon（多边形障碍物）两种类型，
  /// 以障碍物 ID 为键存入各自的 map 中。
  static ErrorType GetObstacleSetFromRosObstacleSet(
      const vehicle_msgs::msg::ObstacleSet &msg, common::ObstacleSet *obstacle_set) {
    obstacle_set->obs_circle.clear();
    obstacle_set->obs_polygon.clear();
    // 解码圆形障碍物
    for (const auto &obs : msg.obs_circle) {
      common::CircleObstacle obs_temp;
      GetCircleObstacleFromRosCircleObstacle(obs, &obs_temp);
      obstacle_set->obs_circle.insert(
          std::pair<int, common::CircleObstacle>(obs_temp.id, obs_temp));
    }
    // 解码多边形障碍物
    for (const auto &obs : msg.obs_polygon) {
      common::PolygonObstacle obs_temp;
      GetPolygonObstacleFromRosPolygonObstacle(obs, &obs_temp);
      obstacle_set->obs_polygon.insert(
          std::pair<int, common::PolygonObstacle>(obs_temp.id, obs_temp));
    }
    return kSuccess;
  }

  /// @brief 将 ROS CircleObstacle 消息解码为 common::CircleObstacle
  /// @param msg ROS 圆形障碍物消息
  /// @param circle [输出] 解码后的圆形障碍物
  /// @return kSuccess
  static ErrorType GetCircleObstacleFromRosCircleObstacle(
      const vehicle_msgs::msg::CircleObstacle &msg, common::CircleObstacle *circle) {
    circle->id = msg.id;
    GetCircleFromRosCircle(msg.circle, &circle->circle);
    return kSuccess;
  }

  /// @brief 将 ROS PolygonObstacle 消息解码为 common::PolygonObstacle
  /// @param msg ROS 多边形障碍物消息
  /// @param poly [输出] 解码后的多边形障碍物
  /// @return kSuccess
  static ErrorType GetPolygonObstacleFromRosPolygonObstacle(
      const vehicle_msgs::msg::PolygonObstacle &msg, common::PolygonObstacle *poly) {
    poly->id = msg.id;
    GetPolygonFromRosPolygon(msg.polygon, &poly->polygon);
    return kSuccess;
  }

  /// @brief 将 ROS Circle 消息解码为 common::Circle
  /// @param msg ROS 圆形消息
  /// @param circle [输出] 解码后的圆形（圆心坐标 + 半径）
  /// @return kSuccess
  static ErrorType GetCircleFromRosCircle(const vehicle_msgs::msg::Circle &msg,
                                          common::Circle *circle) {
    circle->center.x = msg.center.x;
    circle->center.y = msg.center.y;
    circle->radius = msg.radius;
    return kSuccess;
  }

  /// @brief 将 ROS Polygon 消息解码为 common::Polygon
  /// @param msg ROS 多边形消息（geometry_msgs::msg::Polygon）
  /// @param poly [输出] 解码后的多边形（顶点列表）
  /// @return kSuccess
  ///
  /// 逐点提取顶点坐标，生成 common::Point 链表。
  static ErrorType GetPolygonFromRosPolygon(const geometry_msgs::msg::Polygon &msg,
                                            common::Polygon *poly) {
    for (const auto p : msg.points) {
      common::Point pt;
      pt.x = p.x;
      pt.y = p.y;
      poly->points.push_back(pt);
    }
    return kSuccess;
  }

  /// @brief 从完整竞技场信息 ROS 消息中提取所有仿真数据
  /// @param msg ROS ArenaInfo 消息
  /// @param time_stamp [输出] 消息时间戳
  /// @param lane_net [输出] 车道网络
  /// @param vehicle_set [输出] 车辆集合
  /// @param obstacle_set [输出] 障碍物集合
  /// @return kSuccess
  ///
  /// 这是最全面的解码入口，一次性提取静态和动态全部数据。
  static ErrorType GetSimulatorDataFromRosArenaInfo(
      const vehicle_msgs::msg::ArenaInfo &msg, rclcpp::Time *time_stamp,
      common::LaneNet *lane_net, common::VehicleSet *vehicle_set,
      common::ObstacleSet *obstacle_set) {
    *time_stamp = msg.header.stamp;
    GetLaneNetFromRosLaneNet(msg.lane_net, lane_net);
    GetVehicleSetFromRosVehicleSet(msg.vehicle_set, vehicle_set);
    GetObstacleSetFromRosObstacleSet(msg.obstacle_set, obstacle_set);
    return kSuccess;
  }

  /// @brief 从静态竞技场信息 ROS 消息中提取场景数据
  /// @param msg ROS ArenaInfoStatic 消息
  /// @param time_stamp [输出] 消息时间戳
  /// @param lane_net [输出] 车道网络
  /// @param obstacle_set [输出] 障碍物集合
  /// @return kSuccess
  ///
  /// 仅提取静态数据（车道网络和障碍物），不包含车辆动态状态。
  /// 适用于只需要地图信息的场景。
  static ErrorType GetSimulatorDataFromRosArenaInfoStatic(
      const vehicle_msgs::msg::ArenaInfoStatic &msg, rclcpp::Time *time_stamp,
      common::LaneNet *lane_net, common::ObstacleSet *obstacle_set) {
    *time_stamp = msg.header.stamp;
    GetLaneNetFromRosLaneNet(msg.lane_net, lane_net);
    GetObstacleSetFromRosObstacleSet(msg.obstacle_set, obstacle_set);
    return kSuccess;
  }

  /// @brief 从动态竞技场信息 ROS 消息中提取车辆状态
  /// @param msg ROS ArenaInfoDynamic 消息
  /// @param time_stamp [输出] 消息时间戳
  /// @param vehicle_set [输出] 车辆集合（所有车辆的实时状态）
  /// @return kSuccess
  ///
  /// 仅提取动态数据（车辆实时状态），适用于只需要感知周围车辆的场景。
  static ErrorType GetSimulatorDataFromRosArenaInfoDynamic(
      const vehicle_msgs::msg::ArenaInfoDynamic &msg, rclcpp::Time *time_stamp,
      common::VehicleSet *vehicle_set) {
    *time_stamp = msg.header.stamp;
    GetVehicleSetFromRosVehicleSet(msg.vehicle_set, vehicle_set);
    return kSuccess;
  }

  /// @brief 将 ROS ControlSignal 消息解码为 common::VehicleControlSignal
  /// @param msg ROS 控制信号消息
  /// @param ctrl [输出] 解码后的车辆控制信号
  /// @return kSuccess
  ///
  /// 解码的核心控制量：
  ///   - acc：纵向加速度
  ///   - steer_rate：转向角变化速率
  ///   - is_openloop：是否为开环控制（true 表示直接使用目标状态，false 表示使用闭环仿真）
  ///   - state：开环模式下的目标状态（仅在 is_openloop == true 时有效）
  ///
  /// 此方法是 phy_simulator_planning_node 中控制信号回调的核心解码函数。
  static ErrorType GetControlSignalFromRosControlSignal(
      const vehicle_msgs::msg::ControlSignal &msg,
      common::VehicleControlSignal *ctrl) {
    ctrl->acc = msg.acc;
    ctrl->steer_rate = msg.steer_rate;
    ctrl->is_openloop = msg.is_openloop.data;
    GetStateFromRosStateMsg(msg.state, &(ctrl->state));
    return kSuccess;
  }
};

}  // namespace vehicle_msgs

#endif  //_VEHICLE_MSGS_INC_VEHICLE_MSGS_DECODER_H__
