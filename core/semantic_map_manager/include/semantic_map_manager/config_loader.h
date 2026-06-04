/**
 * @file config_loader.h
 * @author EPSILON Autonomous Driving Group
 * @brief 配置加载器——从JSON配置文件解析智能体（Agent）运行参数
 *
 * @details
 * ConfigLoader 负责从指定路径的 JSON 配置文件中解析单个智能体的配置信息，
 * 填充到 AgentConfigInfo 结构体中。它是 SemanticMapManager 初始化的第一步，
 * 在构造函数中被调用。
 *
 * JSON 配置文件结构（示例）：
 * @code
 * {
 *   "agent_config": {
 *     "info": [
 *       {
 *         "id": 0,
 *         "obstacle_map_meta_info": {
 *           "width": 200,
 *           "height": 200,
 *           "resolution": 0.2
 *         },
 *         "surrounding_search_radius": 150.0,
 *         "enable_openloop_prediction": true,
 *         "enable_tracking_noise": false,
 *         "enable_log": false,
 *         "log_file": ""
 *       }
 *     ]
 *   }
 * }
 * @endcode
 *
 * 通过 ego_id_ 与 JSON 中 agent["id"] 进行匹配，实现多智能体共享同一配置文件
 * 但各自提取自己部分的设计。
 *
 * @version 0.1
 * @date 2019-03-20
 */

#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_CONFIG_LOADER_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_CONFIG_LOADER_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <json/json.hpp>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/free_state.h"
#include "common/state/state.h"
#include "semantic_map_manager/basics.h"

namespace semantic_map_manager {

/// @class ConfigLoader
/// @brief 从 JSON 文件加载智能体配置并填充 AgentConfigInfo 结构体
///
/// 使用 nlohmann::json 库解析 JSON，按 ego_id 匹配目标智能体条目。
/// 支持多智能体共享同一配置文件但各自加载对应配置段的设计模式。
class ConfigLoader {
 public:
  ConfigLoader() {}
  /// @brief 通过配置文件路径构造
  /// @param agent_config_path JSON 配置文件路径
  ConfigLoader(const std::string &agent_config_path)
      : agent_config_path_(agent_config_path) {}
  ~ConfigLoader() {}

  inline std::string agent_config_path() const { return agent_config_path_; }
  inline void set_agent_config_path(const std::string &path) {
    agent_config_path_ = path;
  };

  inline int ego_id() const { return ego_id_; }
  /// @brief 设置自车ID——用于在 JSON 配置数组中匹配对应的智能体条目
  inline void set_ego_id(const int &id) { ego_id_ = id; }

  /// @brief 解析智能体配置文件，填充 AgentConfigInfo
  ///
  /// 流程：
  ///   1. 打开 agent_config_path_ 指向的JSON文件
  ///   2. 定位到 root["agent_config"]["info"] 数组
  ///   3. 遍历数组，找到 agent["id"] == ego_id_ 的条目
  ///   4. 提取各字段并填充到 p_agent_config 中
  ///   5. 调用 PrintInfo() 输出配置确认
  ///
  /// @param p_agent_config [输出] 解析后的智能体配置
  ErrorType ParseAgentConfig(AgentConfigInfo *p_agent_config);

 private:
  int ego_id_;                    ///< 自车ID——用于在配置数组中匹配
  std::string agent_config_path_; ///< JSON 配置文件路径
};

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_CONFIG_LOADER_H_
