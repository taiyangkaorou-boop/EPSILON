/**
 * @file config_loader.cc
 * @author EPSILON Autonomous Driving Group
 * @brief ConfigLoader 类实现——从JSON解析智能体配置
 *
 * @details
 * 该文件实现了从 JSON 配置文件解析单个智能体运行参数的功能。
 *
 * JSON 结构说明：
 *   配置文件包含 "agent_config" -> "info" 数组，每个元素定义一个智能体的参数。
 *   通过 ego_id_ 与 agent["id"] 进行匹配，支持多智能体共享同一配置文件。
 *
 * 解析的配置项：
 *   - obstacle_map_meta_info：栅格地图尺寸(宽x高)和分辨率
 *   - surrounding_search_radius：周围搜索半径（米）
 *   - enable_openloop_prediction：开环轨迹预测开关
 *   - enable_tracking_noise：跟踪噪声注入开关（可选，默认 false）
 *   - enable_log：日志记录开关（可选，默认 false）
 *   - log_file：日志文件路径（可选，enable_log 为 true 时必须）
 */

#include "semantic_map_manager/config_loader.h"

namespace semantic_map_manager {

using Json = nlohmann::json;  // nlohmann JSON 库类型别名

/// @brief 从 JSON 文件解析智能体配置并填充 AgentConfigInfo
///
/// 步骤：
///   1. 打开 agent_config_path_ 指向的 JSON 文件
///   2. 解析 JSON 根对象
///   3. 导航到 root["agent_config"]["info"] 数组
///   4. 遍历数组，找到 agent["id"] == ego_id_ 的条目
///   5. 逐字段提取并填充 p_agent_config
///   6. 调用 PrintInfo() 输出验证
///
/// @param p_agent_config [输出] 解析后的智能体配置
ErrorType ConfigLoader::ParseAgentConfig(AgentConfigInfo *p_agent_config) {
  printf("\n[ConfigLoader] Loading vehicle set\n");

  // 打开 JSON 配置文件
  std::fstream fs(agent_config_path_);
  Json root;
  fs >> root;

  // 定位到智能体配置数组
  Json agent_config_json = root["agent_config"];
  int num = static_cast<int>(agent_config_json["info"].size());

  // 遍历数组查找匹配 ego_id_ 的条目
  for (int i = 0; i < num; ++i) {
    Json agent = agent_config_json["info"][i];
    if (agent["id"].get<int>() != ego_id_) continue;  // 按自车ID匹配

    // ====== 解析障碍物地图元信息 ======
    p_agent_config->obstacle_map_meta_info = common::GridMapMetaInfo(
        agent["obstacle_map_meta_info"]["width"].get<double>(),
        agent["obstacle_map_meta_info"]["height"].get<double>(),
        agent["obstacle_map_meta_info"]["resolution"].get<double>());

    // ====== 解析周围搜索半径 ======
    p_agent_config->surrounding_search_radius =
        agent["surrounding_search_radius"].get<double>();

    // ====== 解析功能开关 ======
    p_agent_config->enable_openloop_prediction =
        agent["enable_openloop_prediction"].get<bool>();

    // ====== 可选配置——跟踪噪声 ======
    if (agent.count("enable_tracking_noise")) {
      p_agent_config->enable_tracking_noise =
          agent["enable_tracking_noise"].get<bool>();
    }
    // ====== 可选配置——日志记录 ======
    if (agent.count("enable_log")) {
      p_agent_config->enable_log = agent["enable_log"].get<bool>();
      p_agent_config->log_file = agent["log_file"].get<std::string>();
    }
  }

  // 输出解析结果验证
  p_agent_config->PrintInfo();
  fs.close();
  return kSuccess;
}

}  // namespace semantic_map_manager
