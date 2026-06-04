/**
 * @file macros.h
 * @author HKUST Aerial Robotics Group
 * @brief EPSILON 自动驾驶决策规划系统 — 全局宏定义头文件
 *
 * @details
 * 本文件定义了 EPSILON 系统中所有模块共享的预处理宏。
 * 当前内容：
 *   - DECLARE_BACKWARD：栈回溯信号处理器的声明宏
 *
 * DECLARE_BACKWARD 用于在模块入口（如 main 函数）声明 backward-cpp 的信号处理句柄。
 * 当程序因段错误（SIGSEGV）、非法指令（SIGILL）等异常信号崩溃时，
 * backward-cpp 会自动打印完整的调用栈（stack trace），便于定位崩溃位置。
 *
 * 使用方式：
 * @code
 *   int main() {
 *     DECLARE_BACKWARD;  // 注册栈回溯信号处理器
 *     // ... 程序主体 ...
 *   }
 * @endcode
 *
 * @note backward-cpp 需要编译时包含调试信息（-g 编译选项）才能解析函数名和行号。
 *       basics.h 中通过 #define BACKWARD_HAS_UNWIND 1 和 BACKWARD_HAS_DW 1
 *       启用了 unwind 和 DWARF 支持。
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _COMMON_INC_COMMON_MACROS_H__
#define _COMMON_INC_COMMON_MACROS_H__

/**
 * @def DECLARE_BACKWARD
 * @brief 在 backward 命名空间中声明 backward::SignalHandling 对象
 *
 * @details
 * 展开后等价于：
 * @code
 *   namespace backward {
 *     backward::SignalHandling sh;
 *   }
 * @endcode
 *
 * backward::SignalHandling 在构造时注册信号处理器（SIGSEGV, SIGABRT 等），
 * 并在析构时恢复默认信号处理。因此 DECLARE_BACKWARD 宏所在的 scope
 * 即确定了栈回溯功能的生命周期（通常在整个 main 函数 scope 内）。
 */
#define DECLARE_BACKWARD       \
  namespace backward {         \
  backward::SignalHandling sh; \
  }

#endif
