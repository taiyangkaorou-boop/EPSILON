/**
 * @file thread_pool.h
 * @author Jakob Progsch (original), HKUST Aerial Robotics Group (adaptation)
 * @brief EPSILON 自动驾驶决策规划系统 — 轻量级 C++ 线程池
 *
 * @details
 * 本文件实现了基于 C++11 标准的轻量级线程池，源自开源项目 progschj/ThreadPool，
 * 经 EPSILON 团队适配后纳入 common 库。
 *
 * 线程池的核心价值在于避免频繁创建和销毁线程的系统开销：
 *   - 预创建固定数量的工作线程
 *   - 通过任务队列（std::queue<std::function<void()>>）实现异步任务分发
 *   - 使用条件变量（std::condition_variable）实现高效的任务等待和唤醒
 *   - 通过 std::future 支持异步任务的返回值获取
 *
 * EPSILON 中的典型使用场景：
 *   - 并行轨迹采样和评估（多个候选轨迹并发评估）
 *   - 多车辆行为预测（每个车辆的行为预测任务并行执行）
 *   - 大规模碰撞检测的并行化（多个障碍物的碰撞检测并发进行）
 *
 * @note Enqueue 函数是模板函数，仅在头文件中实现（header-only template）。
 *       析构函数中会自动等待所有工作线程完成当前任务后退出。
 *
 * @version 0.1
 * @date 2019-07-21
 *
 * @copyright Copyright (c) 2019
 * @see https://github.com/progschj/ThreadPool
 */
#ifndef _COMMON_INC_COMMON_BASICS_THREAD_POOL_H_
#define _COMMON_INC_COMMON_BASICS_THREAD_POOL_H_

#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <vector>

namespace common {

/**
 * @class ThreadPool
 * @brief 基于 C++11 的轻量级线程池实现
 *
 * @details
 * 线程池在构造时启动指定数量的工作线程，这些线程在后台循环等待任务。
 * 通过 Enqueue() 函数提交异步任务（返回 std::future 以获取结果）。
 * 析构时自动停止所有线程并等待当前任务完成。
 *
 * 线程同步机制：
 *   - std::mutex (queue_mutex)：保护任务队列的并发访问
 *   - std::condition_variable (condition)：工作线程在任务队列为空时阻塞等待
 *   - stop 标志：析构时通知所有线程退出
 *
 * 使用示例：
 * @code
 *   ThreadPool pool(4);  // 创建4个工作线程
 *   auto result = pool.Enqueue([](int a, int b) { return a + b; }, 3, 4);
 *   int sum = result.get();  // sum = 7
 * @endcode
 */
class ThreadPool {
 public:
  /**
   * @brief 构造函数 — 启动指定数量的工作线程
   * @param threads 工作线程数量（建议设置为 CPU 核心数，避免过度竞争）
   */
  ThreadPool(size_t);

  /**
   * @brief 向线程池提交异步任务
   *
   * @details
   * 将可调用对象及其参数包装为 std::packaged_task 并加入队列。
   * 返回 std::future 以便调用方同步获取任务结果。
   *
   * @tparam F 可调用对象类型（函数、lambda、函数对象等）
   * @tparam Args 参数类型包
   * @param f 可调用对象
   * @param args 参数列表（完美转发）
   * @return std::future<typename std::result_of<F(Args...)>::type> 任务结果的 future
   */
  template <class F, class... Args>
  auto Enqueue(F&& f, Args&&... args)
      -> std::future<typename std::result_of<F(Args...)>::type>;

  /**
   * @brief 析构函数 — 等待所有任务完成后销毁线程池
   *
   * @details
   * 设置 stop 标志 -> 通知所有等待中的工作线程 -> 等待所有线程完成当前任务并退出。
   * 确保没有任务被丢弃，所有已提交的任务都会得到执行。
   */
  ~ThreadPool();

 private:
  std::vector<std::thread> workers;              ///< 工作线程容器
  std::queue<std::function<void()> > tasks;      ///< 任务队列（FIFO）

  std::mutex queue_mutex;                        ///< 任务队列互斥锁
  std::condition_variable condition;             ///< 条件变量（用于工作线程的等待/唤醒）
  bool stop;                                     ///< 停止标志（析构时置为 true）
};

// ========== 内联实现：构造函数 ==========

/**
 * @details
 * 创建 threads 个工作线程，每个线程执行无限循环：
 *   1. 获取互斥锁
 *   2. 在条件变量上等待（直到 stop 为 true 或队列非空）
 *   3. 若 stop 为 true 且队列为空，退出循环
 *   4. 从队列取出一个任务
 *   5. 解锁并执行任务
 *   6. 回到步骤 1
 *
 * @note 使用 lambda 捕获 this 指针来访问成员变量。
 */
inline ThreadPool::ThreadPool(size_t threads) : stop(false) {
  for (size_t i = 0; i < threads; ++i)
    workers.emplace_back([this] {
      for (;;) {
        std::function<void()> task;

        {
          std::unique_lock<std::mutex> lock(this->queue_mutex);
          this->condition.wait(
              lock, [this] { return this->stop || !this->tasks.empty(); });
          if (this->stop && this->tasks.empty()) return;
          task = std::move(this->tasks.front());
          this->tasks.pop();
        }

        task();  // ~ 执行任务（已离开锁的作用域，避免长时间持锁）
      }
    });
}

// ========== 模板实现：Enqueue ==========

/**
 * @details
 * 使用 std::packaged_task 包装可调用对象和参数，使其成为可异步执行的单元。
 * 通过 std::future 让调用方可以在未来某个时刻获取结果。
 *
 * 实现步骤：
 *   1. 推导返回类型 return_type
 *   2. 创建 std::packaged_task 智能指针（shared_ptr 确保生命周期安全）
 *   3. 获取 future
 *   4. 加锁后将 lambda 加入任务队列
 *   5. 通知一个等待中的工作线程
 *   6. 返回 future
 *
 * @throws std::runtime_error 若在 stop=true 后尝试 Enqueue（防止新任务被丢弃）
 */
template <class F, class... Args>
auto ThreadPool::Enqueue(F&& f, Args&&... args)
    -> std::future<typename std::result_of<F(Args...)>::type> {
  using return_type = typename std::result_of<F(Args...)>::type;

  // ~ 将 f(args...) 包装为 packaged_task 以便异步执行
  auto task = std::make_shared<std::packaged_task<return_type()> >(
      std::bind(std::forward<F>(f), std::forward<Args>(args)...));

  std::future<return_type> res = task->get_future();
  {
    std::unique_lock<std::mutex> lock(queue_mutex);

    // ~ 线程池已停止，拒绝新任务加入
    if (stop) throw std::runtime_error("enqueue on stopped ThreadPool");

    tasks.emplace([task]() { (*task)(); });
  }
  condition.notify_one();  // ~ 唤醒一个等待中的工作线程
  return res;
}

// ========== 内联实现：析构函数 ==========

/**
 * @details
 * 1. 加锁设置 stop = true
 * 2. 解锁后通知所有工作线程（condition.notify_all()）
 * 3. 等待所有工作线程完成当前任务并退出（join）
 *
 * @note notify_all 在解锁后调用，避免"惊群效应"（所有线程被唤醒后立即又阻塞在锁上）。
 */
inline ThreadPool::~ThreadPool() {
  {
    std::unique_lock<std::mutex> lock(queue_mutex);
    stop = true;
  }
  condition.notify_all();
  for (std::thread& worker : workers) worker.join();  // ~ 等待所有线程退出
}

}  // namespace common

#endif  // _COMMON_INC_COMMON_BASICS_THREAD_POOL_H_
