/**
 * @file ooqp_interface.cc
 * @brief OOQP（Object-Oriented Quadratic Programming）求解器的接口实现文件
 *
 * 本文件实现了EPSILON自动驾驶规划系统中对OOQP二次规划求解库的封装接口。
 * OOQP是一个面向对象的QP求解器，支持稀疏矩阵和不等式约束。
 *
 * ==================== 二次规划问题标准形式 ====================
 *
 * 本接口求解的QP问题形式为：
 *
 *   min  1/2 * x' * Q * x + c' * x
 *   s.t. A * x = b               (等式约束)
 *        d <= C * x <= f          (不等式约束)
 *        l <= x <= u              (变量边界约束)
 *
 * 其中：
 *   - x: 待求解的优化变量向量 (n维)
 *   - Q: 二次目标函数的Hessian矩阵 (n×n, 半正定对称)
 *   - c: 线性目标函数的系数向量 (n维)
 *   - A: 等式约束矩阵 (m_y × n)
 *   - b: 等式约束右端向量 (m_y维)
 *   - C: 不等式约束矩阵 (m_z × n)
 *   - d, f: 不等式约束的上下界向量 (m_z维)
 *   - l, u: 变量x的上下界向量 (n维)
 *
 * ==================== OOQP求解器设置流程 ====================
 *
 * 1. 确定问题维度(n, my, mz)和非零元素数(nnzQ, nnzA, nnzC)
 * 2. 创建 QpGenSparseMa27 对象（使用MA27稀疏线性求解器）
 * 3. 将Q矩阵转换为下三角形式（OOQP要求对称矩阵的下三角部分）
 * 4. 处理变量和不等式约束的上下界（将无穷大边界标记为不活跃）
 * 5. 调用 qp->makeData() 构造问题数据
 * 6. 创建变量存储(vars)、残差(resid)和求解器(s)对象
 * 7. 调用 solver->solve() 求解
 * 8. 检查求解状态并提取最优解
 *
 * ==================== 无穷大边界的处理 ====================
 *
 * 当上下界为 ±numeric_limits<double>::max() 时，
 * 使用 generateLimits 函数将其标记为"不活跃"(unused)，
 * 对应 useLowerLimit[i] = 0 或 useUpperLimit[i] = 0。
 *
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/solver/ooqp_interface.h"

#include <stdexcept>

#include "ooqp/QpGenData.h"
#include "ooqp/QpGenVars.h"
#include "ooqp/QpGenResiduals.h"
#include "ooqp/GondzioSolver.h"
#include "ooqp/QpGenSparseMa27.h"
#include "ooqp/Status.h"

namespace common {

using namespace Eigen;
using namespace std;

/*
 * OoQpItf::solve - 求解标准QP问题的主函数
 *
 * 这是OOQP求解器的核心接口函数。它接收标准形式的QP问题参数，
 * 设置并调用OOQP求解器，返回最优解。
 *
 * 参数说明：
 * @param Q 二次项Hessian矩阵 (n×n, 稀疏, 对称, 半正定)
 * @param c 线性项系数向量 (n维)
 * @param A 等式约束矩阵 (m_y × n, 稀疏)
 * @param b 等式约束右端向量 (m_y维)
 * @param C 不等式约束矩阵 (m_z × n, 稀疏)
 * @param d 不等式约束下界向量 (m_z维)
 * @param f 不等式约束上界向量 (m_z维)
 * @param l 变量下界向量 (n维)
 * @param u 变量上界向量 (n维)
 * @param x 输出参数，最优解向量 (n维)
 * @param ignoreUnknownError 是否忽略UNKNOWN状态（视为成功）
 * @param verbose 是否输出详细的求解信息
 * @return true 求解成功（SUCCESSFUL_TERMINATION 或 被忽略的UNKNOWN）
 */
bool OoQpItf::solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
                    const Eigen::VectorXd& c,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    const Eigen::VectorXd& b,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                    const Eigen::VectorXd& d, const Eigen::VectorXd& f,
                    const Eigen::VectorXd& l, const Eigen::VectorXd& u,
                    Eigen::VectorXd& x, const bool ignoreUnknownError,
                    const bool verbose) {
  int nx = Q.rows();  // 原始变量的数量
  x.setZero(nx);

  // 创建拷贝（因为OOQP会修改数据）
  auto ccopy(c);
  auto Acopy(A);
  auto bcopy(b);
  auto Ccopy(C);

  /*
   * 将Q转换为下三角形式。
   * OOQP要求Q矩阵为下三角形式（对称矩阵只需存储一半）。
   * 参考OOQP用户手册第2.2节（第11页）。
   */
  SparseMatrix<double, Eigen::RowMajor> Q_triangular =
      Q.triangularView<Lower>();

  if (verbose) {
    printProblemFormulation(Q_triangular, ccopy, Acopy, bcopy, Ccopy, d, f, l,
                            u);
  }

  // 压缩稀疏矩阵以提高计算效率（参考Eigen稀疏矩阵用户手册）
  Q_triangular.makeCompressed();
  Acopy.makeCompressed();
  Ccopy.makeCompressed();

  assert(Ccopy.rows() == d.size());
  assert(Ccopy.rows() == f.size());
  /*
   * 处理变量和约束的上下界。
   * 将无穷大边界标记为不活跃（useLowerLimit[i]/useUpperLimit[i] = 0）。
   * 参考OOQP用户手册第2.2节（第10页）。
   */
  Matrix<char, Eigen::Dynamic, 1> useLowerLimitForX;
  Matrix<char, Eigen::Dynamic, 1> useUpperLimitForX;
  VectorXd lowerLimitForX;
  VectorXd upperLimitForX;
  Matrix<char, Eigen::Dynamic, 1> useLowerLimitForInequalityConstraints;
  Matrix<char, Eigen::Dynamic, 1> useUpperLimitForInequalityConstraints;
  VectorXd lowerLimitForInequalityConstraints;
  VectorXd upperLimitForInequalityConstraints;

  generateLimits(l, u, useLowerLimitForX, useUpperLimitForX, lowerLimitForX,
                 upperLimitForX);
  generateLimits(d, f, useLowerLimitForInequalityConstraints,
                 useUpperLimitForInequalityConstraints,
                 lowerLimitForInequalityConstraints,
                 upperLimitForInequalityConstraints);

  if (verbose) {
    cout << "-------------------------------" << endl;
    cout << "LIMITS FOR X" << endl;
    printLimits(useLowerLimitForX, useUpperLimitForX, lowerLimitForX,
                upperLimitForX);
    cout << "-------------------------------" << endl;
    cout << "LIMITS FOR INEQUALITY CONSTRAINTS" << endl;
    printLimits(useLowerLimitForInequalityConstraints,
                useUpperLimitForInequalityConstraints,
                lowerLimitForInequalityConstraints,
                upperLimitForInequalityConstraints);
  }

  // ========== 设置OOQP求解器 ==========
  // 参考OOQP用户手册第2.3节（第14页）

  // 初始化问题规模
  int my = bcopy.size();       // 等式约束数量
  int mz = lowerLimitForInequalityConstraints.size();  // 不等式约束数量
  int nnzQ = Q_triangular.nonZeros();   // Q矩阵的非零元数
  int nnzA = Acopy.nonZeros();          // A矩阵的非零元数
  int nnzC = Ccopy.nonZeros();          // C矩阵的非零元数

  // 创建QP问题对象（使用MA27稀疏线性求解器）
  QpGenSparseMa27* qp = new QpGenSparseMa27(nx, my, mz, nnzQ, nnzA, nnzC);

  // 获取矩阵数据的原始指针
  double* cp = &ccopy.coeffRef(0);
  int* krowQ = Q_triangular.outerIndexPtr();     // Q的行偏移数组
  int* jcolQ = Q_triangular.innerIndexPtr();     // Q的列索引数组
  double* dQ = Q_triangular.valuePtr();          // Q的非零元值数组
  double* xlow = &lowerLimitForX.coeffRef(0);
  char* ixlow = &useLowerLimitForX.coeffRef(0);
  double* xupp = &upperLimitForX.coeffRef(0);
  char* ixupp = &useUpperLimitForX.coeffRef(0);
  int* krowA = Acopy.outerIndexPtr();
  int* jcolA = Acopy.innerIndexPtr();
  double* dA = Acopy.valuePtr();
  double* bA = &bcopy.coeffRef(0);
  int* krowC = Ccopy.outerIndexPtr();
  int* jcolC = Ccopy.innerIndexPtr();
  double* dC = Ccopy.valuePtr();
  double* clow = &lowerLimitForInequalityConstraints.coeffRef(0);
  char* iclow = &useLowerLimitForInequalityConstraints.coeffRef(0);
  double* cupp = &upperLimitForInequalityConstraints.coeffRef(0);
  char* icupp = &useUpperLimitForInequalityConstraints.coeffRef(0);

  // 构造QP问题数据
  QpGenData* prob = (QpGenData*)qp->makeData(
      cp, krowQ, jcolQ, dQ, xlow, ixlow, xupp, ixupp, krowA, jcolA, dA, bA,
      krowC, jcolC, dC, clow, iclow, cupp, icupp);

  // 创建变量存储对象
  QpGenVars* vars = (QpGenVars*)qp->makeVariables(prob);

  // 创建残差存储对象
  QpGenResiduals* resid = (QpGenResiduals*)qp->makeResiduals(prob);

  // 创建求解器对象（使用Gondzio迭代内点法）
  GondzioSolver* s = new GondzioSolver(qp, prob);

  if (verbose) {
    s->monitorSelf();  // 输出求解过程中的自监测信息
  }

  // ========== 求解 ==========
  int status = s->solve(prob, vars, resid);

  // 如果求解成功（或被允许忽略UNKNOWN状态），提取最优解
  if ((status == SUCCESSFUL_TERMINATION) ||
      (ignoreUnknownError && (status == UNKNOWN)))
    vars->x->copyIntoArray(&x.coeffRef(0));

  if (verbose) {
    printSolution(status, x);
  }

  // 清理内存
  delete s;
  delete resid;
  delete vars;
  delete prob;
  delete qp;

  return ((status == SUCCESSFUL_TERMINATION) ||
          (ignoreUnknownError && (status == UNKNOWN)));
}

/*
 * generateLimits - 生成变量/约束的上下界标记数组
 *
 * 将无穷大的边界（±numeric_limits<double>::max()）标记为不活跃，
 * 这意味着求解器不会对该变量施加对应的边界约束。
 *
 * @param l 下界向量
 * @param u 上界向量
 * @param useLowerLimit 输出参数，下界是否活跃的标记数组（1=活跃, 0=不活跃）
 * @param useUpperLimit 输出参数，上界是否活跃的标记数组
 * @param lowerLimit 输出参数，处理后的下界值（不活跃项设为0）
 * @param upperLimit 输出参数，处理后的上界值（不活跃项设为0）
 */
void OoQpItf::generateLimits(
    const Eigen::VectorXd& l, const Eigen::VectorXd& u,
    Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit,
    Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit,
    Eigen::VectorXd& lowerLimit, Eigen::VectorXd& upperLimit) {
  int n = l.size();
  useLowerLimit.setConstant(n, 1);
  useUpperLimit.setConstant(n, 1);
  lowerLimit = l;
  upperLimit = u;

  for (int i = 0; i < n; i++) {
    // 如果下界接近负无穷大，标记下界为不活跃
    if (NumericalUtil::ApproximatelyEqual(
            l(i), -std::numeric_limits<double>::max())) {
      useLowerLimit(i) = 0;
      lowerLimit(i) = 0.0;
    }
    // 如果上界接近正无穷大，标记上界为不活跃
    if (NumericalUtil::ApproximatelyEqual(u(i),
                                          std::numeric_limits<double>::max())) {
      useUpperLimit(i) = 0;
      upperLimit(i) = 0.0;
    }
  }
}

/*
 * printProblemFormulation - 输出QP问题的数学表述
 *
 * 输出标准形式: min 1/2 x'Qx + c'x, s.t. Ax=b, d<=Cx<=f, l<=x<=u
 * 包含所有矩阵和向量的详细内容。
 */
void OoQpItf::printProblemFormulation(
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
    const Eigen::VectorXd& c,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
    const Eigen::VectorXd& b,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
    const Eigen::VectorXd& d, const Eigen::VectorXd& f,
    const Eigen::VectorXd& l, const Eigen::VectorXd& u) {
  cout << "-------------------------------" << endl;
  cout << "Find x: min 1/2 x' Q x + c' x such that A x = b, d <= Cx <= f, and "
          "l <= x <= u"
       << endl
       << endl;
  cout << "Q (triangular) << " << endl << MatrixXd(Q) << endl;
  cout << "c << " << c.transpose() << endl;
  cout << "A << " << endl << MatrixXd(A) << endl;
  cout << "b << " << b.transpose() << endl;
  cout << "C << " << endl << MatrixXd(C) << endl;
  cout << "d << " << d.transpose() << endl;
  cout << "f << " << f.transpose() << endl;
  cout << "l << " << l.transpose() << endl;
  cout << "u << " << u.transpose() << endl;
}

/*
 * printLimits - 输出上下界信息
 */
void OoQpItf::printLimits(
    const Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit,
    const Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit,
    const Eigen::VectorXd& lowerLimit, const Eigen::VectorXd& upperLimit) {
  cout << "useLowerLimit << " << std::boolalpha
       << useLowerLimit.cast<bool>().transpose() << endl;
  cout << "lowerLimit << " << lowerLimit.transpose() << endl;
  cout << "useUpperLimit << " << std::boolalpha
       << useUpperLimit.cast<bool>().transpose() << endl;
  cout << "upperLimit << " << upperLimit.transpose() << endl;
}

/*
 * printSolution - 输出求解结果
 *
 * @param status 求解器返回的状态码（0=SUCCESSFUL_TERMINATION）
 * @param x 最优解向量
 */
void OoQpItf::printSolution(const int status, const Eigen::VectorXd& x) {
  if (status == 0) {
    cout << "-------------------------------" << endl;
    cout << "SOLUTION" << endl;
    cout << "Ok, ended with status " << status << "." << endl;
    cout << "x << " << x.transpose() << endl;
  } else {
    cout << "-------------------------------" << endl;
    cout << "SOLUTION" << endl;
    cout << "Error, ended with status " << status << "." << endl;
  }
}

}  // namespace common
