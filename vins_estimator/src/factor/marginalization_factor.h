#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <pthread.h>
#include <ceres/ceres.h>
#include <unordered_map>

#include "../utility/utility.h"
#include "../utility/tic_toc.h"

const int NUM_THREADS = 4;

//观测项
struct ResidualBlockInfo
{
    // 构造函数需要，cost function（约束），loss function：残差的计算方式，相关联的参数块，待边缘化的参数块的索引
    ResidualBlockInfo(ceres::CostFunction *_cost_function, ceres::LossFunction *_loss_function, std::vector<double *> _parameter_blocks, std::vector<int> _drop_set)
        : cost_function(_cost_function), loss_function(_loss_function), parameter_blocks(_parameter_blocks), drop_set(_drop_set) {}

    void Evaluate();

    ceres::CostFunction *cost_function;
    ceres::LossFunction *loss_function;
    std::vector<double *> parameter_blocks;//优化变量数据, 和该约束相关的参数块
    std::vector<int> drop_set;//待边缘化的优化变量id

    double **raw_jacobians;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
    Eigen::VectorXd residuals;//残差 IMU:15X1 视觉2X1

    // 7维pose（四元数）则返回6
    int localSize(int size)
    {
        return size == 7 ? 6 : size;
    }
};

struct ThreadsStruct
{
    std::vector<ResidualBlockInfo *> sub_factors;
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    std::unordered_map<long, int> parameter_block_size; //global size, 此参数块的维度
    std::unordered_map<long, int> parameter_block_idx; //local size, 在大矩阵中的id，也就是落座的位置
};

class MarginalizationInfo
{
  public:
    ~MarginalizationInfo();
    int localSize(int size) const;
    int globalSize(int size) const;
    //添加残差块的信息（优化变量，待marg的变量）
    void addResidualBlockInfo(ResidualBlockInfo *residual_block_info);
    //计算每个残差对应的雅可比，并更新parameter_block_data
    void preMarginalize();
    //pos为所有变量的维度，m是需要margin掉的变量维度，n是需要保留的变量维度
    void marginalize();
    std::vector<double *> getParameterBlocks(std::unordered_map<long, double *> &addr_shift);

    std::vector<ResidualBlockInfo *> factors;//所有观测项
    //m为要marg掉的变量个数（参数块总大小），也就是parameter_block_idx的总localSize，以double为单位，VBias为9，PQ为6
    //n为要保留下来的优化变量的变量个数， n = localSize(parameter_block_size) - m
    int m, n; //m：待边缘化的总维度总大小（不是个数）， n:需要保留的总维度
    std::unordered_map<long, int> parameter_block_size; //global size <优化变量内存地址,localSize>，参数块的global维度，数据块变量的长度
    int sum_block_size;
    // <内存地址,在parameter_block_size中的id> 参数块在H矩阵中的索引,需要将marg的变量放前面
    // 先后用作待边缘化的优化变量参数块，所有的参数块
    std::unordered_map<long, int> parameter_block_idx; //local size
    std::unordered_map<long, double *> parameter_block_data;//<优化变量内存地址,数据>，参数块的数据

    std::vector<int> keep_block_size; //global size  表示上一次边缘化留下来的各个参数块的长度
    std::vector<int> keep_block_idx;  //local size ,此参数块的id
    std::vector<double *> keep_block_data;

    Eigen::MatrixXd linearized_jacobians; //从信息矩阵恢复出来的雅可比，用于更新残差
    Eigen::VectorXd linearized_residuals; //从信息矩阵恢复出来的残差
    const double eps = 1e-8;

};

// 由于边缘化的costfuntion不是固定大小的，因此只能继承最基本的类
class MarginalizationFactor : public ceres::CostFunction
{
  public:
    MarginalizationFactor(MarginalizationInfo* _marginalization_info);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    MarginalizationInfo* marginalization_info;
};
