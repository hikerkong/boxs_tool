#ifndef ALGORITHMMANAGER_H
#define ALGORITHMMANAGER_H

#include "algorithm_registry.h"
#include <vector>
#include <string>
#include <unordered_map>
#include <functional>

namespace algorithm_pipeline {

// 一个分支算法
struct BranchStep {
    std::string name;
    nlohmann::json config;
};

// 流水线步骤（可以是单算法或多个分支）
struct PipelineStep {
    std::vector<BranchStep> branches;  // 支持多个分支并行
    // 合并函数: 将多个分支结果合并为一个输出
    std::function<PointCloudPtr(const std::vector<PointCloudPtr>&)> mergeFunc;
};

class AlgorithmManager {
public:
    AlgorithmManager();
    ~AlgorithmManager();

    // 动态库加载
    bool loadLibrary(const std::string& path);
    void loadLibraries(const std::vector<std::string>& paths);

    void setInputCloud(const PointCloudPtr& cloud) { input_cloud_ = cloud; }

    // 单算法运行
    PointCloudPtr run(const std::string& algoName, const nlohmann::json& config = {});

    // 流水线管理
    void addStep(const PipelineStep& step);
    void clearPipeline();

    // 执行整个管道（支持分支并行）
    PointCloudPtr runPipeline();

    void printPipeline() const;

private:
    PointCloudPtr input_cloud_;
    std::vector<void*> handles_;
    std::vector<PipelineStep> pipeline_;
};

} // namespace algorithm_pipeline

#endif // ALGORITHMMANAGER_H








