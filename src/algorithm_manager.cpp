// AlgorithmManager.cpp
#include "algorithm_manager.h"
#include <dlfcn.h>
#include <spdlog/spdlog.h>
#include <future>

namespace algorithm_pipeline {

AlgorithmManager::AlgorithmManager() {}
AlgorithmManager::~AlgorithmManager() {
    for (auto handle : handles_) {
        if (handle) dlclose(handle);
    }
    handles_.clear();
}

bool AlgorithmManager::loadLibrary(const std::string& path) {
    void* handle = dlopen(path.c_str(), RTLD_NOW | RTLD_GLOBAL);
    if (!handle) {
        spdlog::error("无法加载 {}: {}", path, dlerror());
        return false;
    }
    spdlog::info("成功加载库: {}", path);
    handles_.push_back(handle);
    return true;
}

void AlgorithmManager::loadLibraries(const std::vector<std::string>& paths) {
    for (const auto& path : paths) loadLibrary(path);
}

PointCloudPtr AlgorithmManager::run(const std::string& algoName, const nlohmann::json& config) {
    auto algo = AlgorithmRegistry::instance().create(algoName);
    if (!algo) {
        spdlog::error("Algorithm {} not found", algoName);
        return nullptr;
    }
    if (!algo->initialize(config)) {
        spdlog::error("Failed to initialize {}", algoName);
        return nullptr;
    }
    PointCloudPtr output(new PointCloud);
    if (!algo->process(input_cloud_, output)) {
        spdlog::error("Failed to process {}", algoName);
        return nullptr;
    }
    return output;
}

// ========== 管道管理 ==========

void AlgorithmManager::addStep(const PipelineStep& step) {
    pipeline_.push_back(step);
    spdlog::info("Added pipeline step with {} branches", step.branches.size());
}

void AlgorithmManager::clearPipeline() {
    pipeline_.clear();
    spdlog::info("Pipeline cleared");
}

PointCloudPtr AlgorithmManager::runPipeline() {
    PointCloudPtr current = input_cloud_;

    for (size_t i = 0; i < pipeline_.size(); ++i) {
        const auto& step = pipeline_[i];
        std::vector<std::future<PointCloudPtr>> futures;
        std::vector<PointCloudPtr> branchOutputs;

        // 并行执行每个分支
        for (const auto& branch : step.branches) {
            futures.push_back(std::async(std::launch::async, [this, branch, current]() {
                auto algo = AlgorithmRegistry::instance().create(branch.name);
                if (!algo) {
                    spdlog::error("Branch algorithm {} not found", branch.name);
                    return PointCloudPtr(nullptr);
                }
                if (!algo->initialize(branch.config)) {
                    spdlog::error("Failed to initialize branch {}", branch.name);
                    return PointCloudPtr(nullptr);
                }
                PointCloudPtr output(new PointCloud);
                if (!algo->process(current, output)) {
                    spdlog::error("Failed to process branch {}", branch.name);
                    return PointCloudPtr(nullptr);
                }
                return output;
            }));
        }

        // 收集分支结果
        for (auto& fut : futures) {
            auto res = fut.get();
            if (res) branchOutputs.push_back(res);
        }

        // 合并分支结果
        if (step.mergeFunc) {
            current = step.mergeFunc(branchOutputs);
        } else {
            // 默认合并: 拼接点云
            PointCloudPtr merged(new PointCloud);
            for (auto& b : branchOutputs) {
                *merged += *b;  // 假设 PointCloud 支持 +=
            }
            current = merged;
        }

        spdlog::info("Step {} completed, {} branches merged", i, step.branches.size());
    }

    return current;
}

void AlgorithmManager::printPipeline() const {
    spdlog::info("Pipeline [{} steps]:", pipeline_.size());
    for (size_t i = 0; i < pipeline_.size(); ++i) {
        spdlog::info("Step [{}] with {} branches:", i, pipeline_[i].branches.size());
        for (const auto& branch : pipeline_[i].branches) {
            spdlog::info("  - {}", branch.name);
        }
    }
}

} // namespace algorithm_pipeline
