#ifndef ALGORITHMMANAGER_H
#define ALGORITHMMANAGER_H


#include "AlgorithmRegistry.h"

namespace algorithm_pipeline {

class AlgorithmManager {
public:
    void setInputCloud(const PointCloudPtr& cloud) { input_cloud_ = cloud; }

    PointCloudPtr run(const std::string& algoName, const nlohmann::json& config = {}) {
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

private:
    PointCloudPtr input_cloud_;
};

} // namespace algorithm_pipeline






#endif // ALGORITHMMANAGER_H
