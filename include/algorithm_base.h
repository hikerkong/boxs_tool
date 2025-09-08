#pragma once

#include <memory>
#include <string>
#include <functional>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace algorithm_pipeline {

using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

/**
 * @brief Base class for all algorithms in the pipeline
 */
class AlgorithmBase {
public:
    AlgorithmBase(const std::string& name) : name_(name) {}
    virtual ~AlgorithmBase() = default;

    /**
     * @brief Initialize algorithm with configuration
     * @param config JSON configuration object
     * @return true if initialization successful
     */
    virtual bool initialize(const nlohmann::json& config) = 0;

    /**
     * @brief Process point cloud data
     * @param input Input point cloud
     * @param output Output point cloud
     * @return true if processing successful
     */
    virtual bool process(const PointCloudPtr& input, PointCloudPtr& output) = 0;

    /**
     * @brief Get algorithm name
     */
    const std::string& getName() const { return name_; }

    /**
     * @brief Get algorithm configuration
     */
    const nlohmann::json& getConfig() const { return config_; }

    /**
     * @brief Set debug mode
     */
    void setDebugMode(bool debug) { debug_mode_ = debug; }

    /**
     * @brief Check if in debug mode
     */
    bool isDebugMode() const { return debug_mode_; }

protected:
    std::string name_;
    nlohmann::json config_;
    bool debug_mode_ = false;
    std::shared_ptr<spdlog::logger> logger_;

    /**
     * @brief Initialize logger for this algorithm
     */
    void initializeLogger() {
        try {
            logger_ = spdlog::get(name_);
            if (!logger_) {
                logger_ = spdlog::stdout_color_mt(name_);
            }
        } catch (const std::exception& e) {
            // Fallback: create a simple logger if spdlog fails
            std::cerr << "Warning: Failed to initialize spdlog logger for " << name_
                      << ": " << e.what() << ". Using fallback logger." << std::endl;
            logger_ = spdlog::stdout_color_mt(name_ + "_fallback");
        }
    }
};

/**
 * @brief Factory function type for creating algorithms
 */
using AlgorithmFactory = std::function<std::unique_ptr<AlgorithmBase>()>;

} // namespace algorithm_pipeline
