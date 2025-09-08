#include "downsample.h"
#include "algorithm_registry.h"
#include <iostream>




namespace algorithm_pipeline {
namespace algorithms {

bool Downsample::initialize(const nlohmann::json& config) {
    config_ = config;
    initializeLogger();

    try {
        if (config.contains("voxel_size")) {
            if (config["voxel_size"].is_number()) {
                // Single value for all dimensions
                double size = config["voxel_size"].get<double>();
                voxel_size_x_ = voxel_size_y_ = voxel_size_z_ = size;
            } else if (config["voxel_size"].is_array() && config["voxel_size"].size() == 3) {
                // Separate values for x, y, z
                voxel_size_x_ = config["voxel_size"][0].get<double>();
                voxel_size_y_ = config["voxel_size"][1].get<double>();
                voxel_size_z_ = config["voxel_size"][2].get<double>();
            }
        }

        if (config.contains("voxel_size_x")) {
            voxel_size_x_ = config["voxel_size_x"].get<double>();
        }
        if (config.contains("voxel_size_y")) {
            voxel_size_y_ = config["voxel_size_y"].get<double>();
        }
        if (config.contains("voxel_size_z")) {
            voxel_size_z_ = config["voxel_size_z"].get<double>();
        }

        logger_->info("Downsample algorithm initialized with voxel size: ({}, {}, {})",
                      voxel_size_x_, voxel_size_y_, voxel_size_z_);

        return true;
    } catch (const std::exception& e) {
        logger_->error("Failed to initialize downsample algorithm: {}", e.what());
        return false;
    }
}

bool Downsample::process(const PointCloudPtr& input, PointCloudPtr& output) {
    if (!input || input->empty()) {
        logger_->warn("Input point cloud is empty");
        output = std::make_shared<PointCloud>();
        return true;
    }

    try {
        auto start_time = std::chrono::high_resolution_clock::now();

        voxel_filter_.setInputCloud(input);
        voxel_filter_.setLeafSize(voxel_size_x_, voxel_size_y_, voxel_size_z_);

        output = std::make_shared<PointCloud>();
        voxel_filter_.filter(*output);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        logger_->info("Downsampled {} points to {} points in {} ms",
                      input->size(), output->size(), duration.count());

        return true;
    } catch (const std::exception& e) {
        logger_->error("Failed to process downsample: {}", e.what());
        return false;
    }
}



} // namespace algorithms
} // namespace algorithm_pipeline

namespace {
struct DownsampleRegister {
    DownsampleRegister() {
        AlgorithmRegistry::instance().register_algorithm("downsample", [](){
            // Downsample::run();
             std::cout << "运行 Downsample 算法" << std::endl;
             return std::make_unique<algorithm_pipeline::algorithms::Downsample>();
        });
    }
};
DownsampleRegister reg;
}
