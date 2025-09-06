#include "Downsample.h"
#include "AlgorithmRegistry.h"

namespace algorithm_pipeline::algorithms {

bool Downsample::initialize(const nlohmann::json& config) {
    config_ = config;
    initializeLogger();

    if (config.contains("voxel_size_x")) voxel_size_x_ = config["voxel_size_x"].get<double>();
    if (config.contains("voxel_size_y")) voxel_size_y_ = config["voxel_size_y"].get<double>();
    if (config.contains("voxel_size_z")) voxel_size_z_ = config["voxel_size_z"].get<double>();

    voxel_filter_.setLeafSize(voxel_size_x_, voxel_size_y_, voxel_size_z_);
    logger_->info("Downsample initialized: voxel sizes = {}, {}, {}",
                  voxel_size_x_, voxel_size_y_, voxel_size_z_);
    return true;
}

bool Downsample::process(const PointCloudPtr& input, PointCloudPtr& output) {
    if (!input || input->empty()) {
        logger_->warn("Input cloud is empty.");
        return false;
    }

    output.reset(new PointCloud);
    voxel_filter_.setInputCloud(input);
    voxel_filter_.filter(*output);

    if (debug_mode_) {
        logger_->info("Downsample processed: input size = {}, output size = {}",
                      input->size(), output->size());
    }
    return true;
}

REGISTER_ALGORITHM("downsample", Downsample);

} // namespace algorithm_pipeline::algorithms



