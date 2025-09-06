#pragma once

#include "algorithm_base.h"
#include <pcl/filters/voxel_grid.h>


namespace algorithm_pipeline::algorithms {

/**
 * @brief Voxel grid downsampling algorithm
 */
class Downsample : public AlgorithmBase {
public:
    Downsample() : AlgorithmBase("downsample") {}

    bool initialize(const nlohmann::json& config) override;
    bool process(const PointCloudPtr& input, PointCloudPtr& output) override;

private:
    double voxel_size_x_ = 0.01;
    double voxel_size_y_ = 0.01;
    double voxel_size_z_ = 0.01;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
};

} // namespace algorithm_pipeline::algorithms


