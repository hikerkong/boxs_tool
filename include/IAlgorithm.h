#ifndef IALGORITHM_H
#define IALGORITHM_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <nlohmann/json.hpp>
#include <string>

using PointT = pcl::PointXYZ;
using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;

class IAlgorithm {
public:
    virtual ~IAlgorithm() = default;
    virtual bool initialize(const nlohmann::json& config) = 0;
    virtual bool process(const PointCloudPtr& input, PointCloudPtr& output) = 0;
    virtual std::string name() const = 0;
};


#endif // IALGORITHM_H
