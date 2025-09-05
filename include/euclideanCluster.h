#ifndef EUCLIDEANCLUSTER_H
#define EUCLIDEANCLUSTER_H
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>



class EuclideanCluster
{
public:
    using PointT = pcl::PointXYZ;
    pcl::search::KdTree<PointT>::Ptr tree{new pcl::search::KdTree<PointT>};
    pcl::PointCloud<PointT>::Ptr cluster_in{new pcl::PointCloud<PointT>};
    pcl::PointCloud<PointT>::Ptr cluster_out{new pcl::PointCloud<PointT>};
    std::vector<std::vector<PointT>> cluster_vec_;
    EuclideanCluster()
    {}

    void clusterObj(const  pcl::PointCloud<PointT>::Ptr cloud, std::vector<std::vector<PointT>>& cluster_vec)
    {
        if(cloud->points.empty()) return;
        tree->setInputCloud(cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(0.04);
        ec.setMinClusterSize(30);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        cluster_vec.clear();
        for (const auto& cluster : cluster_indices)
        {
            std::vector<PointT> clusterPoints;
            clusterPoints.reserve(cluster.indices.size());
            for (const auto& idx : cluster.indices)
                clusterPoints.push_back((*cloud)[idx]);
            cluster_vec.push_back(clusterPoints);
        }

    }

    void clusterObj(const  pcl::PointCloud<PointT>::Ptr cloud,const  pcl::PointCloud<PointT>::Ptr& cluster_)
    {
        if(cloud->points.empty()) return;
        tree->setInputCloud(cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(0.04);
        ec.setMinClusterSize(30);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        cluster_->points.clear();
        for (const auto& cluster : cluster_indices)
        {
            for (const auto& idx : cluster.indices)
                cluster_->points.push_back((*cloud)[idx]);

        }
    }



};






#endif // EUCLIDEANCLUSTER_H
