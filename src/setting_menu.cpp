#include "setting_menu.h"
#include "ui_setting_menu.h"
#include "euclideanCluster.h"
#include <nlohmann/json.hpp>
#include <iostream>
#include "algorithm_manager.h"

// #include "algorithm_registry.h"

struct settingMenu::Impl
{
    algorithm_pipeline::AlgorithmManager algo_mgr;
    pcl::PointCloud<PointT>::Ptr cloud{new pcl::PointCloud<PointT>};
    pcl::PointCloud<PointT>::Ptr cloud_out{new pcl::PointCloud<PointT>};
    EuclideanCluster m_cluster;
    Impl() {}

    pcl::PointCloud<PointT>::Ptr process()
    {
        cloud_out->clear();
        m_cluster.clusterObj(cloud,cloud_out);
        return cloud_out;
    }

};

settingMenu::settingMenu(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::settingMenu)
    ,m_impl(std::make_shared<Impl>())
{
    ui->setupUi(this);
    spdlog::set_level(spdlog::level::info);

}

void settingMenu::handleCloud(const PointCloudConstPtr& cloud) {
    if (cloud) {
        // std::cout << "Received point cloud with " << cloud->size() << " points" << std::endl;
        *m_impl->cloud = *cloud;
        m_impl->algo_mgr.setInputCloud(m_impl->cloud);

    } else {
        std::cerr << "Received empty point cloud" << std::endl;
    }
}

void settingMenu::registerUpdateViewerCallback(std::function<void()> callback)
{
    update_viewer = callback;
}

settingMenu::~settingMenu()
{
    delete ui;
}

void settingMenu::updateViewerWithCloud(const PointCloudPtr& cloud, const std::array<double,3>& color) {
    if (!cloud || cloud->empty()) return;
    if (viewer_->contains(cloudId)) viewer_->removePointCloud(cloudId);
    viewer_->addPointCloud(cloud, cloudId);
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR,
        color[0], color[1], color[2], cloudId);
    if (update_viewer) update_viewer();
}

void settingMenu::on_pushButton_clicked()
{
    auto cloud = m_impl->process();

    updateViewerWithCloud(cloud, {1.0, 0.0, 0.0});
}


void settingMenu::on_pushButton_2_clicked()
{
    updateViewerWithCloud(m_impl->cloud, {1.0, 0.0, 0.0});
}


void settingMenu::on_pushButton_3_clicked()
{
    viewer_->removeAllShapes();
     updateViewerWithCloud(m_impl->cloud, {1.0, 0.0, 0.0});
}


void settingMenu::on_pushButton_4_clicked()
{

    nlohmann::json config;
    config["voxel_size"] = 0.01;  // UI 设置
    // auto cloud = m_impl->algo_mgr.run("downsample", config);

     m_impl->algo_mgr.loadLibraries({
        "./algorithmTool/libdownsample.so",
        "./algorithmTool/libfilter.so",
        "./algorithmTool/libsegmentation.so"
    });


     algorithm_pipeline::PipelineStep step1;
     step1.branches.push_back({"downsample", {{"voxel_size", 0.1}}});
     step1.branches.push_back({"filter", {{"threshold", 0.2}}});
     // step1.mergeFunc = 自定义合并函数（可选）
     m_impl->algo_mgr.addStep(step1);

     algorithm_pipeline::PipelineStep step2;
     step2.branches.push_back({"segmentation", {{"method", "RANSAC"}}});
     m_impl->algo_mgr.addStep(step2);

     m_impl->algo_mgr.printPipeline();

     auto result = m_impl->algo_mgr.runPipeline();
     spdlog::info("Final output points: {}", result ? result->size() : 0);


    updateViewerWithCloud(result, {0.0, 1.0, 0.0});

}

