#include "setting_menu.h"
#include "ui_setting_menu.h"
#include "euclideanCluster.h"
#include <nlohmann/json.hpp>
#include <iostream>
#include "AlgorithmManager.h"



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
    auto cloud = m_impl->algo_mgr.run("downsample", config);
    updateViewerWithCloud(cloud, {0.0, 1.0, 0.0});


    // try {

    //     if (!m_impl || !m_impl->cloud || !viewer_) {
    //         std::cerr << "Error: Required objects not initialized" << std::endl;
    //         return;
    //     }

    //     if (m_impl->cloud->empty()) {
    //         std::cerr << "Warning: Input point cloud is empty" << std::endl;
    //         return;
    //     }

    //     auto downsample_algo = std::make_unique<Downsample>();

    //     nlohmann::json config;
    //     config["voxel_size"] = 0.01;

    //     if (!downsample_algo->initialize(config)) {
    //         std::cerr << "Error: Failed to initialize downsample algorithm" << std::endl;
    //         return;
    //     }

    //     m_impl->cloud_out->clear();
    //     if (!downsample_algo->process(m_impl->cloud, m_impl->cloud_out)) {
    //         std::cerr << "Error: Failed to process downsample" << std::endl;
    //         return;
    //     }

    //    updateViewerWithCloud(m_impl->cloud_out, {0.0, 1.0, 0.0});

    // } catch (const std::exception& e) {
    //     std::cerr << "Exception in on_pushButton_4_clicked: " << e.what() << std::endl;
    // } catch (...) {
    //     std::cerr << "Unknown exception in on_pushButton_4_clicked" << std::endl;
    // }
}

