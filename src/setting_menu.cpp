#include "setting_menu.h"
#include "ui_setting_menu.h"
#include "euclideanCluster.h"


struct settingMenu::Impl
{
    pcl::PointCloud<PointT>::Ptr cloud{new pcl::PointCloud<PointT>};
    pcl::PointCloud<PointT>::Ptr cloud_{new pcl::PointCloud<PointT>};
    EuclideanCluster m_cluster;
    Impl() {}

    pcl::PointCloud<PointT>::Ptr process()
    {
        m_cluster.clusterObj(cloud,cloud_);
        return cloud_;
    }

};

settingMenu::settingMenu(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::settingMenu)
    ,m_impl(std::make_shared<Impl>())
{
    ui->setupUi(this);


}

void settingMenu::handleCloud(const PointCloudConstPtr& cloud) {
    if (cloud) {
        // std::cout << "Received point cloud with " << cloud->size() << " points" << std::endl;
        m_impl->cloud->clear();
        m_impl->cloud->reserve(cloud->points.size());
        m_impl->cloud->insert(m_impl->cloud->end(),cloud->begin(),cloud->end());

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

void settingMenu::on_pushButton_clicked()
{
    auto cloud = m_impl->process();

    if (viewer_->contains(cloudId)) {
        viewer_->removePointCloud(cloudId);
    }
    viewer_->addPointCloud(cloud, cloudId);
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud");
    update_viewer();
}


void settingMenu::on_pushButton_2_clicked()
{
    if (viewer_->contains(cloudId)) {
        viewer_->removePointCloud(cloudId);
    }
    viewer_->addPointCloud( m_impl->cloud, cloudId);
    update_viewer();
}


void settingMenu::on_pushButton_3_clicked()
{
    viewer_->removeAllShapes();
    update_viewer();
}

