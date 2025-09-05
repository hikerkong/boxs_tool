#ifndef SETTING_MENU_H
#define SETTING_MENU_H

#include <QWidget>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QDebug>
#include <memory>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <memory>
#include <functional>
using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;

namespace Ui {
class settingMenu;
}

class settingMenu : public QWidget
{
    Q_OBJECT

public:
    using CallbackFunction = std::function<void(const PointCloudConstPtr&)>;
    explicit settingMenu(QWidget *parent = nullptr);
    ~settingMenu();
    void handleCloud(const PointCloudConstPtr& cloud);
    void registerUpdateViewerCallback( std::function<void()> callback);
public:
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_{nullptr};
private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

private:
    Ui::settingMenu *ui;
    QTabWidget* tabWidget_;
    struct Impl;
    std::shared_ptr<Impl> m_impl;
    std::function<void()> update_viewer;
    std::string cloudId = "cloud";
};

#endif // SETTING_MENU_H
