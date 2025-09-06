#pragma once
#include <QWidget>
#include <memory>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <functional>

#include <QWidget>
#include <QVBoxLayout>
#include <QMouseEvent>
#include <QMap>
#include <memory>


#include <pcl/visualization/pcl_visualizer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "BoundingBox.h"



using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;


namespace pcl { namespace visualization { class PCLVisualizer; } }
class QVTKOpenGLNativeWidget;
class vtkGenericOpenGLRenderWindow;

class PCLViewerWidget : public QWidget
{
    Q_OBJECT
public:
    using CallbackFunction = std::function<void(const PointCloudConstPtr&)>;
    explicit PCLViewerWidget(QWidget* parent = nullptr);
    void loadPCD(const QString& path);
    void clear();
    pcl::PointCloud<PointT>::ConstPtr getCloud() const;
    void registerCallback(CallbackFunction callback);
    void updateViewer();
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
signals:
    void pointPicked(float x, float y, float z);

protected:
    bool eventFilter(QObject* obj, QEvent* ev) override;

private:
    pcl::KdTreeFLANN<PointT> kdtree;
    void showCloud();
    bool pickPointAt(int x, int y, float& px, float& py, float& pz);

    QVTKOpenGLNativeWidget* qvtk_ = nullptr;

    vtkGenericOpenGLRenderWindow* render_window_ = nullptr;

    pcl::PointCloud<PointT>::Ptr cloud_;
    QString current_path_;
    bool has_cloud_ = false;
    std::vector<CallbackFunction> cloud_callbacks_;
    bool viewer_initialized_;
    bool interactor_initialized_{false};



};
