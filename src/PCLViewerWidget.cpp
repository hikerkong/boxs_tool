#include "PCLViewerWidget.h"

#include <QVBoxLayout>
#include <QEvent>
#include <QMouseEvent>
#include <QApplication>

#include <QVTKOpenGLNativeWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkPointPicker.h>
#include <vtkSmartPointer.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include "BoundingBox.h"


PCLViewerWidget::PCLViewerWidget(QWidget* parent)
    : QWidget(parent)
    , cloud_(new pcl::PointCloud<PointT>)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0,0,0,0);

    qvtk_ = new QVTKOpenGLNativeWidget(this);
    layout->addWidget(qvtk_);

    render_window_ = vtkGenericOpenGLRenderWindow::New();
    qvtk_->SetRenderWindow(render_window_);

    viewer_.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    viewer_->setBackgroundColor(0.1, 0.1, 0.12);
    viewer_->addCoordinateSystem(0.5, "axis");
    // viewer_->initCameraParameters();


    qvtk_->GetRenderWindow()->AddRenderer(viewer_->getRendererCollection()->GetFirstRenderer());
    qvtk_->setFocusPolicy(Qt::StrongFocus);
    qvtk_->installEventFilter(this);


}

void PCLViewerWidget::loadPCD(const QString& path)
{
    pcl::PointCloud<PointT> tmp;
    if (pcl::io::loadPCDFile(path.toStdString(), tmp) != 0) {
        clear();
        return;
    }
    *cloud_ = tmp;
    if (cloud_) {
        for (const auto& cb : cloud_callbacks_)
        {
            cb(cloud_);
        }
    }
    current_path_ = path;
    has_cloud_ = true;
    showCloud();
}

void PCLViewerWidget::registerCallback(CallbackFunction callback)
{
     cloud_callbacks_.push_back(callback);
}

void PCLViewerWidget::updateViewer()
{
     qvtk_->GetRenderWindow()->Render();
}


pcl::PointCloud<PointT>::ConstPtr PCLViewerWidget::getCloud() const
{
    return cloud_;
}

void PCLViewerWidget::clear()
{
    has_cloud_ = false;
    cloud_->clear();
    viewer_->removeAllPointClouds();
    viewer_->removeAllShapes();
    qvtk_->GetRenderWindow()->Render();
}

void PCLViewerWidget::showCloud()
{
    viewer_->removeAllPointClouds();
    viewer_->removeAllShapes();

    pcl::visualization::PointCloudColorHandlerCustom<PointT> color(cloud_, 180, 220, 255);
    viewer_->addPointCloud<PointT>(cloud_, color, "cloud");
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    viewer_->resetCamera();
    qvtk_->GetRenderWindow()->Render();
}

// 捕获 Ctrl + 左键：进行拾取
bool PCLViewerWidget::eventFilter(QObject* obj, QEvent* ev)
{
    if (obj == qvtk_ && ev->type() == QEvent::MouseButtonPress) {
        auto* me = static_cast<QMouseEvent*>(ev);
        if ((me->button() == Qt::LeftButton) &&
            (QApplication::keyboardModifiers() & Qt::ControlModifier) &&
            has_cloud_ && !cloud_->empty())
        {
            float x, y, z;
            if (pickPointAt(me->pos().x(), me->pos().y(), x, y, z)) {
                // 在视图里画一个小球标记（先清除旧标记）
                viewer_->removeShape("picked_sphere");
                viewer_->addSphere(PointT(x,y,z), 0.02, 1.0, 0.3, 0.3, "picked_sphere");
                qvtk_->GetRenderWindow()->Render();  // 修改此处

                emit pointPicked(x,y,z);
                return true; // 事件已处理
            }
        }


    }
    return QWidget::eventFilter(obj, ev);
}

// 使用 VTK 的 PointPicker + PCL KDTree，得到最近点坐标
bool PCLViewerWidget::pickPointAt(int x, int y, float& px, float& py, float& pz)
{
    // VTK 的坐标原点在左下角，Qt 事件是左上角，需要换算
    int* size = qvtk_->GetRenderWindow()->GetSize();  // 修改此处
    int pickX = x;
    int pickY = size[1] - y;

    vtkSmartPointer<vtkPointPicker> picker = vtkSmartPointer<vtkPointPicker>::New();
    picker->SetTolerance(0.01); // 可按点密度调
    bool ok = picker->Pick(pickX, pickY, 0, viewer_->getRendererCollection()->GetFirstRenderer());
    if (!ok) return false;

    double pos[3]; picker->GetPickPosition(pos);

    // 用 KDTree 在点云中找最近点，避免 VTK/Actor 映射误差

    kdtree.setInputCloud(cloud_);
    PointT searchPoint(pos[0], pos[1], pos[2]);

    std::vector<int> idx(1);
    std::vector<float> dist(1);
    if (kdtree.nearestKSearch(searchPoint, 1, idx, dist) > 0) {
        const auto& p = cloud_->at(idx[0]);
        px = p.x; py = p.y; pz = p.z;
        return true;
    }
    return false;
}
