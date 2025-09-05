
#ifndef BOXWIDGET_H
#define BOXWIDGET_H

#include <QWidget>
#include <QMap>
#include <QObject> 
#include <QString>
#include <QPointer>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkSmartPointer.h>
#include <vtkCamera.h>
#include <QListWidget>
#include <pcl/kdtree/kdtree_flann.h>
#include "CustomInteractorStyle.h"
// #include "box_handler.h"

#include "box_storage.h"

using PointT = pcl::PointXYZ;

class QVTKOpenGLWidget;
class vtkGenericOpenGLRenderWindow;
class vtkActor;

// struct BoxParams {
//     double cx, cy, cz;
//     double sx, sy, sz;
//     double yawDeg;

//     friend QDataStream &operator<<(QDataStream &out, const BoxParams &params) {
//         out << params.cx << params.cy << params.cz;
//         out << params.sx << params.sy << params.sz;
//         out << params.yawDeg;
//         return out;
//     }


//     friend QDataStream &operator>>(QDataStream &in, BoxParams &params) {
//         in >> params.cx >> params.cy >> params.cz;
//         in >> params.sx >> params.sy >> params.sz;
//         in >> params.yawDeg;
//         return in;
//     }
// };

class BoxWidget : public QWidget {
    Q_OBJECT
public:
    explicit BoxWidget(QWidget* parent = nullptr);
    void loadPCD(const QString& path);
    void updateCloud(const pcl::PointCloud<PointT>::ConstPtr cloud);
    void addBoxByScreenPos(int x, int y);
    void addBoxAtViewCenter();
    void selectByScreenPos(int x, int y);
    void deleteSelectedBox();

    // Parameters
    std::string currentSelectionId() const { return selectedId_; }
    BoxParams getBoxParams(const QString& id) const;
    void updateBoxParams(const QString& id, float cx, float cy, float cz,
                         float sx, float sy, float sz, float yawDeg);
    void onPointPickedWithCtrl(int x, int y);
    QString getDefaultSavePath();
signals:
    void selectionChanged();

private slots:
    void onBoxListItemClicked(QListWidgetItem* item);

private:
    void updateBoxsPoints(const BoxItem& item);
    vtkSmartPointer<CustomInteractorStyle> style;
    pcl::KdTreeFLANN<PointT> kdtree;
    pcl::PointCloud<PointT>::Ptr cloud_{new pcl::PointCloud<PointT>};
    pcl::PointCloud<PointT>::Ptr temp_{new pcl::PointCloud<PointT>};


    void ensureDemoCloud();
    void updateBoxActor(BoxItem& item);
    QString genNextId() const;
    void setSelected(const QString& id);

    QVTKOpenGLWidget* vtkWidget_ = nullptr;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow_;
    pcl::visualization::PCLVisualizer::Ptr viewer_;

    QMap<QString, BoxItem> boxes_;
    std::string selectedId_;
    std::unordered_map<std::string,std::string> edgeToBox_;
    QListWidget* boxList_;
    std::vector<PointT> pickedPoints_;
    // BoxHandler boxHandler;
    BoxStorage storage{"./setting/boxes.json"};

};

#endif // BOXWIDGET_H
