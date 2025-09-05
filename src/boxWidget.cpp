
#include "boxWidget.h"
#include <QVBoxLayout>
#include <QDebug>
#include <vector>
#include <QVTKOpenGLWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPropPicker.h>
#include <vtkCubeSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <vtkPointPicker.h>
#include <QFile>
#include <QDir>
#include "points_in_boxes.h"


BoxWidget::BoxWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* lay = new QHBoxLayout(this);
    lay->setContentsMargins(0,0,0,0);

    // 左边：VTK 渲染
    vtkWidget_ = new QVTKOpenGLWidget(this);
    lay->addWidget(vtkWidget_, 9);

    boxList_ = new QListWidget(this);
    lay->addWidget(boxList_, 1);
    connect(boxList_, &QListWidget::itemClicked,
            this, &BoxWidget::onBoxListItemClicked);

    renderWindow_ = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    vtkWidget_->SetRenderWindow(renderWindow_);

    viewer_.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    renderWindow_->AddRenderer(viewer_->getRendererCollection()->GetFirstRenderer());

    auto* interactor = vtkWidget_->GetInteractor();
    style = vtkSmartPointer<CustomInteractorStyle>::New();
    style->setQtViewer(this);
    interactor->SetInteractorStyle(style);

    viewer_->setBackgroundColor(0.1,0.1,0.12);
    viewer_->addCoordinateSystem(0.5, "axis");
    // ensureDemoCloud();

     renderWindow_->Render();
    getDefaultSavePath();
    boxes_ = storage.loadQt();

    // boxes_ = boxHandler.boxes_;
    for (auto& it : boxes_) {
        // 重建actor和边
        // updateBoxActor(it);
        boxList_->addItem(QString::fromStdString(it.id));
    }


}

QString BoxWidget::getDefaultSavePath() {
    QString settingPath = QDir::current().absolutePath() + "/setting";

    QDir dir(settingPath);
    if (!dir.exists()) {
        if (!dir.mkpath(".")) {
            qWarning() << "创建目录失败:" << settingPath << "错误:" << strerror(errno);
            return "";
        }
    }
    return settingPath;
}

void BoxWidget::updateBoxsPoints(const BoxItem& item)
{
    // PointsInBoxes::Box3d m_box_ = {item.params.cx,item.params.cy,item.params.cz,item.params.sx,item.params.sy,item.params.sz,item.params.yawDeg};
    std::vector<std::vector<int>> indexs;
    std::vector<PointsInBoxes::Box3d> m_box_ = {{item.params.cx,item.params.cy,item.params.cz,item.params.sx,item.params.sy,item.params.sz,item.params.yawDeg}};
    PointsInBoxes::points_in_boxes_cpu(m_box_,cloud_,indexs);
    temp_->clear();
    for (auto& indx : indexs) {
        for (int i = 0; i < indx.size(); ++i) {
            if(!indx[i]) continue;
            temp_->push_back(cloud_->points[i]);
        }
    }
    std::string pid = "cloud_" + item.id;
    if(viewer_->contains(pid))
    {
        viewer_->removeShape(pid);
    }
    viewer_->addPointCloud<PointT>(temp_, pid);
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, pid);
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, pid);
    // renderWindow_->Render();

}

void BoxWidget::onPointPickedWithCtrl(int x, int y) {
    vtkRenderer* ren = viewer_->getRendererCollection()->GetFirstRenderer();
    vtkSmartPointer<vtkPointPicker> picker = vtkSmartPointer<vtkPointPicker>::New();
    picker->SetTolerance(0.02); // 可按点密度调
    if (picker->Pick(x, y, 0, ren)) {

        int* size = vtkWidget_->GetRenderWindow()->GetSize();
        int pickX = x;
        int pickY = size[1] - y;

        bool ok = picker->Pick(pickX, pickY, 0, viewer_->getRendererCollection()->GetFirstRenderer());
        if (!ok) return ;

        double pos[3]; picker->GetPickPosition(pos);
        kdtree.setInputCloud(cloud_);
        PointT searchPoint(pos[0], pos[1], pos[2]);

        std::vector<int> idx(1);
        std::vector<float> dist(1);
        if (kdtree.nearestKSearch(searchPoint, 1, idx, dist) > 0) {
            const auto& p = cloud_->at(idx[0]);
            std::string pid = "picked_sh";
            if(viewer_->contains(pid))
            {
                viewer_->removeShape(pid);
            }
            viewer_->addSphere(p, 0.05, 1.0, 1.0, 0.0, pid);
            pickedPoints_.push_back(p);

            BoxItem item;
            auto id = genNextId();
            item.id = id.toStdString();
            item.params = {p.x,p.y,p.z, 1.0, 1.0, 1.0, 0.0};

            boxes_.insert(id, item);
            // boxHandler.addOrUpdateBox(item.id, item);
            storage.save(boxes_);
            updateBoxActor(boxes_[id]);
            setSelected(id);
            boxList_->addItem(id);

            renderWindow_->Render();
            emit selectionChanged();
        }
    }
}


void BoxWidget::ensureDemoCloud() {
    // generate a small random cloud if not provided
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    for (int i=0;i<1000;++i) {
        PointT p;
        p.x = (std::rand()%1000)/100.0 - 5.0;
        p.y = (std::rand()%1000)/100.0 - 5.0;
        p.z = (std::rand()%1000)/200.0 - 2.5;
        cloud->push_back(p);
    }
    viewer_->addPointCloud<PointT>(cloud, "cloud");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
}

void BoxWidget::loadPCD(const QString& path) {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT>(path.toStdString(), *cloud) == 0) {
        if (viewer_->contains("cloud")) viewer_->removePointCloud("cloud");
        viewer_->addPointCloud<PointT>(cloud, "cloud");
        viewer_->resetCamera();
        renderWindow_->Render();
    } else {
        qWarning() << "加载PCD失败:" << path;
    }
}

void BoxWidget::updateCloud(const pcl::PointCloud<PointT>::ConstPtr cloud)
{
    if(cloud->empty()) return;
    cloud_->clear();
    cloud_->reserve(cloud->points.size());
    cloud_->insert(cloud_->end(),cloud->begin(),cloud->end());

    if (viewer_->contains("cloud")) viewer_->removePointCloud("cloud");
    viewer_->addPointCloud<PointT>(cloud_, "cloud");
    viewer_->resetCamera();
    renderWindow_->Render();
}


QString BoxWidget::genNextId() const {
    if (boxes_.isEmpty()) {
        return "box_1";
    }
    int maxNum = 0;
    for (const auto& box : boxes_) {
        bool ok = false;
        int num = QString::fromStdString(box.id).mid(4).toInt(&ok);

        if (ok && num > maxNum) {
            maxNum = num;
        }
    }
    return QString("box_%1").arg(maxNum + 1);
}

BoxParams BoxWidget::getBoxParams(const QString& id) const {
    BoxParams p{0,0,0, 1,1,1, 0};
    auto it = boxes_.find(id);
    if (it!=boxes_.end()) p = it->params;
    return p;
}

void BoxWidget::updateBoxParams(const QString& id, float cx, float cy, float cz,
                                      float sx, float sy, float sz, float yawDeg) {
    auto it = boxes_.find(id);
    if (it==boxes_.end()) return;
    it->params = {cx,cy,cz,sx,sy,sz,yawDeg};
    updateBoxActor(*it);
    setSelected(id);
    renderWindow_->Render();
}



void BoxWidget::updateBoxActor(BoxItem& item) {
    for (const auto &eid : item.edgeIds) {
        if (viewer_->contains(eid))
            viewer_->removeShape(eid);
    }
    item.edgeIds.clear();

    // 2) 计算局部 8 个顶点（未旋转、未平移）
    const float hx = static_cast<float>(item.params.sx * 0.5);
    const float hy = static_cast<float>(item.params.sy * 0.5);
    const float hz = static_cast<float>(item.params.sz * 0.5);

    // 局部顶点顺序（上面 0..3，下面 4..7）
    std::array<Eigen::Vector3f, 8> local = {{
        { +hx, +hy, +hz }, // 0
        { -hx, +hy, +hz }, // 1
        { -hx, -hy, +hz }, // 2
        { +hx, -hy, +hz }, // 3
        { +hx, +hy, -hz }, // 4
        { -hx, +hy, -hz }, // 5
        { -hx, -hy, -hz }, // 6
        { +hx, -hy, -hz }  // 7
    }};

    // 3) 构造旋转矩阵（绕 Z 轴）与中心平移
    const double yaw_rad = item.params.yawDeg * M_PI / 180.0;
    const float cy = static_cast<float>(cos(yaw_rad));
    const float sy = static_cast<float>(sin(yaw_rad));
    Eigen::Matrix2f R2;
    R2 << cy, -sy,
        sy,  cy;

    Eigen::Vector3f center(static_cast<float>(item.params.cx),
                           static_cast<float>(item.params.cy),
                           static_cast<float>(item.params.cz));

    // 4) 计算世界坐标下的 8 个顶点
    std::array<Eigen::Vector3f, 8> world;
    for (int i = 0; i < 8; ++i) {
        Eigen::Vector2f xy(local[i].x(), local[i].y());
        Eigen::Vector2f rxy = R2 * xy;
        world[i] = Eigen::Vector3f(rxy.x(), rxy.y(), local[i].z()) + center;
    }

    // 5) 定义需要绘制的 12 条边（顶点索引对）
    static const std::array<std::pair<int,int>, 12> edges = {{
        {0,1},{1,2},{2,3},{3,0}, // top rectangle
        {4,5},{5,6},{6,7},{7,4}, // bottom rectangle
        {0,4},{1,5},{2,6},{3,7}  // vertical edges
    }};

    // 6) 颜色与线宽
    double r = (item.id == selectedId_) ? 1.0 : 1.0;
    double g = (item.id == selectedId_) ? 1.0 : 0.1;
    double b = (item.id == selectedId_) ? 0.0 : 0.1;
    double line_width = 2.0;

    // 7) 添加每条边为单独的 shape（便于选中 / 删除 / 修改）
    for (size_t ei = 0; ei < edges.size(); ++ei) {
        const auto &pr = edges[ei];
        const Eigen::Vector3f &A = world[pr.first];
        const Eigen::Vector3f &B = world[pr.second];

        pcl::PointXYZ p1(A.x(), A.y(), A.z());
        pcl::PointXYZ p2(B.x(), B.y(), B.z());

        std::string edgeId = item.id + "_e" + std::to_string(ei);

        viewer_->addLine(p1, p2, edgeId);
        viewer_->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR,
            r, g, b,
            edgeId
            );

        viewer_->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
            line_width,
            edgeId
            );

        item.edgeIds.push_back(edgeId);
        edgeToBox_[edgeId] = item.id;
    }

    std::string labelId = item.id;
    if (viewer_->contains(labelId)) {
        viewer_->removeShape(labelId);
    }

    pcl::PointXYZ pcl_position;
    pcl_position.x = center.x();
    pcl_position.y = center.y();
    pcl_position.z = center.z();
    viewer_->addText3D(labelId, pcl_position, 0.2,  r, g, b, labelId);
    updateBoxsPoints(item);
    renderWindow_->Render();
}



void BoxWidget::setSelected(const QString& id) {
    selectedId_ = id.toStdString();

    for (auto it = boxes_.begin(); it!=boxes_.end(); ++it) {
        updateBoxActor(it.value());
    }

    renderWindow_->Render();
    emit selectionChanged();
}

void BoxWidget::onBoxListItemClicked(QListWidgetItem* item) {
    if (!item) return;
    QString id = item->text();
    if (boxes_.contains(id)) {
        setSelected(id);
        renderWindow_->Render();
        emit selectionChanged();
    }
}



void BoxWidget::addBoxAtViewCenter() {
    vtkRenderer* ren = viewer_->getRendererCollection()->GetFirstRenderer();
    int* size = ren->GetRenderWindow()->GetSize();

    double displayX = size[0] / 2.0;
    double displayY = size[1] / 2.0;

    ren->SetDisplayPoint(displayX, displayY, 0.5);
    ren->DisplayToWorld();
    double world[4];
    ren->GetWorldPoint(world);

    if (world[3] == 0.0) {
        return; // 避免除零
    }

    for (int i = 0; i < 3; i++) {
        world[i] /= world[3];
    }

    BoxItem item;
    auto id = genNextId();
    item.id = id.toStdString();
    item.params = {static_cast<float>(world[0]), static_cast<float>(world[1]), static_cast<float>(world[2]), 1.0, 1.0, 1.0, 0.0};

    boxes_.insert(id, item);
    // boxHandler.addOrUpdateBox(id, item);
     storage.save(boxes_);
    updateBoxActor(boxes_[id]);
    setSelected(id);
    boxList_->addItem(id);

    renderWindow_->Render();
}

void BoxWidget::addBoxByScreenPos(int x, int y) {
    vtkRenderer* ren = viewer_->getRendererCollection()->GetFirstRenderer();
    vtkSmartPointer<vtkPropPicker> picker = vtkSmartPointer<vtkPropPicker>::New();
    picker->Pick(x, y, 0, ren);
    double world[3];
    picker->GetPickPosition(world);

    // If picking empty space, fallback to focal point
    if (world[0]==0 && world[1]==0 && world[2]==0) {
        double fp[3];
        ren->GetActiveCamera()->GetFocalPoint(fp);
        world[0]=fp[0]; world[1]=fp[1]; world[2]=fp[2];
    }

    BoxItem item;
    auto id = genNextId();
    item.id = id.toStdString();
    item.params = {static_cast<float>(world[0]), static_cast<float>(world[1]), static_cast<float>(world[2]), 1.0, 1.0, 1.0, 0.0};
    boxes_.insert(id, item);
    updateBoxActor(boxes_[id]);
    setSelected(id);
    boxList_->addItem(id);
    renderWindow_->Render();
}

void BoxWidget::selectByScreenPos(int x, int y) {
    vtkRenderer* ren = viewer_->getRendererCollection()->GetFirstRenderer();
    vtkSmartPointer<vtkPropPicker> picker = vtkSmartPointer<vtkPropPicker>::New();
    if (picker->Pick(x, y, 0, ren)) {
        vtkProp* prop = picker->GetViewProp();
        vtkActor* picked = vtkActor::SafeDownCast(prop);
        if (picked) {
            // find which box this actor belongs to
            QString found;
            // for (auto it = boxes_.begin(); it!=boxes_.end(); ++it) {
            //     if (it->actor == picked) {
            //         found = it.key();
            //         break;
            //     }
            // }
            auto shapeMap = viewer_->getShapeActorMap();
            for (auto &kv : *shapeMap) {
                if (kv.second.GetPointer() == picked) {
                    std::string pickedId = kv.first;
                    auto it = edgeToBox_.find(pickedId);
                    if (it != edgeToBox_.end()) {
                        found = QString::fromStdString(it->second);
                        setSelected(QString::fromStdString(it->second));
                        renderWindow_->Render();
                        return;
                    }
                }
            }

            if (!found.isEmpty()) {
                setSelected(found);
                renderWindow_->Render();
                return;
            }
        }
    }
    // click background: clear selection
    setSelected(QString());
    renderWindow_->Render();


}

void BoxWidget::deleteSelectedBox() {
    if (selectedId_.empty()) return;
    auto it = boxes_.find(QString::fromStdString(selectedId_));
    if (it != boxes_.end()) {

        for (const auto &eid : it->edgeIds) {
            if (viewer_->contains(eid))
                viewer_->removeShape(eid);
        }
        viewer_->removeShape("cloud_" + it->id);
        viewer_->removeShape(it->id);
        // boxHandler.removeBox(it->id);
        boxes_.erase(it);
        storage.save(boxes_);
        QList<QListWidgetItem*> found = boxList_->findItems(QString::fromStdString(selectedId_), Qt::MatchExactly);
        for (auto* f : found) {
            delete boxList_->takeItem(boxList_->row(f));
        }
        selectedId_.clear();
        emit selectionChanged();
        renderWindow_->Render();
    }
}
