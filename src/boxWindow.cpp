#include "boxWindow.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QPushButton>
#include <QFileDialog>
#include <QKeyEvent>
#include <QGroupBox>

enum class Axis { None, X, Y, Z };
enum class SizeT { None, X, Y, Z };
Axis currentAxis_ = Axis::None;
SizeT currentSize_ = SizeT::None;

BoxWindow::BoxWindow(QWidget* parent)
    : QWidget(parent)
{
    viewer_ = new BoxWidget(this);

    // 主布局：左侧 3D viewer，右侧控制面板
    QHBoxLayout* mainLayout = new QHBoxLayout(this);
    mainLayout->addWidget(viewer_,9);

    QVBoxLayout* rightLayout = new QVBoxLayout;
    mainLayout->addLayout(rightLayout);

    // 工具按钮
    QPushButton* addCenterBtn = new QPushButton(tr("在中心添加Box"));
    connect(addCenterBtn, &QPushButton::clicked, this, &BoxWindow::onAddBoxAtCenter);
    rightLayout->addWidget(addCenterBtn);

    // 控制面板
    createControls();
    rightLayout->addWidget(selInfo_->parentWidget(),1); // 直接把 panel 加入布局

    // 状态显示
    // statusLabel_ = new QLabel(tr("Shift+左键：在点击处添加Box；左键单击：选中Box；Delete删除选中Box。"));
    statusMessage(tr("Shift+左键：在点击处添加Box；左键单击：选中Box；Delete删除选中Box。"));

    // rightLayout->addWidget(statusLabel_);

    setLayout(mainLayout);

    // 选中 Box 时更新 UI
    connect(viewer_, &BoxWidget::selectionChanged, this, &BoxWindow::syncUiFromSelection);



}



void BoxWindow::createControls() {
    QGroupBox* controls = new QGroupBox(tr("选中Box参数"));
    QFormLayout* form = new QFormLayout(controls);

    auto mkSpin = [&](double minv, double maxv, double step)->QDoubleSpinBox*{
        auto* s = new QDoubleSpinBox(controls);
        s->setRange(minv, maxv);
        s->setDecimals(3);
        s->setSingleStep(step);
        return s;
    };

    posX_ = mkSpin(-1000, 1000, 0.1);
    posY_ = mkSpin(-1000, 1000, 0.1);
    posZ_ = mkSpin(-1000, 1000, 0.1);
    sizeX_ = mkSpin(0.01, 1000, 0.05);
    sizeY_ = mkSpin(0.01, 1000, 0.05);
    sizeZ_ = mkSpin(0.01, 1000, 0.05);
    yawDeg_ = mkSpin(-180, 180, 1.0);

    selInfo_ = new QLabel(tr("未选中"));
    form->addRow(tr("当前选择"), selInfo_);

    form->addRow(tr("中心X"), posX_);
    form->addRow(tr("中心Y"), posY_);
    form->addRow(tr("中心Z"), posZ_);
    form->addRow(tr("尺寸X"), sizeX_);
    form->addRow(tr("尺寸Y"), sizeY_);
    form->addRow(tr("尺寸Z"), sizeZ_);
    form->addRow(tr("航向角(°)"), yawDeg_);

    QPushButton* applyBtn = new QPushButton(tr("应用到选中Box"));
    connect(applyBtn, &QPushButton::clicked, this, &BoxWindow::onApplyParams);
    form->addRow(applyBtn);



    controls->setLayout(form);

    // SpinBox 改变时自动应用
    auto connectSpin = [&](QDoubleSpinBox* spin){
        connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &BoxWindow::onApplyParams);
    };
    connectSpin(posX_); connectSpin(posY_); connectSpin(posZ_);
    connectSpin(sizeX_); connectSpin(sizeY_); connectSpin(sizeZ_);
    connectSpin(yawDeg_);
}

void BoxWindow::onOpenPCD() {
    QString path = QFileDialog::getOpenFileName(this, tr("打开PCD"), QString(), tr("PCD Files (*.pcd)"));
    if(path.isEmpty()) return;
    viewer_->loadPCD(path);
}

void BoxWindow::updateCloud(const pcl::PointCloud<PointT>::ConstPtr cloud)
{
    if(cloud->empty()) return;
    viewer_->updateCloud(cloud);
}

void BoxWindow::onAddBoxAtCenter() {
    viewer_->addBoxAtViewCenter();
}

void BoxWindow::onApplyParams() {
    auto id = viewer_->currentSelectionId();
    if(id.empty())
    {
        std::cout << "id None " << std::endl;
        return;
    }

    viewer_->updateBoxParams(QString::fromStdString(id),
                             posX_->value(),
                             posY_->value(),
                             posZ_->value(),
                             sizeX_->value(),
                             sizeY_->value(),
                             sizeZ_->value(),
                             yawDeg_->value());
}

void BoxWindow::syncUiFromSelection() {
    auto id = viewer_->currentSelectionId();
    if(id.empty()) {
        selInfo_->setText(tr("未选中"));
        return;
    }
    auto params = viewer_->getBoxParams(QString::fromStdString(id));
    selInfo_->setText(QString::fromStdString(id));
    posX_->setValue(params.cx);
    posY_->setValue(params.cy);
    posZ_->setValue(params.cz);
    sizeX_->setValue(params.sx);
    sizeY_->setValue(params.sy);
    sizeZ_->setValue(params.sz);
    yawDeg_->setValue(params.yawDeg);
}

void BoxWindow::adjustCurrentAxis(int direction) {
    if(currentAxis_ == Axis::None)
    {
        std::cout << "Axis::None " << std::endl;
        return;
    }

    double step = 0.1;
    switch(currentAxis_) {
    case Axis::X: posX_->setValue(posX_->value() + direction * step); break;
    case Axis::Y: posY_->setValue(posY_->value() + direction * step); break;
    case Axis::Z: posZ_->setValue(posZ_->value() + direction * step); break;
    default: break;
    }
    onApplyParams();
}

void BoxWindow::adjustCurrentSize(int direction) {
    if(currentSize_ == SizeT::None)
    {
        return;
    }

    double step = 0.1;
    switch(currentSize_) {
    case SizeT::X: sizeX_->setValue(sizeX_->value() + direction * step); break;
    case SizeT::Y: sizeY_->setValue(sizeY_->value() + direction * step); break;
    case SizeT::Z: sizeZ_->setValue(sizeZ_->value() + direction * step); break;
    default: break;
    }
    onApplyParams();
}

void BoxWindow::keyPressEvent(QKeyEvent* event) {


    if (event->modifiers() & Qt::ShiftModifier) {
        if (event->key() == Qt::Key_W) {
            currentSize_ = SizeT::Y;
            adjustCurrentSize(+1);
            return;
        }
        else if (event->key() == Qt::Key_S) {
            currentSize_ = SizeT::Y;
            adjustCurrentSize(-1);
            return;
        }
        else if (event->key() == Qt::Key_A) {
            currentSize_ = SizeT::X;
            adjustCurrentSize(-1);
            return;
        }
        else if (event->key() == Qt::Key_D) {
            currentSize_ = SizeT::X;
            adjustCurrentSize(+1);
            return;
        }
        else if (event->key() == Qt::Key_Q) {
            currentSize_ = SizeT::Z;
            adjustCurrentSize(-1);
            return;
        }
        else if (event->key() == Qt::Key_E) {
            currentSize_ = SizeT::Z;
            adjustCurrentSize(+1);
            return;
        }
    }
    else {
        if(event->key() == Qt::Key_Delete) {
            viewer_->deleteSelectedBox();
        }

        if (event->key() == Qt::Key_W) {
            currentAxis_ = Axis::Y;
            adjustCurrentAxis(+1);
        }
        else if (event->key() == Qt::Key_S) {
            currentAxis_ = Axis::Y;
            adjustCurrentAxis(-1);
        }
        else if (event->key() == Qt::Key_A) {
            currentAxis_ = Axis::X;
            adjustCurrentAxis(-1);
        }
        else if (event->key() == Qt::Key_D) {
            currentAxis_ = Axis::X;
            adjustCurrentAxis(+1);
        }
        else if (event->key() == Qt::Key_Q) {
            currentAxis_ = Axis::Z;
            adjustCurrentAxis(-1);
        }
        else if (event->key() == Qt::Key_E) {
            currentAxis_ = Axis::Z;
            adjustCurrentAxis(+1);
        }
        else {
            QWidget::keyPressEvent(event);
        }
    }
}
