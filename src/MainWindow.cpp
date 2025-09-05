#include "MainWindow.h"
#include "PCLViewerWidget.h"

#include <QMenuBar>
#include <QFileDialog>
#include <QKeyEvent>
#include <QStatusBar>
#include <QLabel>
#include <QDir>
#include <QFileInfoList>
#include <QMessageBox>
#include <QDockWidget>


MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    m_setting_menu = std::make_shared<settingMenu>();

    QTabWidget* tabs = new QTabWidget(this);

    QDockWidget* dockWidget = new QDockWidget(tr("setting"), this);
    dockWidget->setWidget(m_setting_menu.get());
    dockWidget->setAllowedAreas(Qt::RightDockWidgetArea);       // 只能停靠右侧
    dockWidget->setFeatures(QDockWidget::DockWidgetFloatable);  // 允许浮动
    addDockWidget(Qt::RightDockWidgetArea, dockWidget);
    dockWidget->hide();

    // ================== 中央点云视图 ==================
    viewer_ = new PCLViewerWidget(this);
    tabs->addTab(viewer_, "点云查看");
    boxTab = new BoxWindow(this);
    connect(boxTab, &BoxWindow::statusMessage, this, [this](const QString& msg){
        status_label_left_->setText(msg);
    });
    tabs->addTab(boxTab, "Box编辑");

    setCentralWidget(tabs);

    m_setting_menu->viewer_ = viewer_->viewer_;
    viewer_->registerCallback(std::bind(&settingMenu::handleCloud, m_setting_menu.get(), std::placeholders::_1));
    viewer_->registerCallback(std::bind(&BoxWindow::updateCloud, boxTab, std::placeholders::_1));
    m_setting_menu->registerUpdateViewerCallback(std::bind(&PCLViewerWidget::updateViewer,viewer_));

    // ================== 菜单栏 ==================
    auto* fileMenu = menuBar()->addMenu(tr("文件(&F)"));
    auto* openAct = fileMenu->addAction(tr("打开PCD文件夹(&O)"));
    openAct->setShortcut(QKeySequence("Ctrl+O"));
    connect(openAct, &QAction::triggered, this, &MainWindow::openFolder);

    auto* setMenu = menuBar()->addMenu(tr("设置(&S)"));
    auto* setAct = setMenu->addAction(tr("显示/隐藏设置面板"));
    connect(setAct, &QAction::triggered, [dockWidget](){
        dockWidget->setVisible(!dockWidget->isVisible());
    });


    // ================== 状态栏 ==================
    status_label_left_ = new QLabel(this);
    status_label_right_ = new QLabel(this);
    statusBar()->addWidget(status_label_left_, 1);
    statusBar()->addPermanentWidget(status_label_right_, 0);
    statusBar()->showMessage(
        tr("提示：Ctrl+左键点击点云可查看坐标；左右方向键切换点云"), 8000);




    // 将 viewer_ 的点选结果显示到状态栏
    connect(viewer_, &PCLViewerWidget::pointPicked,
            this, [this](float x, float y, float z){
                status_label_left_->setText(
                    QString("选中点: x=%1, y=%2, z=%3")
                        .arg(x, 0, 'f', 4)
                        .arg(y, 0, 'f', 4)
                        .arg(z, 0, 'f', 4));
            });
    someFunction();

}

void MainWindow::openFolder()
{
    QString dir = QFileDialog::getExistingDirectory(this, tr("选择包含 .pcd 的文件夹"));
    if (dir.isEmpty()) return;

    QDir qdir(dir);
    QStringList filters; filters << "*.pcd" << "*.PCD";
    QFileInfoList list = qdir.entryInfoList(filters, QDir::Files, QDir::Name);

    pcd_files_.clear();
    for (const auto& fi : list) pcd_files_ << fi.absoluteFilePath();

    if (pcd_files_.isEmpty()) {
        QMessageBox::warning(this, tr("无文件"), tr("该目录下未找到 .pcd 文件"));
        current_index_ = -1;
        viewer_->clear();
        status_label_right_->clear();
        return;
    }
    current_index_ = 0;
    loadCurrentCloud();
    updateStatusIndex();
}

void MainWindow::someFunction() {
    this->activateWindow(); // 激活窗口（使其显示在最前）
    this->setFocus();       // 设置焦点到窗口
    this->setFocusPolicy(Qt::StrongFocus); // 确保窗口能接收焦点
}


void MainWindow::keyPressEvent(QKeyEvent* event)
{
    if (pcd_files_.isEmpty()) { QMainWindow::keyPressEvent(event); return; }

    if (event->key() == Qt::Key_Left) {
        current_index_ = (current_index_ - 1 + pcd_files_.size()) % pcd_files_.size();
        loadCurrentCloud();
        updateStatusIndex();
        return;
    } else if (event->key() == Qt::Key_Right) {
        current_index_ = (current_index_ + 1) % pcd_files_.size();
        loadCurrentCloud();
        updateStatusIndex();
        return;
    }
    // else if(event->key() == Qt::Key_Plus || event->key() == Qt::Key_Equal) {
    //     std::cout << " + " << std::endl;
    //     boxTab->adjustCurrentAxis(+1);
    // }
    // else if(event->key() == Qt::Key_Minus) {
    //     std::cout << "- " << std::endl;
    //     boxTab->adjustCurrentAxis(-1);
    // }
    // else if(event->key() == Qt::Key_M) {
    //     std::cout << "- " << std::endl;
    //     boxTab->adjustCurrentAxis(-1);
    // }




    QMainWindow::keyPressEvent(event);
}

void MainWindow::loadCurrentCloud()
{
    if (current_index_ < 0 || current_index_ >= pcd_files_.size()) return;
    viewer_->loadPCD(pcd_files_[current_index_]);
}

void MainWindow::updateStatusIndex()
{
    status_label_right_->setText(
        QString("[%1 / %2]  %3")
            .arg(current_index_ + 1)
            .arg(pcd_files_.size())
            .arg(QFileInfo(pcd_files_[current_index_]).fileName()));
}
