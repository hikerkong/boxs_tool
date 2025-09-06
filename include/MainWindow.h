#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <QStringList>
#include <memory>
#include "setting_menu.h"
#include "boxWindow.h"

class PCLViewerWidget;
class QLabel;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);

protected:
    void keyPressEvent(QKeyEvent* event) override;

private slots:
    void openFolder();

private:
    void loadCurrentCloud();
    void updateStatusIndex();
     void someFunction();
    settingMenu* m_setting_menu{nullptr};

    PCLViewerWidget* viewer_ = nullptr;
    QStringList pcd_files_;
    int current_index_ = -1;
    QLabel* status_label_left_ = nullptr;
    QLabel* status_label_right_ = nullptr;
    BoxWindow* boxTab= nullptr;

};
#endif
