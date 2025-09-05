#ifndef BOXWINDOW_H
#define BOXWINDOW_H

#include <QWidget>
#include <QDoubleSpinBox>
#include <QLabel>
#include "boxWidget.h"

class BoxWindow : public QWidget {
    Q_OBJECT
public:
    explicit BoxWindow(QWidget* parent = nullptr);
    void updateCloud(const pcl::PointCloud<PointT>::ConstPtr cloud);
protected:
    void keyPressEvent(QKeyEvent* event) override;
signals:
    void statusMessage(const QString& msg);

private slots:
    void onOpenPCD();
    void onAddBoxAtCenter();
    void onApplyParams();
public:
    void adjustCurrentAxis(int direction);
    void adjustCurrentSize(int direction);
private:

    void syncUiFromSelection();
    void createControls();

    BoxWidget* viewer_ = nullptr;

    QDoubleSpinBox* posX_ = nullptr;
    QDoubleSpinBox* posY_ = nullptr;
    QDoubleSpinBox* posZ_ = nullptr;
    QDoubleSpinBox* sizeX_ = nullptr;
    QDoubleSpinBox* sizeY_ = nullptr;
    QDoubleSpinBox* sizeZ_ = nullptr;
    QDoubleSpinBox* yawDeg_ = nullptr;
    QLabel* selInfo_ = nullptr;
    QLabel* statusLabel_ = nullptr;
};

#endif
