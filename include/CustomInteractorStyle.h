
#ifndef CUSTOMINTERACTORSTYLE_H
#define CUSTOMINTERACTORSTYLE_H

#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkSmartPointer.h>
#include <QObject>
#include <QPointer>

class BoxWidget;

class CustomInteractorStyle : public vtkInteractorStyleTrackballCamera {
public:
    static CustomInteractorStyle* New();
    vtkTypeMacro(CustomInteractorStyle, vtkInteractorStyleTrackballCamera);

    void setQtViewer(BoxWidget* v) { viewer_ = v; }

    void OnLeftButtonDown() override;
    void OnLeftButtonUp() override;
signals:
    // void pointPickedWithCtrl(int x, int y);

private:
    QPointer<BoxWidget> viewer_;
    bool dragging_ = false;
};

#endif // CUSTOMINTERACTORSTYLE_H
