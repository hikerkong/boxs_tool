
#include "CustomInteractorStyle.h"
#include "boxWidget.h"
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkPropPicker.h>
#include <vtkRenderWindow.h>

vtkStandardNewMacro(CustomInteractorStyle);

void CustomInteractorStyle::OnLeftButtonDown() {
    int* pos = this->GetInteractor()->GetEventPosition();
    bool shift = this->GetInteractor()->GetShiftKey();
    if(viewer_) {
        if(shift) {
            viewer_->addBoxByScreenPos(pos[0], pos[1]);
        }
        else if (this->GetInteractor()->GetControlKey()) {
             // viewer_->addBoxByScreenPos(pos[0], pos[1]);
             viewer_->onPointPickedWithCtrl(pos[0], pos[1]);
        }
    }

    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
    dragging_ = true;
}

void CustomInteractorStyle::OnLeftButtonUp() {
    dragging_ = false;
    vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
}
