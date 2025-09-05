// -------------------- src/BoundingBox.cpp --------------------
#include "BoundingBox.h"
#include <vtkCubeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>


// Helper function to create actor for box; user code will set position/rotation later
vtkSmartPointer<vtkActor> createBoxActor(double cx,double cy,double cz,double lx,double ly,double lz,double yaw_deg)
{
    vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New();
    cube->SetXLength(lx);
    cube->SetYLength(ly);
    cube->SetZLength(lz);
    cube->Update();


    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(cube->GetOutputPort());


    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);


    vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New();
    t->PostMultiply();
    t->Translate(cx, cy, cz);
    t->RotateZ(yaw_deg);
    actor->SetUserTransform(t);


    actor->GetProperty()->SetRepresentationToWireframe();
    actor->GetProperty()->SetLineWidth(2.0);
    return actor;
}
