#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkTransform.h>
#include <string>


struct BoundingBox
{
    std::string id; // unique id for the actor in visualizer
    double cx, cy, cz; // center
    double lx, ly, lz; // sizes
    double yaw; // rotation around Z (degrees)
    vtkSmartPointer<vtkActor> actor;
};

vtkSmartPointer<vtkActor> createBoxActor(double cx,double cy,double cz,
                                         double lx,double ly,double lz,
                                         double yaw_deg);


#endif // BOUNDINGBOX_H

