#ifndef POINT_IN_BOXES_H
#define POINT_IN_BOXES_H
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloud::Ptr;


void points_in_boxes_launcher(int batch_size, int boxes_num, int pts_num, const float *boxes,
    const float *pts, int *box_idx_of_points);

namespace PointsInBoxes{
using namespace std;

struct Box3d {
    float x;
    float y;
    float z;
    float dx;
    float dy;
    float dz;
    float heading;
    Box3d(){};
    Box3d(float x_, float y_, float z_, float w_, float l_, float h_, float rt_)
        : x(x_), y(y_), z(z_), dx(w_), dy(l_), dz(h_), heading(rt_) {}
};


int points_in_boxes_cpu(vector<Box3d>& boxes, PointCloud::Ptr& pts, vector<vector<int>>& in_box);


} // namespace PointsInBoxes{





#endif
