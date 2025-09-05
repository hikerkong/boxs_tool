#include <assert.h>
#include <math.h>

#include "points_in_boxes.h"

namespace PointsInBoxes{

using namespace std;

inline void lidar_to_local_coords_cpu(float shift_x, float shift_y, float rot_angle, float &local_x, float &local_y){
    float cosa = cos(-rot_angle), sina = sin(-rot_angle);
    local_x = shift_x * cosa + shift_y * (-sina);
    local_y = shift_x * sina + shift_y * cosa;
}


inline int check_pt_in_box3d_cpu(const PointT& pt, const Box3d& box3d, float &local_x, float &local_y){
    // param pt: (x, y, z)
    // param box3d: [x, y, z, dx, dy, dz, heading], (x, y, z) is the box center
    const float MARGIN = 1e-2;

    if (fabsf(pt.z - box3d.z) > box3d.dz / 2.0) return 0;
    lidar_to_local_coords_cpu(pt.x - box3d.x, pt.y - box3d.y, box3d.heading, local_x, local_y);
    float in_flag = (fabs(local_x) < box3d.dx / 2.0 + MARGIN) & (fabs(local_y) < box3d.dy / 2.0 + MARGIN);
    return in_flag;
}


int points_in_boxes_cpu(vector<Box3d>& boxes, PointCloud::Ptr& pts, vector<vector<int>>& in_box){
    // params boxes: (N, 7) [x, y, z, dx, dy, dz, heading], (x, y, z) is the box center, each box DO NOT overlaps
    // params pts: (num_points, 3) [x, y, z]
    // params in_box: (N, num_points)


    int boxes_num = boxes.size();
    int pts_num = pts->size();

    in_box.resize(boxes_num);
    for (int i = 0; i < boxes_num; ++i) {
        in_box[i].resize(pts_num);
    }

    float local_x = 0, local_y = 0;
    for (int i = 0; i < boxes_num; i++){
        for (int j = 0; j < pts_num; j++){
            int cur_in_flag = check_pt_in_box3d_cpu(pts->points[j], boxes[i], local_x, local_y);
            in_box[i][j] = cur_in_flag;
        }
    }

    return 1;
}


} // namespace PointsInBoxes{
