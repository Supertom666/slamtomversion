//
// Created by tom on 25/11/2020.
//

#ifndef ALOGRITHM_H
#define ALOGRITHM_H
#include "myslam/commoninclude.h"

namespace myslam {

/**
 * linear triangulation with SVD
 * @param poses     poses,
 * @param points    points in normalized plane
 * @param pt_world  triangulated point in the world
 * @return true if success
 */
   //三角化，已知位姿和归一化平面的点 变换矩阵RT ,恢复对应的3D点
    inline bool triangulation(const std::vector<SE3> &poses,
                              const std::vector<Vec3> points, Vec3 &pt_world) {
        MatXX A(2 * poses.size(), 4);
        VecX b(2 * poses.size());
        b.setZero();
        for (size_t i = 0; i < poses.size(); ++i) {
            Mat44 m = poses[i].matrix();
            //    x1* p1.row3(1x4) - p1row1(1x4)  x2* p2.row3(1x4) - p1row1(1x4)
            //    y1* p1.row3(1x4) - p1row2(1x4)   y2* p2.row3(1x4) - p2row2(1x4)
            A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
            A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
           // A.block<1, 4>(2 * i, 0) = points[i][0] * poses[i].row(2) - poses[i].row(0);
           //A.block<1, 4>(2 * i + 1, 0) = points[i][1] * poses[i].row(2) - poses[i].row(1);
        }
        auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        //col3 / the last value
        pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();
        //if the third 奇异值 smaller  the second 奇异值 解质量好
        if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
            // 解质量好
            return true;
        }
        return false;
    }

// converters
    inline Vec2 toVec2(const cv::Point2f p) { return Vec2(p.x, p.y); }

}  // namespace myslam
#endif //INC_0_1_ALOGRITHM_H
