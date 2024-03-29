#ifndef COMMON_DEF_H
#define COMMON_DEF_H

#include <memory>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include "../3rdPartLib/Sophus/sophus/so3.h"
#include "../3rdPartLib/Sophus/sophus/se3.h"
#include <unordered_map>
#include <unordered_set>
using namespace std;
using namespace Eigen;
using namespace Sophus;

typedef Eigen::Matrix<double, 2, 1> Vec2;
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<int, 3, 1>   Vec3I;
typedef Eigen::Matrix<size_t, 6, 1>   Vec6I;
typedef Eigen::Matrix<int, 6, 4> Mat6x4I;
typedef Eigen::Matrix<double, 4, 1> Vec4;
typedef Eigen::Matrix<double, 6, 1> Vec6;
typedef Eigen::Matrix<double, 9, 1> Vec9;
typedef Eigen::Matrix<double, 12, 1> Vec12;
typedef Eigen::Matrix<double, 15, 1> Vec15;
typedef Eigen::Matrix<double, 16, 1> Vec16;
typedef Eigen::Matrix<double, 1, 1> Mat1x1;
typedef Eigen::Matrix<double, 1, 4> Mat1x4;
typedef Eigen::Matrix<double, 3, 3> Mat3x3;
typedef Eigen::Matrix<double, 3, 4> Mat3x4;
typedef Eigen::Matrix<double, 4, 4> Mat4x4;
typedef Eigen::Matrix<double, 6, 6> Mat6x6;
typedef Eigen::Matrix<double, 9, 9> Mat9x9;
typedef Eigen::Matrix<double, 12, 12> Mat12x12;
typedef Eigen::Matrix<double, 15, 15> Mat15x15;
typedef Eigen::Matrix<double, 15, 6> Mat15x6;
typedef Eigen::Matrix<double, 6, 15> Mat6x15;
typedef Eigen::Matrix<double, 9, 15> Mat9x15;
typedef Eigen::Matrix<double, 15, 12> Mat15x12;
typedef Eigen::Matrix<double, 15, 9> Mat15x9;
typedef Eigen::Matrix<double, 3, 15> Mat3x15;
typedef Eigen::Matrix<double, 15, 3> Mat15x3;
typedef Eigen::Matrix<double, 1, 15> Mat1x15;
typedef Eigen::Matrix<double, 15, 1> Mat15x1;



typedef pcl::PointXYZ     PointP;
typedef pcl::PointXYZRGB  PointRGB;
typedef pcl::PointXYZRGBA PointRGBA;
typedef pcl::PointXYZI    PointI;

typedef pcl::PointCloud<PointP>    PointCloudP;
typedef pcl::PointCloud<PointRGB>  PointCloudRGB;
typedef pcl::PointCloud<PointRGBA> PointCloudRGBA;
typedef pcl::PointCloud<PointI>    PointCloudI;

typedef PointCloudP::Ptr    PointCloudP_ptr;
typedef PointCloudRGB::Ptr  PointCloudRGB_ptr;
typedef PointCloudRGBA::Ptr PointCloudRGBA_ptr;
typedef PointCloudI::Ptr    PointCloudI_ptr;

#endif // COMMON_H
