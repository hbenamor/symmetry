#ifndef SYMMETRY_H
#define SYMMETRY_H

#include <unistd.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <limits>
#include <random>

#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/intersections.h>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

#define DEBUG false

/**
 * \brief Vector to PointXYZ
 * \param v the vector to be transformed
 * \return Returns a PointXYX
 */
pcl::PointXYZ VectorEigenToPointXYZ(Eigen::Vector3f v);

/**
 * \brief Vector to PointXYZ
 * \param v the vector to be transformed
 * \return Returns a PointXYX
 */
pcl::PointXYZ VectorEigenToPointXYZ(Eigen::Vector4f v);

/**
 * \brief Estimate the surface normals of a point cloud
 *
 * Calculates the normals of the point cloud using a pca method
 * \param cloud         The input point cloud
 * \param outputNormals The output normal vectors
 * \param radius        The radius of the local neighborhood to use for esimtation
 */
void estimateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr outputNormals, const double radius = 0.03);

/**
 * \brief Generates hypothesis for the location of and axis of symmetry
 *
 * Given a point cloud, pairs of lines are generated from points and their normals
 * the resulting lines are intersected and the point of intersection stored
 * \param cloud         The input point cloud
 * \param normals       The normals of the point cloud
 * \param numSamples    The number of hypotheses to generate
 */
void generateSymmetryAxisHypotheses(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud, const int numSamples = 50000);

/**
 * \brief Performs a coordinate system transformation on to the principle axes
 *
 * Given a point cloud and the principle axis of the pcl, the function
 * performs a projection onto the principal axes of the new coordinate system
 * \param cloud  The input point cloud
 * \param origin The origin of the new coordinate system
 * \param yAxis  The principal axis of the coordinate system
 * \param transf The pose of the point cloud
 * \return Returns the transformed point cloud
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr projectOnPrincipleCS(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector3f origin, Eigen::Vector3f yAxis, Eigen::Affine3f& transf);

/**
 * \brief Calculates the principal axis of a point cloud
 *
 * Given a point cloud it calculate the principal axis using line fitting and RANSAC
 *
 * \param cloud         The input point cloud
 * \param outCloud      The point cloud of all inliers
 * \param distThresh    The distance threshold for determining inliers
 * \param axisOrigin    The computed origin of the principal axis
 * \param axisDir       The computed direction of the principal axis
 */
void computePrincipalAxis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud, double distThresh,
    Eigen::Vector3f& axisOrigin, Eigen::Vector3f& axisDir);

/**
 * \brief Performs an inverse revolution around the canonical y-axis
 *
 * This function can be used in order to identify the the curve from which a
 * a revoluted object has been created. The function assumes that the point cloud
 * exists in the canonical coordinate system. Further, it assumes that the y axis
 * is the axis of revolution for this object. The function unrevolutes all points
 * such that they lie on the xy-plane
 *
 * \param cloud The input cloud
 */
void unrevoluteCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

/**
 * \brief Revolutes a curve around an axis to create rotationally symmetric point cloud
 *
 * Given a point cloud defining the outer curve, this function revolves the curve around the
 * y-axis to generate a large point cloud for a rotationally symmetric object.
 *
 * \param cloud         The input point cloud holding points along a curve
 * \param outCloud      The output point cloud holding a rotationally symmetric pcl
 * \param sections      The number of cross sections to use during generation of the pcl
 */
void revoluteCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud, const int sections = 360);

/*
 * \brief Generates a complete point cloud from a partial point cloud
 *
 * Uses symmetry analysis to find the axis of rotation of a point cloud
 * and generate a complete version of it. The completed point cloud is in a local CS.
 * The pose is returned as a transformation.
 *
 * \param partialCloud  The partial point cloud that needs completion (works only for rotationally symmetric objects)
 * \param completeCloud The completed point cloud after finding the symmetry information
 * \param numSections   The number of cross-sections to use in the reconstruction
 * \param thresh        The threshold for inliers (RANSAC)
 * \return Returns the pose of the object
 */
Eigen::Affine3f generateCompletePointCloudFromSymmetry(pcl::PointCloud<pcl::PointXYZ>::Ptr partialCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr completeCloud, int numSections = 360, double thresh = 0.01);

#endif
