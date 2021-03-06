#include "symmetry.h"

pcl::PointXYZ VectorEigenToPointXYZ(Eigen::Vector3f v)
{
    pcl::PointXYZ p;
    p.x = v[0];
    p.y = v[1];
    p.z = v[2];
    return p;
}

pcl::PointXYZ VectorEigenToPointXYZ(Eigen::Vector4f v)
{
    pcl::PointXYZ p;
    p.x = v[0];
    p.y = v[1];
    p.z = v[2];
    return p;
}

void estimateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr outputNormals, const double radius)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (radius);

  // Compute the features
  ne.compute (*outputNormals);
}

void generateSymmetryAxisHypotheses(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud, const int numSamples)
{
  // vector containing all intersection points
  std::vector<Eigen::Vector4f>   centroids;

  Eigen::VectorXf lineA(6);
  Eigen::VectorXf lineB(6);

  int size = cloud->width;

  for( int k = 0; k < numSamples; k++ )
  {
    int i = rand() % size;
    int j = rand() % size;

    // points
    lineA.block<3,1>(0,0) = (*cloud)[i].getVector3fMap();
    lineB.block<3,1>(0,0) = (*cloud)[j].getVector3fMap();

    // normals
    lineA[3] = (*normals)[i].normal_x;
    lineA[4] = (*normals)[i].normal_y;
    lineA[5] = (*normals)[i].normal_z;

    lineB[3] = (*normals)[j].normal_x;
    lineB[4] = (*normals)[j].normal_y;
    lineB[5] = (*normals)[j].normal_z;

    // intersect lines
    Eigen::Vector4f center;
    bool foundInt = pcl::lineWithLineIntersection(lineA, lineB, center);

    // add new point to our centroids
    if(foundInt)
    {
      // probably a wrong intersection
      if(center[0] == 0 || center[1] == 0 || center[2] == 0)
              continue;

      // print hypotheses
      if(DEBUG) std::cout << "[LOG] Hypothesis for axis of symmetry: " << center[0] << " " << center[1] << " " << center[2] << std::endl;
      centroids.push_back(center);
    }
  }
  // Fill in the cloud data
  outCloud->width    = centroids.size();
  outCloud->height   = 1;
  outCloud->is_dense = false;
  outCloud->points.resize (outCloud->width * outCloud->height);

  for(int k =0; k < outCloud->width; k++)
  {
    // fill point cloud
    outCloud->points[k] = VectorEigenToPointXYZ(centroids[k]);
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr projectOnPrincipleCS(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector3f origin, Eigen::Vector3f yAxis, Eigen::Affine3f& transf)
{
  // 1. calculate the coordinate system
  Eigen::Vector3f zAxis(0,0,1);
  Eigen::Vector3f xAxis = yAxis.cross(zAxis);
  zAxis = xAxis.cross(yAxis);

  // create a rotation matrix
  transf.linear() << xAxis, yAxis, zAxis;
  transf.translation() = origin;

  // create inverse matrix
  Eigen::Affine3f invM = transf.inverse();

  // create a transformed matrix
  // apply transformation to all points
  pcl::PointCloud<pcl::PointXYZ>::Ptr transfCloud (new pcl::PointCloud<pcl::PointXYZ> ());

  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*cloud, *transfCloud, invM);

  return transfCloud;
}

void computePrincipalAxis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud, double distThresh,
    Eigen::Vector3f& axisOrigin, Eigen::Vector3f& axisDir)
{
  // created RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud));

  // the inliers of the model
  std::vector<int> inliers;

  // apply line fitting
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model);
  ransac.setDistanceThreshold (distThresh);
  ransac.computeModel();
  ransac.getInliers(inliers);

  // copies all inliers of the model computed to another PointCloud
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *outCloud);

  // compute model coefficients'
  Eigen::VectorXf coefficients;
  ransac.getModelCoefficients(coefficients);
  Eigen::VectorXf coeff_refined = coefficients;

  // calculate origin and direction of principal axis
  axisOrigin  = Eigen::Vector3f(coeff_refined[0], coeff_refined[1], coeff_refined[2]);
  axisDir = Eigen::Vector3f(coeff_refined[3], coeff_refined[4], coeff_refined[5]);

  // normalize to get direction
  axisDir.normalize ();

  // analyze computed axis
  bool multiAxis = checkMutiAxisSymmetry(cloud);

  if(multiAxis)
  {
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);

    axisDir = Eigen::Vector3f(0,1,0);
    axisOrigin = pca.getMean().head(3);
  }
}

bool checkMutiAxisSymmetry(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const double thresh)
{
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cloud);

  // get the first eigenvalue
  Eigen::Vector3f evs = pca.getEigenValues();

  // check length
  std::cout << "[LOG] First Eigenvalue: " << evs[0] << std::endl;

  // return true if points are condensed
  if(evs[0] < thresh)
    return true;

  else
    return false;
}

void unrevoluteCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  Eigen::Vector3f xAxis(1,0,0);
  Eigen::Vector3f yAxis(0,1,0);

  Eigen::Affine3f t = Eigen::Affine3f::Identity();

  // unrevolute every point by rotating it back in the xz plane
  for( int i = 0; i < cloud->width; i++ )
  {
    // rotation angle
    Eigen::Vector3f p = ((*cloud)[i].getVector3fMap());
    p[1] = 0.0;
    p.normalize();

    // calculate angle between x-axis and projected point
    double angle = xAxis.dot(p);
    if(angle > M_PI)
    {
      angle = angle - 2.0* M_PI;
    }
    // rotate point back by that angle
    t.linear() = ( Eigen::AngleAxisf(-acos(angle), xAxis.cross(p).normalized()) ).toRotationMatrix();
    Eigen::Vector3f pnew = t * (*cloud)[i].getVector3fMap();

    (*cloud)[i] = VectorEigenToPointXYZ(pnew);
  }
}

void revoluteCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud, const int sections)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr transfCloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.02f, 0.01f, 0.01f);
  sor.filter (*outCloud);

  int origCloudSize = outCloud->width;

  // resize cloud
  outCloud->points.resize (outCloud->width * sections);

  // coordinate system
  Eigen::Vector3f xAxis(1,0,0);
  Eigen::Vector3f yAxis(0,1,0);

  // create transformation
  Eigen::Affine3f t = Eigen::Affine3f::Identity();

  // unrevolute every point by rotating it back in the xz plane
  for( int k = 1; k < sections; k++ )
  {
    double angle = k * (360.0/(double)sections) * (M_PI/180.0);

    for( int i = 0; i < origCloudSize; i++ )
    {
      // rotate point back by that angle
      t.linear() = ( Eigen::AngleAxisf(angle, yAxis) ).toRotationMatrix();
      Eigen::Vector3f pnew = t * (*outCloud)[i].getVector3fMap();

      (*outCloud)[k * origCloudSize + i] = VectorEigenToPointXYZ(pnew);
    }
  }
}

Eigen::Affine3f generateCompletePointCloudFromSymmetry(pcl::PointCloud<pcl::PointXYZ>::Ptr partialCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr completeCloud, int numSections, double thresh)
{
  // estimate the normals of the cloud
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  estimateNormals(partialCloud, normals);

  // intersect lines and create hypotheses for axis of symmetry
  pcl::PointCloud<pcl::PointXYZ>::Ptr hypos( new pcl::PointCloud<pcl::PointXYZ>() );
  generateSymmetryAxisHypotheses(partialCloud, normals, hypos);

  // filter the centers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (hypos);
  sor.setMeanK (5);
  sor.setStddevMulThresh (0.1);
  sor.filter (*cloudFiltered);

  // inliers hypotheses that could be potential axis of symmetries
  pcl::PointCloud<pcl::PointXYZ>::Ptr axisHypotheses (new pcl::PointCloud<pcl::PointXYZ>);

  // estimate axis of rotation
  Eigen::Vector3f lPos, lAxis;
  computePrincipalAxis(cloudFiltered, axisHypotheses, thresh, lPos, lAxis);

  // project to generate pcl in canonical CS
  Eigen::Affine3f pose;
  pcl::PointCloud<pcl::PointXYZ>::Ptr projectedPcl = projectOnPrincipleCS(partialCloud, lPos, lAxis, pose);

  // unrevolute point cloud
  unrevoluteCloud(projectedPcl);

  // revolute to generate complete pcl
  revoluteCloud(projectedPcl, completeCloud, numSections);

  return pose;
}
