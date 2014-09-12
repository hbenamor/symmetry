#include "symmetry.h"

/**
 * @function main
 */
int main( int argc, char* argv[] )
{

  if(argc < 2)
  {
    std::cout << "Usage: ./symmetryTest object.pcd" << std::endl;
  }
  // load the point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud( new pcl::PointCloud<pcl::PointXYZ>() );
  pcl::io::loadPCDFile (argv[1], *inputCloud);
  
  // visualize normals
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor (0.0, 0.0, 0.5);

  // complete point cloud and get pose
  pcl::PointCloud<pcl::PointXYZ>::Ptr completedCloud( new pcl::PointCloud<pcl::PointXYZ>() );
  Eigen::Affine3f pose = generateCompletePointCloudFromSymmetry(inputCloud, completedCloud);

  viewer.addPointCloud(inputCloud, "input");
  viewer.addPointCloud(completedCloud, "completed");

  // display coordinate system
  Eigen::Affine3f t(Eigen::Affine3f::Identity());
  t.translation() = Eigen::Vector3f(0,0,0);
  viewer.addCoordinateSystem(0.1, t, "c1", 0);

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }

  return 0;
}

