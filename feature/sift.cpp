#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>  
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/vfh.h>
#include <pcl/search/kdtree.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>

#include <string>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
/* Define some custom types to make the rest of our code easier to read */

// Define "PointCloud" to be a pcl::PointCloud of pcl::PointXYZRGB points
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

// Define "SurfaceNormals" to be a pcl::PointCloud of pcl::Normal points
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> SurfaceNormals;
typedef pcl::PointCloud<NormalT>::Ptr SurfaceNormalsPtr;
typedef pcl::PointCloud<NormalT>::ConstPtr SurfaceNormalsConstPtr;

// Define "LocalDescriptors" to be a pcl::PointCloud of pcl::FPFHSignature33 points
typedef pcl::FPFHSignature33 LocalDescriptorT;
typedef pcl::PointCloud<LocalDescriptorT> LocalDescriptors;
typedef pcl::PointCloud<LocalDescriptorT>::Ptr LocalDescriptorsPtr;
typedef pcl::PointCloud<LocalDescriptorT>::ConstPtr LocalDescriptorsConstPtr;

// Define "GlobalDescriptors" to be a pcl::PointCloud of pcl::VFHSignature308 points
typedef pcl::VFHSignature308 GlobalDescriptorT;
typedef pcl::PointCloud<GlobalDescriptorT> GlobalDescriptors;
typedef pcl::PointCloud<GlobalDescriptorT>::Ptr GlobalDescriptorsPtr;
typedef pcl::PointCloud<GlobalDescriptorT>::ConstPtr GlobalDescriptorsConstPtr;



boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}

SurfaceNormalsPtr
estimateSurfaceNormals (const PointCloudPtr & input, float radius)
{
  pcl::NormalEstimation<PointT, NormalT> normal_estimation;
  normal_estimation.setSearchMethod (pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
  normal_estimation.setRadiusSearch (radius);   //设置搜索半径
  normal_estimation.setInputCloud (input);       //设置输入点云
  SurfaceNormalsPtr normals (new SurfaceNormals); //SurfaceNormals就是 pcl::PointCloud<NormalT>
  normal_estimation.compute (*normals);

  return (normals);
}

PointCloudPtr
detectKeypoints (const PointCloudPtr & points, const SurfaceNormalsPtr & normals,
                 float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast)
{
  pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift_detect;
  sift_detect.setSearchMethod (pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
   //设置检测的方式是Kdtree方式
  sift_detect.setScales (min_scale, nr_octaves, nr_scales_per_octave);
  sift_detect.setMinimumContrast (min_contrast);
  sift_detect.setInputCloud (points);     //输入的点云
  pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
  sift_detect.compute (keypoints_temp);
  PointCloudPtr keypoints (new PointCloud);
  pcl::copyPointCloud (keypoints_temp, *keypoints);

  return (keypoints);
}


int 
main (int argc, char ** argv)
{

  PointCloudPtr cloud (new PointCloud);
  pcl::io::loadPCDFile (argv[1], *cloud);
  pcl::console::print_info ("Loaded %s (%lu points)\n", argv[1], cloud->size ());

  SurfaceNormalsPtr normals;  //法线数据
  double surface_radius=0.01;   //命令里输入的法线搜索的半径
  
  normals = estimateSurfaceNormals (cloud, surface_radius);
 //  pcl::io::savePCDFile ("normal", *normals);  

    PointCloudPtr keypoints;   //申明关键点的结构
  float min_scale =0.01;
  int nr_octaves = 10;
  int nr_scales = 5;
  float min_contrast = 1.0;
  keypoints = detectKeypoints (cloud, normals, min_scale, nr_octaves, nr_scales, min_contrast);
  pcl::io::savePCDFile ("sift.pcd", *keypoints);
 
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVis(cloud);
//      viewer->addPointCloudNormals<PointT,NormalT> (cloud, normals, 4, 0.02, "normals");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (keypoints, 255, 0, 0);
      viewer->addPointCloud (keypoints, red, "keypoints");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "keypoints");    //对关键点着色

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
/*
   pcl::visualization::PCLVisualizer vis;

    vis.addPointCloud (cloud); 
   // vis.addPointCloudNormals<PointT,NormalT> (cloud, normals, 4, 0.02, "normals");
/*
      pcl::visualization::PointCloudColorHandlerCustom<PointT> red (keypoints, 255, 0, 0);
      vis.addPointCloud (keypoints, red, "keypoints");
      vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints");    //对关键点着色
  
 while (!vis.wasStopped()) 
  {
  
  }
*/
return 0;

}




