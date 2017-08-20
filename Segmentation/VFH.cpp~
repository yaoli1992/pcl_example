#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/io/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/vfh.h>
#include <pcl/search/kdtree.h>

#include <vector>
#include <string>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
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

// Define "GlobalDescriptors" to be a pcl::PointCloud of pcl::VFHSignature308 points
typedef pcl::VFHSignature308 GlobalDescriptorT;
typedef pcl::PointCloud<GlobalDescriptorT> GlobalDescriptors;
typedef pcl::PointCloud<GlobalDescriptorT>::Ptr GlobalDescriptorsPtr;
typedef pcl::PointCloud<GlobalDescriptorT>::ConstPtr GlobalDescriptorsConstPtr;


/* Use NormalEstimation to estimate a cloud's surface normals 
 * Inputs:
 *   input
 *     The input point cloud
 *   radius
 *     The size of the local neighborhood used to estimate the surface
 * Return: A pointer to a SurfaceNormals point cloud
 */
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

/* Use VFHEstimation to compute a single global descriptor for the entire input cloud
 * Inputs:  使用VFHEstimation计算全局描述子
 *   points
 *     The input point cloud
 *   normals
 *     The input surface normals
 * Return: A pointer to a GlobalDescriptors point cloud (a cloud containing a single GlobalDescriptorT point)
 */
GlobalDescriptorsPtr
computeGlobalDescriptor (const PointCloudPtr & points, const SurfaceNormalsPtr & normals)
{
  pcl::VFHEstimation<PointT, NormalT, GlobalDescriptorT> vfh_estimation;
  vfh_estimation.setSearchMethod (pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
  vfh_estimation.setInputCloud (points);
  vfh_estimation.setInputNormals (normals);
  GlobalDescriptorsPtr global_descriptor (new GlobalDescriptors);
  vfh_estimation.compute (*global_descriptor);

  return (global_descriptor);
}

int 
main (int argc, char ** argv)
{
  if (argc < 2) 
  {
    pcl::console::print_info ("Syntax is: %s input.pcd <options>\n", argv[0]);
    pcl::console::print_info ("  where options are:\n");
    pcl::console::print_info ("    -n radius  ...................................... Estimate surface normals\n");
    pcl::console::print_info ("    -s output_name (without .pcd extension).......... Save outputs\n");
    return (1);
  }
  

  // 载入文件
  PointCloudPtr cloud (new PointCloud);
  pcl::io::loadPCDFile (argv[1], *cloud);
  pcl::console::print_info ("Loaded %s (%lu points)\n", argv[1], cloud->size ());
  
  //估算表面的法线
  SurfaceNormalsPtr normals;  //法线数据
  double surface_radius;   //命令里输入的法线搜索的半径
  bool estimate_surface_normals = pcl::console::parse_argument (argc, argv, "-n", surface_radius) > 0;
  
  if (estimate_surface_normals)
  {  
    normals = estimateSurfaceNormals (cloud, surface_radius);
    pcl::console::print_info ("Estimated surface normals\n");
  }
  
  
  // 计算全局描述子
  GlobalDescriptorsPtr global_descriptor;  //申明全局描述子
  if (normals)//如果法线不为假值
  {
    global_descriptor = computeGlobalDescriptor (cloud, normals);//利用点云以及法线就可以计算全局描述子
    pcl::console::print_info ("Computed global descriptor\n");
  }

  // 保存输出
  std::string base_filename, output_filename;//base_filename命令行输入的没有后缀的文件名
  bool save_cloud = pcl::console::parse_argument (argc, argv, "-s", base_filename) > 0;
  if (save_cloud)
  {
   
      output_filename = base_filename;
      output_filename.append ("_vfh.pcd");
      pcl::io::savePCDFile (output_filename, *global_descriptor);
      pcl::console::print_info ("Saved global descriptor as %s\n", output_filename.c_str ());
    }


  }


