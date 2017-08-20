
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

int main(int argc,char** argv)
{
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  
//pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCLoud2),cloud_filtered_blob(new pcl::PCLPointCloud2);
pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2),cloud_filtered_blob(new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

 pcl::PCDReader reader;
 reader.read<pcl::PointXYZ> ("table_scene_lms400.pcd",*cloud);

std::cerr << "cloud before filtering " <<std::endl;
std::cerr <<*cloud <<std::endl;
 
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor1;  //体素栅格下采样对象
  sor1.setInputCloud (cloud_blob);             //原始点云
  sor1.setLeafSize (0.01f, 0.01f, 0.01f);    // 设置采样体素大小
  sor1.filter (*cloud_filtered_blob);        //保存
//最关键的一步骤就是格式转化
pcl::fromPCLPointCloud2 (*cloud_filtered_blob,*cloud_filtered);

//pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
sor.setInputCloud(cloud);
sor.setMeanK(50);
sor.setStddevMulThresh(1.0);
sor.filter (*cloud_filtered);

pcl::PCDWriter writer;
writer.write<pcl::PointXYZ>("table_scene_lms400_inliers.pcd",*cloud_filtered,false);

sor.setNegative(true);
sor.filter (*cloud_filtered);
writer.write<pcl::PointXYZ>("table_scene_lms400_outliers.pcd",*cloud_filtered,false);
return (0);



}

