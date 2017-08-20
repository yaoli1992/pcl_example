#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>

int main(int argc,char** argv)
{
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PCLPointCloud2::Ptr cloud_grid (new pcl::PCLPointCloud2 ());
pcl::PCLPointCloud2::Ptr cloud_grid_filtered (new pcl::PCLPointCloud2 ());

cloud->width=10;
cloud->height=1;
cloud->points.resize(cloud->width * cloud->height);

for(size_t i=0;i<cloud->points.size();i++)
 {
 cloud->points[i].x = 1024*rand()/(RAND_MAX +1.0f);
 cloud->points[i].y = 1024*rand()/(RAND_MAX +1.0f);
 cloud->points[i].z = 1024*rand()/(RAND_MAX +1.0f);
 }
std::cerr <<"Cloud before filter:  "<<std::endl;

for(size_t i=0;i <cloud->points.size();i++)
 
  std::cerr <<"  " <<cloud->points[i].x <<"  "
                   <<cloud->points[i].y <<"  "
		   <<cloud->points[i].z <<std::endl;
/*****************直通滤波***************************************************************/
/****************************************************************************************/
pcl::PassThrough<pcl::PointXYZ> pass;
pass.setInputCloud(cloud);
pass.setFilterFieldName("z");
pass.setFilterLimits (0.0,1.0);
pass.setFilterLimitsNegative(true);  //选择的范围是否为负范围
pass.filter (*cloud_filter);

std::cerr <<"Cloud after filter:  "<<std::endl;

for(size_t i=0;i <cloud_filter->points.size();i++)
 
  std::cerr <<"  " <<cloud_filter->points[i].x <<"  "
                   <<cloud_filter->points[i].y <<"  "
		   <<cloud_filter->points[i].z <<std::endl;
/**********************************************************************************/
/***********************************************************************************/

pcl::PCDWriter writer;
writer.write<pcl::PointXYZ>("cloud.pcd",*cloud,false);
 /*writer.write ("cloud.pcd", *cloud, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
*/
pcl::PCDReader reader;
reader.read <pcl::PointXYZ>("table_scene_lms400.pcd",*cloud);
//体素滤波
pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
sor.setInputCloud(cloud_grid);
sor.setLeafSize (0.01f,0.01f,0.01f);
sor.filter (*cloud_grid_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_grid_filtered->width * cloud_grid_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_grid_filtered) << ").";
/****************************************************************************************************/
/*****************************平面投影滤波************************************************************/
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());

coefficients->values.resize(4);
coefficients->values[0] = coefficients->values[1]=0;
coefficients->values[2] =1.0;
coefficients->values[3] =0.0;
//create the filtering object
pcl::ProjectInliers<pcl::PointXYZ> proj;
proj.setModelType (pcl::SACMODEL_PLANE);
proj.setInputCloud(cloud);
proj.setModelCoefficients (coefficients);
proj.filter (*cloud_filter);

std::cerr <<"Cloud after project:  "<<std::endl;

for(size_t i=0;i <cloud_filter->points.size();i++)
 
  std::cerr <<"  " <<cloud_filter->points[i].x <<"  "
                   <<cloud_filter->points[i].y <<"  "
		   <<cloud_filter->points[i].z <<std::endl;
return (0);
}












