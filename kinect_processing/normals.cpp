#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>


int main(int argc,char**argv)
{
//创建点云对象
pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>);
//创建法线的对象
pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//读取PCD文件
if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1],*cloud) !=0)
{
 return -1;
}
//创建法线估计的对象
pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normalEstimation;
normalEstimation.setInputCloud(cloud);
//对于每一个点都用半径为3cm的近邻搜索方式
normalEstimation.setRadiusSearch(0.03);
//Kd_tree是一种数据结构便于管理点云以及搜索点云，法线估计对象会使用这种结构来找到哦啊最近邻点
pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
normalEstimation.setSearchMethod(kdtree);

//计算法线
normalEstimation.compute(*normals);
//可视化
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
viewer->addPointCloud<pcl::PointXYZ>(cloud,"cloud");

while(!viewer->wasStopped())
  {
 viewer->spinOnce(100);
boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
 }

}


