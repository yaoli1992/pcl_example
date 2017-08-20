/*

均匀采样在一个给定的点云分成三维网格在，进行下采样+过滤数据。

均匀采样对输入的点云数据创建一个三维体素网格（想像一个像素网格在空间中的一组小的立体盒子）。然后，在每个体素，将小盒子中所有的点，近似（即采样）的质心。这种方法比用体素中心逼近它们要慢一些，但它更精确地表示下采样。
*/

/*
#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
int main(int argc,char** argv)
{
// 新建点云存储对象
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

	// 读取文件
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}
	// 滤波对象
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	//建立搜索对象
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
	filter.setSearchMethod(kdtree);
	//设置搜索邻域的半径为3cm
	filter.setSearchRadius(0.03);
	// Upsampling 采样的方法有 DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
	filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
	// 采样的半径是
	filter.setUpsamplingRadius(0.03);
	// 采样步数的大小
	filter.setUpsamplingStepSize(0.02);

	filter.process(*filteredCloud);

       //可视化
  pcl::visualization::CloudViewer viewer_cloud("original viewer");

  viewer_cloud.showCloud(cloud);
 //区别一下两中不同的显示方式
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("sampling"));
viewer->addPointCloud<pcl::PointXYZ>(filteredCloud,"unsampling");

while(!viewer->wasStopped())
  {
 viewer->spinOnce(100);
boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
 }
}

*/
/*深度传感器的测量是不准确的，和由此产生的点云也是存在的测量误差，比如离群点，孔等表面，可以用一个算法重建表面，遍历所有的点云和插值数据，试图重建原来的表面。比如增采样，PCL使用MLS算法和类。执行这一步是很重要的，因为由此产生的点云的法线将更准确。
*/
#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
int
main(int argc, char** argv)
{
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);	
	pcl::PointCloud<pcl::PointNormal>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointNormal>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// Smoothing object (we choose what point types we want as input and output).
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> filter;
	filter.setInputCloud(cloud);
	// Use all neighbors in a radius of 3cm.
	filter.setSearchRadius(0.03);
	// If true, the surface and normal are approximated using a polynomial estimation
	// (if false, only a tangent one).
	filter.setPolynomialFit(true);
	// We can tell the algorithm to also compute smoothed normals (optional).
	filter.setComputeNormals(true);
	// kd-tree object for performing searches.
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
	filter.setSearchMethod(kdtree);

	filter.process(*smoothedCloud);


  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("smooth"));
viewer->addPointCloud<pcl::PointNormal>(smoothedCloud,"smoothed");

while(!viewer->wasStopped())
  {
 viewer->spinOnce(100);
boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
 }
}




















