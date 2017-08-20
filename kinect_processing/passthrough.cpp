#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
{

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}

int main(int argc,char** argv)
{
 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(argv[1],*cloud)!=0)
{
 return -1;
}
//filters
pcl::PassThrough<pcl::PointXYZRGBA> filter;
filter.setInputCloud(cloud);
//filter.setFilterLimitsNegative (true);
filter.setFilterFieldName("z");   
filter.setFilterLimits(0.0,0.7);

filter.filter(*filteredCloud);

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVis(filteredCloud);
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
