#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/png_io.h>

int main(int argc,char** argv)
{
if(argc !=3)
 {
  std::cout <<"\tUsage: "<<argv[0] <<"<input cloud> <output cloud>" <<std::endl;

  return  -1;
 }

//object for string the point cloud
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
//read a PCDfile from disk
if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(argv[1],*cloud) !=0)
{
 return -1;
}
// Save the image (cloud must be organized).
//pcl::io::savePNGFile("output.png", *cloud, "rgb");

//the mapping tells you to that points of the oldcloud the new ones correspond
//but we  will not use it
std::vector<int> mapping;
pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);
//pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);
//save it back
pcl::io::savePCDFileASCII(argv[2],*cloud);

pcl::visualization::CloudViewer viewer(argv[1]);
         viewer.showCloud(cloud);
        while (!viewer.wasStopped())
	{
		// Do nothing but wait.
	}

}
