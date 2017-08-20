#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudC(new pcl::PointCloud<pcl::PointXYZRGBA>);

	// Read two PCD files from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(argv[1], *cloudA) != 0)
	{
		return -1;
	}
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(argv[2], *cloudB) != 0)
	{
		return -1;
	}

	// Create cloud "C", with the points of both "A" and "B".
	*cloudC = (*cloudA) + (*cloudB);
       pcl::io::savePCDFileASCII("add.pcd",*cloudC);
  

        pcl::visualization::CloudViewer viewer("add viewer");
         viewer.showCloud(cloudC);
        while (!viewer.wasStopped())
	{
		// Do nothing but wait.
	}
	// Now cloudC->points.size() equals cloudA->points.size() + cloudB->points.size().
}

