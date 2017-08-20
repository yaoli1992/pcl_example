#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZRGBA>);

	// Read two PCD files from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(argv[1], *cloudA) != 0)
	{
		return -1;
	}
/*if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(argv[2], *cloudB) != 0)
	{
		return -1;
	}

*/

        pcl::visualization::CloudViewer viewerA("viewerA");
         viewerA.showCloud(cloudA);
        //pcl::visualization::CloudViewer viewerB("viewerB");
        // viewerB.showCloud(cloudB);
        while (!viewerA.wasStopped())
	{
	}
}

