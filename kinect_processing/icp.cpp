#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <iostream>

int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Read two PCD files from disk.
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(argv[1], *sourceCloud) != 0)
	{
		return -1;
	}
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(argv[2], *targetCloud) != 0)
	{
		return -1;
	}

	// ICP object.
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> registration;
	registration.setInputSource(sourceCloud);
	registration.setInputTarget(targetCloud);

	registration.align(*finalCloud);
     
        pcl::visualization::CloudViewer viewer("viewer");

        viewer.showCloud(finalCloud);
  

	if (registration.hasConverged())
	{
		std::cout << "ICP converged." << std::endl
				  << "The score is " << registration.getFitnessScore() << std::endl;
		std::cout << "Transformation matrix:" << std::endl;
		std::cout << registration.getFinalTransformation() << std::endl;
	}
	else std::cout << "ICP did not converge." << std::endl;

         while(!viewer.wasStopped())
       {
   
       }
}
