/*#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>

int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// Object for storing the plane model coefficients.
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	// Create the segmentation object.
	pcl::SACSegmentation<pcl::PointXYZ> segmentation;
	segmentation.setInputCloud(cloud);
	// Configure the object to look for a plane.
	//segmentation.setModelType(pcl::SACMODEL_PLANE);
segmentation.setModelType(pcl::SACMODEL_CYLINDER);
	// Use RANSAC method.
	segmentation.setMethodType(pcl::SAC_RANSAC);
	// Set the maximum allowed distance to the model.
	segmentation.setDistanceThreshold(0.01);
	// Enable model coefficient refinement (optional).
	segmentation.setOptimizeCoefficients(true);

	pcl::PointIndices inlierIndices;
	segmentation.segment(inlierIndices, *coefficients);

	if (inlierIndices.indices.size() == 0)
		std::cout << "Could not find any points that fitted the plane model." << std::endl;
	else
	{
		std::cerr << "Model coefficients: " << coefficients->values[0] << " "
				  << coefficients->values[1] << " "
				  << coefficients->values[2] << " "
				  << coefficients->values[3] << std::endl;

		// Copy all inliers of the model to another cloud.
		pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inlierIndices, *inlierPoints);
  pcl::visualization::CloudViewer viewer(" viewer");
         viewer.showCloud(inlierPoints);
        while (!viewer.wasStopped())
	{}

	}
}
*/

#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>

int
main(int argc, char** argv)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// Object for storing the plane model coefficients.
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	// Create the segmentation object.
	pcl::SACSegmentation<pcl::PointXYZ> segmentation;
	segmentation.setInputCloud(cloud);
	// Configure the object to look for a plane.
	segmentation.setModelType(pcl::SACMODEL_CYLINDER);
	// Use RANSAC method.
	segmentation.setMethodType(pcl::SAC_RANSAC);
	// Set the maximum allowed distance to the model.
	segmentation.setDistanceThreshold(0.01);
	// Enable model coefficient refinement (optional).
	segmentation.setOptimizeCoefficients(true);
	// Set minimum and maximum radii of the cylinder.
	segmentation.setRadiusLimits(0, 0.1);

	pcl::PointIndices inlierIndices;
	segmentation.segment(inlierIndices, *coefficients);

	if (inlierIndices.indices.size() == 0)
		std::cout << "Could not find any points that fitted the cylinder model." << std::endl;
	// Copy all inliers of the model to another cloud.
	else pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inlierIndices, *inlierPoints);
 pcl::visualization::CloudViewer viewer(" viewer");
         viewer.showCloud(inlierPoints);
        while (!viewer.wasStopped())
	{}
}

