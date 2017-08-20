#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include<pcl/visualization/histogram_visualizer.h>

int
main(int argc, char** argv)
{
	// Clouds for storing everything.
	pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *object) != 0)
	{
		return -1;
	}

	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(object);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	// Estimate VFH descriptor.
	pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud(object);
	vfh.setInputNormals(normals);
	vfh.setSearchMethod(kdtree);
	vfh.setNormalizeBins(true);
	vfh.setNormalizeDistance(false);
	vfh.compute(*descriptor);

	// Plotter object.
	pcl::visualization::PCLHistogramVisualizer viewer;
	// We need to set the size of the descriptor beforehand.
	viewer.addFeatureHistogram(*descriptor, 308);

	viewer.spin();
}
