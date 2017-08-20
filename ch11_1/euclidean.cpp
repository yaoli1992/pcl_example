/*
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream>
#include <pcl/visualization/cloud_viewer.h>

int
main(int argc, char** argv)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// kd-tree object for searches.
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(cloud);

	// Euclidean clustering object.
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
	// Set cluster tolerance to 2cm (small values may cause objects to be divided
	// in several clusters, whereas big values may join objects in a same cluster).
	clustering.setClusterTolerance(0.02);
	// Set the minimum and maximum number of points that a cluster can have.
	clustering.setMinClusterSize(50);
	clustering.setMaxClusterSize(1000);
	clustering.setSearchMethod(kdtree);
	clustering.setInputCloud(cloud);
	std::vector<pcl::PointIndices> clusters;
	clustering.extract(clusters);

	// For every cluster...
	int currentClusterNum = 1;
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{
		// ...add all its points to a new cloud...
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
			cluster->points.push_back(cloud->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

		// ...and save it to disk.
		if (cluster->points.size() <= 0)
			break;
		std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
		std::string fileName = "cluster" + boost::to_string(currentClusterNum) + ".pcd";
		pcl::io::savePCDFileASCII(fileName, *cluster);

        pcl::visualization::CloudViewer viewer("viewer");
         viewer.showCloud(cluster);
        while(!viewer.wasStopped())
            {
           }
		currentClusterNum++;
	}
}
*/

/*
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <iostream>


//如果此函数返回true，则将添加候选点到种子点的簇类中。
bool
customCondition(const pcl::PointXYZ& seedPoint, const pcl::PointXYZ& candidatePoint, float squaredDistance)
{
	// 在这里你可以添加你自定义的条件
	if (candidatePoint.y < seedPoint.y)
		return false;

	return true;
}

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// 申明一个条件聚类的对象
	pcl::ConditionalEuclideanClustering<pcl::PointXYZ> clustering;
	clustering.setClusterTolerance(0.02);
	clustering.setMinClusterSize(100);
	clustering.setMaxClusterSize(25000);
	clustering.setInputCloud(cloud);
	// 设置要检查每对点的函数。
	clustering.setConditionFunction(&customCondition);
	std::vector<pcl::PointIndices> clusters;
	clustering.segment(clusters);

	// 对于每一个聚类结果
	int currentClusterNum = 1;
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{
		// ...add all its points to a new cloud...
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
			cluster->points.push_back(cloud->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

		// ...and save it to disk.
		if (cluster->points.size() <= 0)
			break;
		std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
		std::string fileName = "cluster" + boost::to_string(currentClusterNum) + ".pcd";
		pcl::io::savePCDFileASCII(fileName, *cluster);

		currentClusterNum++;
	}
}



*/
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <iostream>
#include <pcl/segmentation/region_growing_rgb.h>

int
main(int argc, char** argv)
{
	//申明点云的类型
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// 法线
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}
       // 申明一个Min-cut的聚类对象
	pcl::MinCutSegmentation<pcl::PointXYZ> clustering;
	clustering.setInputCloud(cloud);   //设置输入
        //创建一个点云，列出所知道的所有属于对象的点 
	// （前景点）在这里设置聚类对象的中心点（想想是不是可以可以使用鼠标直接选择聚类中心点的方法呢？）
	pcl::PointCloud<pcl::PointXYZ>::Ptr foregroundPoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointXYZ point;
	point.x = 1.5;
	point.y = 1.5;
	point.z = 0.7;
	foregroundPoints->points.push_back(point);
	clustering.setForegroundPoints(foregroundPoints);  //设置聚类对象的前景点
       
        //设置sigma，它影响计算平滑度的成本。它的设置取决于点云之间的间隔（分辨率）
	clustering.setSigma(0.02);
	// 设置聚类对象的半径.
	clustering.setRadius(0.01);

         //设置需要搜索的临近点的个数，增加这个也就是要增加边界处图的个数
	clustering.setNumberOfNeighbours(10);

        //设置前景点的权重（也就是排除在聚类对象中的点，它是点云之间线的权重，）
	clustering.setSourceWeight(0.6);

	std::vector <pcl::PointIndices> clusters;
	clustering.extract(clusters);

	std::cout << "Maximum flow is " << clustering.getMaxFlow() << "." << std::endl;

	int currentClusterNum = 1;
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{
		//设置聚类后点云的属性
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
			cluster->points.push_back(cloud->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

	       //保存聚类的结果
		if (cluster->points.size() <= 0)
			break;
		std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
		std::string fileName = "cluster" + boost::to_string(currentClusterNum) + ".pcd";
		pcl::io::savePCDFileASCII(fileName, *cluster);

		currentClusterNum++;
	}
}
/*
	// kd-tree object for searches.
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(cloud);

	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.03);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	// Region growing clustering object.
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> clustering;
	clustering.setMinClusterSize(100);
	clustering.setMaxClusterSize(10000);
	clustering.setSearchMethod(kdtree);
	clustering.setNumberOfNeighbours(30);
	clustering.setInputCloud(cloud);
	clustering.setInputNormals(normals);
	// Set the angle in radians that will be the smoothness threshold
	// (the maximum allowable deviation of the normals).
	clustering.setSmoothnessThreshold(7.0 / 180.0 * M_PI); // 7 degrees.
	// Set the curvature threshold. The disparity between curvatures will be
	// tested after the normal deviation check has passed.
	clustering.setCurvatureThreshold(1.0);

	std::vector <pcl::PointIndices> clusters;
	clustering.extract(clusters);

	// For every cluster...
	int currentClusterNum = 1;
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{
		// ...add all its points to a new cloud...
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
			cluster->points.push_back(cloud->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

		// ...and save it to disk.
		if (cluster->points.size() <= 0)
			break;
		std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
		std::string fileName = "cluster" + boost::to_string(currentClusterNum) + ".pcd";
		pcl::io::savePCDFileASCII(fileName, *cluster);

		currentClusterNum++;
	}
}
*/

