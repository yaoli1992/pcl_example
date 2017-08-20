#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>

#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_stick.h>

int main(int argc,char** argv)
{
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZRGBA>);

if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(argv[1],*cloud)!=0)
{
return -1 ;
}

pcl::NormalEstimation<pcl::PointXYZRGBA,pcl::Normal> normalEstimation;
normalEstimation.setInputCloud(cloud);
//对于每一个点都用半径为3cm的近邻搜索方式
normalEstimation.setRadiusSearch(0.03);
//Kd_tree是一种数据结构便于管理点云以及搜索点云，法线估计对象会使用这种结构来找到哦啊最近邻点
pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
normalEstimation.setSearchMethod(kdtree);

//计算法线
normalEstimation.compute(*normals);
//RANSAC object
//球形模型
pcl::SampleConsensusModelSphere<pcl::PointXYZRGBA>::Ptr model(new pcl::SampleConsensusModelSphere<pcl::PointXYZRGBA>(cloud));
//平面模型
pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA> (cloud));
//2D圆模型A 2D circle on the X-Y plane.
pcl::SampleConsensusModelCircle2D<pcl::PointXYZRGBA>::Ptr model_2d (new pcl::SampleConsensusModelCircle2D<pcl::PointXYZRGBA> (cloud));
//A 3D circle (any plane).  defines a model for 3D circle segmentation.
pcl::SampleConsensusModelCircle3D<pcl::PointXYZRGBA>::Ptr model_3d (new pcl::SampleConsensusModelCircle3D<pcl::PointXYZRGBA> (cloud));
//a clone 
//pcl::SampleConsensusModelCone<pcl::PointXYZRGBA>::Ptr model_clone (new pcl::SampleConsensusModelCone<pcl::PointXYZRGBA> (cloud), new pcl::SampleConsensusModelCone<pcl::Normal> (normals));
//
//pcl::SampleConsensusModelCylinder<pcl::PointXYZRGBA>::Ptr model_cy (new pcl::SampleConsensusModelCylinder<pcl::PointXYZRGBA> (cloud),pcl::PointCloud<pcl::Normal> (normals));
// a line
pcl::SampleConsensusModelLine<pcl::PointXYZRGBA>::Ptr model_line(new pcl::SampleConsensusModelLine<pcl::PointXYZRGBA> (cloud));
//a stick 
pcl::SampleConsensusModelStick<pcl::PointXYZRGBA>::Ptr model_s(new pcl::SampleConsensusModelStick<pcl::PointXYZRGBA> (cloud));

pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac(model_line);
ransac.setDistanceThreshold(0.1);
ransac.computeModel();

std::vector<int> inlierIndices;
ransac.getInliers(inlierIndices);

pcl::copyPointCloud<pcl::PointXYZRGBA>(*cloud, inlierIndices, *inlierPoints);
//pcl::copyPointCloud<pcl::PointXYZRGBA>(*cloud, inlierIndices, *inlierPoints);
  pcl::visualization::CloudViewer viewer_cloud("inlierPoints");

  viewer_cloud.showCloud(inlierPoints);
while(!viewer_cloud.wasStopped())
{

}

}













