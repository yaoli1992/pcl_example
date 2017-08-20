#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>

int main(int argc, char** argv)
{
    char tmpStr[100];
    strcpy(tmpStr,argv[1]);
    char* pext = strrchr(tmpStr, '.');
    std::string extply("ply");
    std::string extpcd("pcd");
    if(pext){
        *pext='\0';
        pext++;
    }
    std::string ext(pext);

    if (!((ext == extply)||(ext == extpcd))){
        std::cout << "文件格式不支持!" << std::endl;
        std::cout << "支持的文件格式*.pcd .ply" << std::endl;
        return(-1);
    }


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>) ; 
    if (ext == extply){
        if (pcl::io::loadPLYFile(argv[1] , *cloud) == -1){
            PCL_ERROR("Could not read ply file!\n") ;
            return -1;
        }
    }
    else{
        if (pcl::io::loadPCDFile(argv[1] , *cloud) == -1){
            PCL_ERROR("Could not read pcd file!\n") ;
            return -1;
        }
    }


  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>); //
  pcl::NormalEstimation<pcl::PointXYZ , pcl::Normal> n ;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>) ;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>) ;
  tree->setInputCloud(cloud) ;
  n.setInputCloud(cloud) ;
  n.setSearchMethod(tree) ;
  n.setKSearch(20);
    n.compute(*normals);


  pcl::concatenateFields(*cloud , *normals , *cloud_with_normals) ;


  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>) ;
  tree2->setInputCloud(cloud_with_normals) ;

  pcl::Poisson<pcl::PointNormal> pn ;
    pn.setConfidence(false); 
    pn.setDegree(2);
    pn.setDepth(8); 
    pn.setIsoDivide(8); 
    pn.setManifold(false); 
    pn.setOutputPolygons(false); 
    pn.setSamplesPerNode(3.0); 
    pn.setScale(1.25); 
    pn.setSolverDivide(8); 
    //pn.setIndices();

  
    pn.setSearchMethod(tree2);
    pn.setInputCloud(cloud_with_normals);
 
    pcl::PolygonMesh mesh ;

    pn.performReconstruction(mesh);

   
    pcl::io::savePLYFile("result.ply", mesh);


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer")) ;
    viewer->setBackgroundColor(0 , 0 , 0) ;
    viewer->addPolygonMesh(mesh , "my") ;
    viewer->addCoordinateSystem (50.0);
    viewer->initCameraParameters() ;
    while (!viewer->wasStopped()){
        viewer->spinOnce(100) ;
        boost::this_thread::sleep(boost::posix_time::microseconds(100000)) ;
    }

    return 0;
}
