#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ascii_io.h>


int
  main (int argc, char** argv)
{
  pcl::ASCIIReader reader;

  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  //reader.readHeader("zxx33_0.pcd", cloud);
  

  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("zxx33_0.pcd", cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "<< cloud.width * cloud.height<< " data points" << std::endl;
  std::cout << "Loaded "<< cloud.fields<< " data points" << std::endl;
 // pcl::ASCIIReader::setInputFields("INT8");

  pcl::PointCloud<pcl::PointXYZRGB> cloud_1;
 // &cloud_1=&cloud;
/*
  // Fill in the cloud data
 // cloud_1.width    = cloud.width;
  cloud_1.height   = 1;
  cloud_1.is_dense = cloud.is_dense;
  //cloud_1.points.resize (cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud_1.points[i].x = cloud.points[i].x;
    cloud_1.points[i].y = cloud.points[i].y;
    cloud_1.points[i].z = cloud.points[i].z;
   // cloud_1.points[i].r = cloud.points[i].r;
    //cloud_1.points[i].g = cloud.points[i].g;
    //cloud_1.points[i].b = cloud.points[i].b;
  }
*/
  //pcl::io::savePCDFileASCII ("rgb_pcd.pcd", cloud_1);
  //std::cerr << "Saved " << cloud_1.points.size () << " data points ." << std::endl;


  return (0);
}


