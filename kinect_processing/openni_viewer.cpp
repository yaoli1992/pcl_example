/*******************************************************************/
//关于实现对Kinect获取的点云进行处理
//时间：2017。03.05
//作者：姚利
/*************************************************************/
#include <iostream>
#include <string>
#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

class SimpleOpenNIViewer
{     
  public:
     SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}  //显示可视化界面的title

     void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) //回调函数
     {  
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_grid (new pcl::PointCloud<pcl::PointXYZRGBA> ());   
       if (!viewer.wasStopped())  //如果没有按下结束按键
       {
      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor; 
      pcl::VoxelGrid<pcl::PointXYZRGBA> sor_grid;  //创建滤波对象
     sor.setInputCloud (cloud);            //设置需要过滤的点云给滤波对象
      sor.setMeanK (50);                      
      sor.setStddevMulThresh (1.0);
      sor.filter (*cloud_filtered);           //执行滤波处理，存储输出 
      
      sor_grid.setInputCloud (cloud_filtered);
      sor_grid.setLeafSize (0.005f, 0.005f, 0.005f);  //设置滤波时创建的体素体积为1cm的立方体
      sor_grid.filter (*cloud_grid);
      
     // Create the filtering object
     pcl::PassThrough<pcl::PointXYZRGBA> pass;
     pass.setInputCloud (cloud_grid);
     pass.setFilterFieldName ("z");
     pass.setFilterLimits (0.0, 0.55);

    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);
      viewer.showCloud (cloud_filtered);  //显示kinect获得的点云图像
      }
     }
void run ()  //运行kinect抓取的数据
 {
  pcl::Grabber* interface = new pcl::OpenNIGrabber();  //创建一个接口用于接受Kinect的数据
  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
         boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);  

  interface->registerCallback (f);//绑定回调函数

  interface->start ();     //接口工作

  while (!viewer.wasStopped())  //如果接受到结束指令
   {
    boost::this_thread::sleep  (boost::posix_time::seconds(1));  //串口线程停留一小段时间
   }
   interface->stop();  //然后停止接口的工作
 }
  pcl::visualization::CloudViewer viewer;
} ;


int main()
{
 SimpleOpenNIViewer v;
 v.run ();
 return 0;
}
