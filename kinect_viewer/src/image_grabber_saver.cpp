
#include <pcl/io/image_grabber.h>    //图像获取接口的头文件
#include <pcl/console/parse.h>    //命令解析头文件
#include <pcl/console/print.h>     //打印头文件
#include <pcl/visualization/boost.h>   //线程头文件
#include <pcl/visualization/cloud_viewer.h>   //点云可视化的头文件
#include <pcl/visualization/image_viewer.h>    //图片可视化的头文件
#include <pcl/io/pcd_io.h>                    //PCD文件操作的头文件

using pcl::console::print_error;    //申明打印错误
using pcl::console::print_info;     //基本信息
using pcl::console::print_value;

//boost::mutex mutex_;    
boost::shared_ptr<pcl::ImageGrabber<pcl::PointXYZRGBA> > grabber;    //申明一个获取接口
pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_;       //申明一个点云的数据结构
std::string out_folder;      //字符的输出文件名
int counter;   //计数


//提示与帮助
void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s <options>\n", argv[0]);
  print_info (" where options are:\n");
  print_info ("\t-rgb_dir   \t<directory_path>    \t= directory path to RGB images to be read from\n");
  print_info ("\t-depth_dir \t<directory_path>    \t= directory path to Depth images to be read from\n");
  print_info ("\t-out_dir   \t<directory_path>    \t= directory path to put the pcd files\n");
  //print_info ("\t-fps frequency           = frames per second\n");
  print_info ("\n");
}

struct EventHelper
{
   //对点云的一个存储的回调函数
  void 
  cloud_cb (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & cloud)
  {
    std::stringstream ss;
    ss << out_folder << "/" << grabber->getPrevDepthFileName() << ".pcd";
    pcl::io::savePCDFileASCII (ss.str(), *cloud);
  }
};

//主函数
int
main (int argc, char** argv)
{
  counter = 0;   //计数为0
  out_folder.clear();   //对给定的文件下清除

  if (argc > 1)  //就是说如果有输入的命令有2个以上，只要有-h出现就执行帮助函数，显示提示信息
  {
    for (int i = 1; i < argc; i++)
    {
      if (std::string (argv[i]) == "-h")
      {
        printHelp (argc, argv);
        return (-1);
      }
    }
  }

  float frames_per_second = 0; // 0 means only if triggered!
  pcl::console::parse (argc, argv, "-fps", frames_per_second);
  if (frames_per_second < 0)
    frames_per_second = 0.0;

  float focal_length = 525.0;
  pcl::console::parse (argc, argv, "-focal", focal_length);

  std::string depth_path = "";
  pcl::console::parse_argument (argc, argv, "-depth_dir", depth_path);

  std::string rgb_path = "";
  pcl::console::parse_argument (argc, argv, "-rgb_dir", rgb_path);

  pcl::console::parse_argument (argc, argv, "-out_dir", out_folder);

  if (out_folder == "" || !boost::filesystem::exists (out_folder))
  {
    PCL_INFO("No correct directory was given with the -out_dir flag. Setting to current dir\n");
    out_folder = "./";
  }
  else
    PCL_INFO("Using %s as output dir", out_folder.c_str());

  if (rgb_path != "" && depth_path != "" && boost::filesystem::exists (rgb_path) && boost::filesystem::exists (depth_path))
  {
    grabber.reset (new pcl::ImageGrabber<pcl::PointXYZRGBA> (depth_path, rgb_path, frames_per_second, false));
  }
  else
  {
    PCL_INFO("No directory was given with the -<rgb/depth>_dir flag.");
    printHelp (argc, argv);
    return (-1);
  }
  grabber->setDepthImageUnits (float (1E-3));
  //grabber->setFocalLength(focal_length); // FIXME

  EventHelper h;
  boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&) > f = boost::bind (&EventHelper::cloud_cb, &h, _1);
  boost::signals2::connection c1 = grabber->registerCallback (f);

  do
  {
    grabber->trigger();
  }
  while (!grabber->atLastFrame());
  grabber->trigger(); // Attempt to process the last frame
  grabber->stop ();
}
/* ]--- */
