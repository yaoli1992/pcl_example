
#define MEASURE_FUNCTION_TIME
#include <pcl/common/time.h>          //fps calculations
#include <pcl/io/openni_grabber.h>    //openni接口
#include <pcl/visualization/pcl_visualizer.h>  //PCL可视化接口
#include <pcl/visualization/boost.h>            //线程的头文件
#include <pcl/visualization/image_viewer.h>    //图像显示的头文件
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)
#else
#define FPS_CALC(_WHAT_) \
do \
{ \
}while(false)
#endif

void
printHelp (int, char **argv)
{
  using pcl::console::print_error;
  using pcl::console::print_info;

  print_error ("Syntax is: %s [((<device_id> | <path-to-oni-file>) [-depthmode <mode>] [-imagemode <mode>] [-xyz] | -l [<device_id>]| -h | --help)]\n", argv [0]);   //命令的用法

  print_info ("%s -h | --help : shows this help\n", argv [0]);
  print_info ("%s -xyz : use only XYZ values and ignore RGB components (this flag is required for use with ASUS Xtion Pro) \n", argv [0]);  //只获取XYZ的值而忽略RGB的值

  print_info ("%s -l : list all available devices\n", argv [0]);  //列出可用的设备
  print_info ("%s -l <device-id> :list all available modes for specified device\n", argv [0]); 
  print_info ("\t\t<device_id> may be \"#1\", \"#2\", ... for the first, second etc device in the list\n");
#ifndef _WIN32
  print_info ("\t\t                   bus@address for the device connected to a specific usb-bus / address combination\n");
  print_info ("\t\t                   <serial-number>\n");
#endif
  print_info ("\n\nexamples:\n");
  print_info ("%s \"#1\"\n", argv [0]);
  print_info ("\t\t uses the first device.\n");
  print_info ("%s  \"./temp/test.oni\"\n", argv [0]);
  print_info ("\t\t uses the oni-player device to play back oni file given by path.\n");
  print_info ("%s -l\n", argv [0]);
  print_info ("\t\t list all available devices.\n");
  print_info ("%s -l \"#2\"\n", argv [0]);
  print_info ("\t\t list all available modes for the second device.\n");
  #ifndef _WIN32
  print_info ("%s A00361800903049A\n", argv [0]);
  print_info ("\t\t uses the device with the serial number \'A00361800903049A\'.\n");
  print_info ("%s 1@16\n", argv [0]);
  print_info ("\t\t uses the device on address 16 at USB bus 1.\n");
  #endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointType>   //声明一个模板
class OpenNIViewer               //一个类
{
  public:
    typedef pcl::PointCloud<PointType> Cloud;      //申明点云的类型
    typedef typename Cloud::ConstPtr CloudConstPtr;

    OpenNIViewer (pcl::Grabber& grabber) //申明一个接口
      : cloud_viewer_ (new pcl::visualization::PCLVisualizer ("PCL OpenNI cloud"))  //显示窗口的名称
      , image_viewer_ ()   //图像显示
      , grabber_ (grabber)  //
      , rgb_data_ (0)   //rgb图像的数据
      , rgb_data_size_ (0)  //大小
    {
    }

    void   //点云的回调函数
    cloud_callback (const CloudConstPtr& cloud)
    {
      FPS_CALC ("cloud callback");   //对每次的回调函数的计时
      boost::mutex::scoped_lock lock (cloud_mutex_);   //锁住线程
      cloud_ = cloud;     //赋值点云数据 
    }

    void    //图像数据的回调函数
    image_callback (const boost::shared_ptr<openni_wrapper::Image>& image)
    {
      FPS_CALC ("image callback");   //每次回调图像时计时
      boost::mutex::scoped_lock lock (image_mutex_);  //锁住线程，放置其他线程影响数据
      image_ = image; 
      
      if (image->getEncoding () != openni_wrapper::Image::RGB)   //如果接口得到的图像的编码的形式是RGB
      {
        if (rgb_data_size_ < image->getWidth () * image->getHeight ())  //此时RGB数据的大小小于图像高与宽相乘的结果
        {
          if (rgb_data_)  //如果数据为空值
            delete [] rgb_data_;
          rgb_data_size_ = image->getWidth () * image->getHeight ();   //对图像大小赋值
          rgb_data_ = new unsigned char [rgb_data_size_ * 3];   //数据乘以3
        }
        image_->fillRGB (image_->getWidth (), image_->getHeight (), rgb_data_);
      }
    }
    
    void    //按键的回调函数
    keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
    {
      if (event.getKeyCode ())   //获得按键的按键值
        cout << "the key \'" << event.getKeyCode() << "\' (" << event.getKeyCode() << ") was";
      else
        cout << "the special key \'" << event.getKeySym() << "\' was";
      if (event.keyDown())
        cout << " pressed" << endl;
      else
        cout << " released" << endl;
    }
    
    void   //鼠标的回调函数
    mouse_callback (const pcl::visualization::MouseEvent& mouse_event, void*)
    {      //获得鼠标的事件类型
      if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
      { //输出鼠标左键在图像中的坐标XY的值
        cout << "left button pressed @ " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
      }
    }
//主函数

    void
    run ()
    {  
      cloud_viewer_->registerMouseCallback (&OpenNIViewer::mouse_callback, *this);   //注册鼠标的回调函数
      cloud_viewer_->registerKeyboardCallback(&OpenNIViewer::keyboard_callback, *this); //注册按键的回调函数
      boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&OpenNIViewer::cloud_callback, this, _1);  //绑定点云的回调函数
      boost::signals2::connection cloud_connection = grabber_.registerCallback (cloud_cb); //注册接口获取的点云的回调函数
      
      boost::signals2::connection image_connection;  //线程的连接

      if (grabber_.providesCallback<void (const boost::shared_ptr<openni_wrapper::Image>&)>())  //回调函数的获取
      {
        image_viewer_.reset (new pcl::visualization::ImageViewer ("PCL OpenNI image"));  //首先初始化并显示显示窗口的名称
        image_viewer_->registerMouseCallback (&OpenNIViewer::mouse_callback, *this);     //鼠标的回调函数
        image_viewer_->registerKeyboardCallback(&OpenNIViewer::keyboard_callback, *this); //按键的回调函数 

        boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&) > image_cb = boost::bind (&OpenNIViewer::image_callback, this, _1);
        image_connection = grabber_.registerCallback (image_cb);//注册图像的回调函数
      }
      
      bool image_init = false, cloud_init = false;
      
      grabber_.start ();  //开始获取回调

      while (!cloud_viewer_->wasStopped () && (image_viewer_ && !image_viewer_->wasStopped ()))  //等待结束按钮
      {
        boost::shared_ptr<openni_wrapper::Image> image;
        CloudConstPtr cloud;

        cloud_viewer_->spinOnce ();     //回调

        // See if we can get a cloud    //如果我们得到一个点云
        if (cloud_mutex_.try_lock ()) //线程锁定
        {
          cloud_.swap (cloud);   //
          cloud_mutex_.unlock ();  //解锁线程
        }

        if (cloud)
        {
          FPS_CALC ("drawing cloud");
          
          if (!cloud_init)   //
          {
            cloud_viewer_->setPosition (0, 0);
            cloud_viewer_->setSize (cloud->width, cloud->height);
            cloud_init = !cloud_init;
          }

          if (!cloud_viewer_->updatePointCloud (cloud, "OpenNICloud"))
          {
            cloud_viewer_->addPointCloud (cloud, "OpenNICloud");   //可视化点云
            cloud_viewer_->resetCameraViewpoint ("OpenNICloud");   //初始化相机的视点
          }          
        }

        // See if we can get an image   //可视化RGB图像
        if (image_mutex_.try_lock ())
        {
          image_.swap (image);
          image_mutex_.unlock ();
        }

        if (image)
        {
          if (!image_init && cloud && cloud->width != 0)
          {
            image_viewer_->setPosition (cloud->width, 0);
            image_viewer_->setSize (cloud->width, cloud->height);
            image_init = !image_init;
          }

          if (image->getEncoding() == openni_wrapper::Image::RGB)
            image_viewer_->addRGBImage (image->getMetaData ().Data (), image->getWidth (), image->getHeight ());
          else
            image_viewer_->addRGBImage (rgb_data_, image->getWidth (), image->getHeight ());
          image_viewer_->spinOnce ();
        }
        
      }

      grabber_.stop ();
      
      cloud_connection.disconnect ();
      image_connection.disconnect ();
      if (rgb_data_)
        delete[] rgb_data_;
    }
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;   //申明点云的可视化
    boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer_;    //图像的可视化
    
    pcl::Grabber& grabber_;
    boost::mutex cloud_mutex_;    //点云的线程
    boost::mutex image_mutex_;    //图像的线程
    
    CloudConstPtr cloud_;
    boost::shared_ptr<openni_wrapper::Image> image_;
    unsigned char* rgb_data_;
    unsigned rgb_data_size_;
};

// Create the PCLVisualizer object
boost::shared_ptr<pcl::visualization::PCLVisualizer> cld;
boost::shared_ptr<pcl::visualization::ImageViewer> img;

/* ---[ */
int
main (int argc, char** argv)
{
  std::string device_id("");
  pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
  pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
  bool xyz = false;
  
  if (argc >= 2)
  {
    device_id = argv[1];
    if (device_id == "--help" || device_id == "-h")
    {
      printHelp(argc, argv);
      return 0;
    }
    else if (device_id == "-l")
    {
      if (argc >= 3)
      {
        pcl::OpenNIGrabber grabber(argv[2]);
        boost::shared_ptr<openni_wrapper::OpenNIDevice> device = grabber.getDevice();
        cout << "Supported depth modes for device: " << device->getVendorName() << " , " << device->getProductName() << endl;
        std::vector<std::pair<int, XnMapOutputMode > > modes = grabber.getAvailableDepthModes();
        for (std::vector<std::pair<int, XnMapOutputMode > >::const_iterator it = modes.begin(); it != modes.end(); ++it)
        {
          cout << it->first << " = " << it->second.nXRes << " x " << it->second.nYRes << " @ " << it->second.nFPS << endl;
        }

        if (device->hasImageStream ())
        {
          cout << endl << "Supported image modes for device: " << device->getVendorName() << " , " << device->getProductName() << endl;
          modes = grabber.getAvailableImageModes();
          for (std::vector<std::pair<int, XnMapOutputMode > >::const_iterator it = modes.begin(); it != modes.end(); ++it)
          {
            cout << it->first << " = " << it->second.nXRes << " x " << it->second.nYRes << " @ " << it->second.nFPS << endl;
          }
        }
      }
      else
      {
        openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
        if (driver.getNumberDevices() > 0)
        {
          for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices(); ++deviceIdx)
          {
            cout << "Device: " << deviceIdx + 1 << ", vendor: " << driver.getVendorName(deviceIdx) << ", product: " << driver.getProductName(deviceIdx)
              << ", connected: " << driver.getBus(deviceIdx) << " @ " << driver.getAddress(deviceIdx) << ", serial number: \'" << driver.getSerialNumber(deviceIdx) << "\'" << endl;
          }

        }
        else
          cout << "No devices connected." << endl;
        
        cout <<"Virtual Devices available: ONI player" << endl;
      }
      return 0;
    }
  }
  else
  {
    openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
    if (driver.getNumberDevices() > 0)
      cout << "Device Id not set, using first device." << endl;
  }
  
  unsigned mode;
  if (pcl::console::parse(argc, argv, "-depthmode", mode) != -1)
    depth_mode = pcl::OpenNIGrabber::Mode (mode);

  if (pcl::console::parse(argc, argv, "-imagemode", mode) != -1)
    image_mode = pcl::OpenNIGrabber::Mode (mode);
  
  if (pcl::console::find_argument (argc, argv, "-xyz") != -1)
    xyz = true;
  
  pcl::OpenNIGrabber grabber (device_id, depth_mode, image_mode);
  
  if (xyz || !grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgb> ())
  {
    OpenNIViewer<pcl::PointXYZ> openni_viewer (grabber);
    openni_viewer.run ();
  }
  else
  {
    OpenNIViewer<pcl::PointXYZRGBA> openni_viewer (grabber);
    openni_viewer.run ();
  }
  
  return (0);
}
/* ]--- */
