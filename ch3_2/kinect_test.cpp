#include <iostream>
#include <string>
#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std; 

const string OUT_DIR = "/home/salm/myopencv/yl_pcl/ch3_2/build"; 

class SimpleOpenNIViewer 
{ 
public: 
    SimpleOpenNIViewer () : viewer ("PCL Viewer") 
    { 
                frames_saved = 0; 
                save_one = false; 
    } 

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) 
    { 
                if (!viewer.wasStopped()) { 
                        viewer.showCloud (cloud);    //可视化

                        if( save_one ) { 
                                save_one = false; 
                                std::stringstream out;   //使用stringstream对象简化类型转换 
                                out << frames_saved;    //把frames_saved输入给out
                                std::string name = OUT_DIR + "cloud" + out.str() + ".pcd"; 
                                pcl::io::savePCDFileASCII( name, *cloud ); 
                        } 
                } 
    } 

    void run () 
    { 
                pcl::Grabber* interface = new pcl::OpenNIGrabber();    //申明一个接口

                boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = 
                        boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1); 

                interface->registerCallback (f); //注册回调函数

                interface->start ();    //开始

                char c; 

                while (!viewer.wasStopped()) 
                { 
                        sleep (1); 

                        c = getchar();  //使用按键S来保存点云数据 
                        if( c == 's' ) { 
                                cout << "Saving frame " << frames_saved << ".\n"; 
                                frames_saved++; 
                                save_one = true; 
                        } 
                } 

                interface->stop (); 
        } 

        pcl::visualization::CloudViewer viewer; 

        private: 
                int frames_saved; 
                bool save_one; 

}; 

int main () 
{ 
    SimpleOpenNIViewer v; 
    v.run (); 
    return 0; 
}
