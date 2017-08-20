

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int    default_k = 0;
double default_radius = 0.0;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options> [optional_arguments]\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -radius X = use a radius of Xm around each point to determine the neighborhood (default: "); 
  print_value ("%f", default_radius); print_info (")\n");
  print_info ("                     -k X      = use a fixed number of X-nearest neighbors around each point (default: "); 
  print_value ("%f", default_k); print_info (")\n");
  print_info (" For organized datasets, an IntegralImageNormalEstimation approach will be used, with the RADIUS given value as SMOOTHING SIZE.\n");
  print_info ("\nOptional arguments are:\n");
  print_info ("                     -input_dir X  = batch process all PCD files found in input_dir\n");
  print_info ("                     -output_dir X = save the processed files from input_dir in this directory\n");
}

bool
loadCloud (const string &filename, pcl::PCLPointCloud2 &cloud,
           Eigen::Vector4f &translation, Eigen::Quaternionf &orientation)
{
  if (loadPCDFile (filename, cloud, translation, orientation) < 0)
    return (false);

  return (true);
}

void
compute (const pcl::PCLPointCloud2::ConstPtr &input, pcl::PCLPointCloud2 &output,
         int k, double radius)
{
  // 把点云的初始数据转化位 PointCloud<T>
  PointCloud<PointXYZ>::Ptr xyz (new PointCloud<PointXYZ>);  
  fromPCLPointCloud2 (*input, *xyz);

  TicToc tt;//时间
  tt.tic ();
 
  PointCloud<Normal> normals;   //法线申明

  // 对于点云是否是有序点云和无序点云是不同的计算法线的方法
  if (xyz->isOrganized ())
  {
    IntegralImageNormalEstimation<PointXYZ, Normal> ne;  //这是一种对于有序 点云的法线求解方法：积分图法
    ne.setInputCloud (xyz);   
    
    //enum pcl::IntegralImageNormalEstimation::NormalEstimationMethod
    //NormalEstimationMethod方法有：
   // COVARIANCE_MATRIX - creates 9 integral images to compute the normal for a specific point from the covariance matrix of its local neighborhood.
   // AVERAGE_3D_GRADIENT - creates 6 integral images to compute smoothed versions of horizontal and vertical 3D gradients and computes the normals using the cross-product between these two gradients.
    //AVERAGE_DEPTH_CHANGE - creates only a single integral image and computes the normals from the average depth changes.
                          
    ne.setNormalEstimationMethod (IntegralImageNormalEstimation<PointXYZ, Normal>::COVARIANCE_MATRIX);
    ne.setNormalSmoothingSize (float (radius));  //Set the normal smoothing size
    ne.setDepthDependentSmoothing (true);//Set whether to use depth depending smoothing or not
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);   // 设置计算法线的区域
    ne.compute (normals);
  }
  else  //否则是无序点云的话 就需要使用kdtree临域查找法，通过周边的点计算法线
  {
    NormalEstimation<PointXYZ, Normal> ne;
    ne.setInputCloud (xyz);
    ne.setSearchMethod (search::KdTree<PointXYZ>::Ptr (new search::KdTree<PointXYZ>));
    ne.setKSearch (k);
    ne.setRadiusSearch (radius);
    ne.compute (normals);
  }

  print_highlight ("Computed normals in "); print_value ("%g", tt.toc ()); print_info (" ms for "); print_value ("%d", normals.width * normals.height); print_info (" points.\n");

  // Convert data back
  pcl::PCLPointCloud2 output_normals;    //法线点云申明
  toPCLPointCloud2 (normals, output_normals);  //格式的转化
  concatenateFields (*input, output_normals, output);  //将法线与原始点云拼接成一副点云
}

void
saveCloud (const string &filename, const pcl::PCLPointCloud2 &output,
           const Eigen::Vector4f &translation, const Eigen::Quaternionf &orientation)
{
  PCDWriter w;
  w.writeBinaryCompressed (filename, output, translation, orientation);
}

int
batchProcess (const vector<string> &pcd_files, string &output_dir, int k, double radius)
{
#if _OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < int (pcd_files.size ()); ++i)
  {
    // Load the first file
    Eigen::Vector4f translation;
    Eigen::Quaternionf rotation;
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    if (!loadCloud (pcd_files[i], *cloud, translation, rotation)) 
      continue;

    // Perform the feature estimation
    pcl::PCLPointCloud2 output;
    compute (cloud, output, k, radius);

    // Prepare output file name
    string filename = pcd_files[i];
    boost::trim (filename);
    vector<string> st;
    boost::split (st, filename, boost::is_any_of ("/\\"), boost::token_compress_on);
    
    // Save into the second file
    stringstream ss;
    ss << output_dir << "/" << st.at (st.size () - 1);
    saveCloud (ss.str (), output, translation, rotation);
  }
  return (0);
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Estimate surface normals using NormalEstimation. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  bool batch_mode = false;

  // Command line parsing
  int k = default_k;
  double radius = default_radius;
  parse_argument (argc, argv, "-k", k);
  parse_argument (argc, argv, "-radius", radius);
  string input_dir, output_dir;
  if (parse_argument (argc, argv, "-input_dir", input_dir) != -1)
  {
    PCL_INFO ("Input directory given as %s. Batch process mode on.\n", input_dir.c_str ());
    if (parse_argument (argc, argv, "-output_dir", output_dir) == -1)
    {
      PCL_ERROR ("Need an output directory! Please use -output_dir to continue.\n");
      return (-1);
    }

    // Both input dir and output dir given, switch into batch processing mode
    batch_mode = true;
  }

  if (!batch_mode)
  {
    // Parse the command line arguments for .pcd files
    vector<int> p_file_indices;
    p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
    if (p_file_indices.size () != 2)
    {
      print_error ("Need one input PCD file and one output PCD file to continue.\n");
      return (-1);
    }

    print_info ("Estimating normals with a k/radius/smoothing size of: "); 
    print_value ("%d / %f / %f\n", k, radius, radius); 

    // Load the first file
    Eigen::Vector4f translation;
    Eigen::Quaternionf rotation;
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    if (!loadCloud (argv[p_file_indices[0]], *cloud, translation, rotation)) 
      return (-1);

    // Perform the feature estimation
    pcl::PCLPointCloud2 output;
    compute (cloud, output, k, radius);

    // Save into the second file
    saveCloud (argv[p_file_indices[1]], output, translation, rotation);
  }
  else
  {
    if (input_dir != "" && boost::filesystem::exists (input_dir))
    {
      vector<string> pcd_files;
      boost::filesystem::directory_iterator end_itr;
      for (boost::filesystem::directory_iterator itr (input_dir); itr != end_itr; ++itr)
      {
        // Only add PCD files
        if (!is_directory (itr->status ()) && boost::algorithm::to_upper_copy (boost::filesystem::extension (itr->path ())) == ".PCD" )
        {
          pcd_files.push_back (itr->path ().string ());
          PCL_INFO ("[Batch processing mode] Added %s for processing.\n", itr->path ().string ().c_str ());
        }
      }
      batchProcess (pcd_files, output_dir, k, radius);
    }
    else
    {
      PCL_ERROR ("Batch processing mode enabled, but invalid input directory (%s) given!\n", input_dir.c_str ());
      return (-1);
    }
  }
}

