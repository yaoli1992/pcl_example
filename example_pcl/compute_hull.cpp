

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

float default_alpha = 0.15f;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.vtk [optional_arguments]\n", argv[0]);
  print_info ("  where the optional arguments are:\n");
  print_info ("                     -alpha X = the alpha value for the ConcaveHull (Alpha Shapes) algorithm. If alpha is not specified, the tool will run the ConvexHull method (default: ");
  print_value ("%f", default_alpha); print_info (")\n");
}


void
compute (PointCloud<PointXYZ>::ConstPtr cloud_in,
         bool convex_concave_hull,
         float alpha,
         PolygonMesh &mesh_out)
{
  if (!convex_concave_hull)
  {
    print_info ("Computing the convex hull of a cloud with %lu points.\n", cloud_in->size ());
    ConvexHull<PointXYZ> convex_hull;
    convex_hull.setInputCloud (cloud_in);
    convex_hull.reconstruct (mesh_out);
  }
  else
  {
    print_info ("Computing the concave hull (alpha shapes) with alpha %f of a cloud with %lu points.\n", alpha, cloud_in->size ());
    ConcaveHull<PointXYZ> concave_hull;
    concave_hull.setInputCloud (cloud_in);
    concave_hull.setAlpha (alpha);
    concave_hull.reconstruct (mesh_out);
  }
}


/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Compute the convex or concave hull of a point cloud. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Command line parsing
  bool convex_concave_hull = false;
  float alpha = default_alpha;

  if (parse_argument (argc, argv, "-alpha", alpha) != -1)
    convex_concave_hull = true;

  vector<int> pcd_file_indices;
  pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (pcd_file_indices.size () != 1)
  {
    print_error ("Need one input PCD file to continue.\n");
    return (-1);
  }

  vector<int> vtk_file_indices;
  vtk_file_indices = parse_file_extension_argument (argc, argv, ".vtk");
  if (vtk_file_indices.size () != 1)
  {
    print_error ("Need one output VTK file to continue.\n");
    return (-1);
  }


  // Load in the point cloud
  PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ> ());
  if (loadPCDFile (argv[pcd_file_indices[0]], *cloud_in) != 0)
  {
    print_error ("Could not load input file %s\n", argv[pcd_file_indices[0]]);
    return (-1);
  }

  // Compute the hull
  PolygonMesh mesh_out;
  compute (cloud_in, convex_concave_hull, alpha, mesh_out);

  // Save the mesh
  io::saveVTKFile (argv[vtk_file_indices[0]], mesh_out);

  return (0);
}

