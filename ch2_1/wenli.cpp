#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;

void ReadPNT(const string &file_path, 
	vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> &points2d, 
	vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> &points3d)
{
	ifstream fin(file_path);
	string line;
	getline(fin, line);
	while(!fin.eof())
	{
		float x, y;
		int manual, type3d;
		float px, py, pz;
		int ident, hasprev;
		float pcx, pcy;
		int support;
		fin >> x >> y >> manual >> type3d >> px >> py >> pz >> 
			ident >> hasprev >> pcx >> pcy >> support;
		if(type3d == 2)
		{
			points2d.push_back(Eigen::Vector2f(x, y));
			points3d.push_back(Eigen::Vector3f(px, py, pz));
		}
	}
}
void generateUV(const Eigen::Vector3f &point3d, 
	const float u0, const float v0, const float fx, const float fy, 
	const float Xs, const float Ys, const float Zs,
	const int width, const int height,
	const Eigen::Matrix3f R, Eigen::Vector2f &uv)
{
	float X = point3d[0], Y = point3d[1], Z = point3d[2];
	float a1 = R(0, 0), a2 = R(0, 1), a3 = R(0, 2);
	float b1 = R(1, 0), b2 = R(1, 1), b3 = R(1, 2);
	float c1 = R(2, 0), c2 = R(2, 1), c3 = R(2, 2);
	uv[0] = -fx * 
		(a1 * (X - Xs) + b1 * (Y - Ys) + c1 * (Z - Zs)) / 
		(a3 * (X - Xs) + b3 * (Y - Ys) + c3 * (Z - Zs)) + u0;
	uv[0] = uv[0] / width;
	uv[1] = -fy * 
		(a2 * (X - Xs) + b2 * (Y - Ys) + c2 * (Z - Zs)) / 
		(a3 * (X - Xs) + b3 * (Y - Ys) + c3 * (Z - Zs)) + v0;
	uv[1] = uv[1] / height;
}
int main(int argc, char **argv)
{
	/*string file_path = "C:\\Users\\whp\\Desktop\\circular_01.pnt";
	vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> all_points_2d;
	vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> all_points_3d;
	ReadPNT(file_path, all_points_2d, all_points_3d);
	string file_path2 = "C:\\Users\\whp\\Desktop\\temp.txt";
	ofstream fout1(file_path2);
	for(unsigned int i = 0; i < all_points_2d.size(); i++)
	{
		fout1 << all_points_2d[i][0] << " " << all_points_2d[i][1] << " "
			<< all_points_3d[i][0] << " " << all_points_3d[i][1] << " " << all_points_3d[i][2] << endl;
	}
	string file_path_3d = "C:\\Users\\whp\\Desktop\\temp.obj";
	ofstream fout2(file_path_3d);
	for(unsigned int i = 0; i < all_points_2d.size(); i++)
	{
		fout2 << "v "<< all_points_3d[i][0] << " " << all_points_3d[i][1] << " " << all_points_3d[i][2] << endl;
	}*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	string file_path_ply = "C:\\Users\\whp\\Desktop\\temp.ply";
	if(pcl::io::loadPLYFile(file_path_ply, *cloud))
	{
		return -1;
	}
	float Cx = 6.7922715802f, Cy = -0.5212861853f, Cz = 1.1300269535f;
	float Ax = -0.0646240933f, Ay = 0.0285786216f, Az = 0.9975003704f;
	float Hx = 1168.1485817776f, Hy = -22.2671618865f, Hz = 76.3176736651f;
	float Vx = 26.0187265534f, Vy = 1248.1713103748f, Vz = -34.0747531712f;
	float sx = 0.0106666667f, sy = 0.0100000000f;
	int Width = 1024, Height = 768;
	float f = 12.4890739568f;
	float H0x = 0.9976921894f, H0y = -0.0190179347f, H0z = 0.0651813888f;
	float V0x = 0.0208331912f, V0y = 0.9994106166f, V0z = -0.0272836507f;

	pcl::texture_mapping::Camera camera_dianyun;
	camera_dianyun.focal_length = f / sx;
	camera_dianyun.height = Height;
	camera_dianyun.width = Width;
	Eigen::Affine3f pose_rotation;
	pose_rotation.matrix() << 
		H0x, H0y, H0z, 0,
		V0x, V0y, V0z, 0,
		Ax, Ay, Az, 0,
		0, 0, 0, 1;
	Eigen::Affine3f pose_translation;
	pose_translation = Eigen::Translation<float, 3>(-Cx, -Cy, -Cz);
	camera_dianyun.pose = pose_translation * pose_rotation;
	cout << camera_dianyun.pose.matrix() << endl;
	string file_path_texture = "C:\\Users\\whp\\Desktop\\circular_01.jpg";
	pcl::TextureMapping<pcl::PointXYZ> dianyun_texturemapping;
	pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud, *camera_cloud, camera_dianyun.pose.matrix());
	vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > all_points = camera_cloud->points;
	ofstream fout("C:\\Users\\whp\\Desktop\\test.txt");
	for(unsigned int i = 0; i < all_points.size(); i++)
	{
		Eigen::Vector2f uv;
		dianyun_texturemapping.getPointUVCoordinates(all_points[i], camera_dianyun, uv);
// 		generateUV(all_points_3d[i], Width / 2, Height / 2, f, f, Cx, Cy, Cz, Width, Height, 
// 			camera_dianyun.pose.linear(), uv);
		fout << "vt " << uv[0] << " " << (1.0f - uv[1]) << endl;
		//fout << "v " << all_points[i].x << " " << all_points[i].y << " " << all_points[i].z << endl;
	}
	return 0;
}