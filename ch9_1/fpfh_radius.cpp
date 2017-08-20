#include <iostream>
#include <pcl/point_types.h>
#include <pcl/features/feature_evaluation/feature_evaluation_framework.h>   //基准测试的头文件

int main (int argc, char** argv)
{ 
  if (argc < 4) {   //说明输入点云文件，真值变换矩阵文件，测试参数文件
    std::cout << "Specify the input cloud, ground truth and parameter files:\n";
    std::cout << "   " << argv[0] << " input_cloud.pcd ground_truth.txt parameters.txt\n";
    exit(0);
  }
  pcl::FeatureEvaluationFramework<pcl::PointXYZRGB> test_features;  //申明测试对象
  test_features.setFeatureTest ("FPFHTest");                     //设置测试的特征算子种类
  test_features.setGroundTruth (argv[2]);                        //设置真值文件
  //如果设置target-cloud文件的参数为“” 那么source-cloud经过真值变换矩阵转换得到target-cloud
  test_features.setInputClouds (argv[1], "", argv[1]);             //设置源和目标点云
  test_features.setThreshold (0.1f,1.0f,0.1f);                      //设置阀值
  //独立变量不需要被分别设置，它的值将从文件中读取，在这里独立变量为搜索半径
  //std::string parameters = "searchradius=0.05";
  //test_features.setParameters (parameters);
  test_features.setDownsampling (true);         //测试时对点云进行下采样
  test_features.setLeafSize (0.01f);            //设置叶子的大小
  test_features.setVerbose (true);                 //设置打印出过程信息
  test_features.setLogFile ("fpfh-radius-variation.txt");   //设置保存结果到
  test_features.runMultipleParameters (argv[3]);   //设置用很多个不同的半径进行测试
  test_features.clearData ();
  return 0;
}	
