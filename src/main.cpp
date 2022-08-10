/*********************************
           HEADERS
**********************************/
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/features/feature.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/bilateral_upsampling.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <fstream>
#include <iostream>
#include <string>

#include "cloudparse/parser.hpp"

void printUsage(const char* progName) { std::cout << "\nUse: " << progName << " <file>" << std::endl << "support: .pcd .ply .txt .xyz" << std::endl << "[q] to exit" << std::endl; }

int main(int argc, char** argv) {
  if (argc < 2 or argc > 2) {
    printUsage(argv[0]);
    return -1;
  }

  // -----------------Read input cloud file -----------------
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  // cloud parser object
  CloudParserLibrary::ParserCloudFile cloud_parser;
  cloud_parser.load_cloudfile(argv[1], input_cloud);

  // set cloud metadata
  input_cloud->width = (int)input_cloud->points.size();
  input_cloud->height = 1;
  input_cloud->is_dense = true;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr dense_points(new pcl::PointCloud<pcl::PointXYZRGB>());

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>());
  pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;

  double search_radius = 0.03;     // 0.03
  double sampling_radius = 0.005;  // 0.005
  double step_size = 0.005;        // 0.005
  double gauss_param = (double)std::pow(search_radius, 2);
  int pol_order = 2;
  unsigned int num_threats = 1;

  mls.setComputeNormals(true);
  mls.setInputCloud(input_cloud);
  mls.setSearchMethod(kd_tree);
  mls.setSearchRadius(search_radius);
  mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>::UpsamplingMethod::SAMPLE_LOCAL_PLANE);
  mls.setUpsamplingRadius(sampling_radius);
  mls.setUpsamplingStepSize(step_size);
  mls.setPolynomialOrder(pol_order);
  mls.setSqrGaussParam(gauss_param);  // (the square of the search radius works best in general)
  mls.setCacheMLSResults(true);       // Set whether the mls results should be stored for each point in the input cloud.
  mls.setNumberOfThreads(num_threats);
  // mls.setDilationVoxelSize();//Used only in the VOXEL_GRID_DILATION upsampling method
  // mls.setPointDensity(15); //15
  mls.process(*dense_points);

  *output_cloud = *input_cloud;
  *output_cloud += *dense_points;

  pcl::console::print_info("\nNew points: ");
  pcl::console::print_value("%d", dense_points->points.size());

  pcl::console::print_info("\nOutput cloud points: ");
  pcl::console::print_value("%d", output_cloud->points.size());
  pcl::console::print_info("\n");

  vtkObject::GlobalWarningDisplayOff();  // Disable vtk render warning
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("VISUALIZER"));

  int PORT1 = 0;
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, PORT1);
  viewer->setBackgroundColor(0, 0, 0, PORT1);
  viewer->addText("ORIGINAL", 10, 10, "PORT1", PORT1);

  int PORT2 = 0;
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, PORT2);
  viewer->setBackgroundColor(0, 0, 0, PORT2);
  viewer->addText("UPSAMPLING", 10, 10, "PORT2", PORT2);

  viewer->removeAllPointClouds(0);

  if (input_cloud->points[0].r <= 0 and input_cloud->points[0].g <= 0 and input_cloud->points[0].b <= 0) {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(input_cloud, 255, 255, 0);
    viewer->addPointCloud(input_cloud, color_handler, "Original", PORT1);
  } else {
    viewer->addPointCloud(input_cloud, "Original", PORT1);
  }

  if (output_cloud->points[0].r <= 0 and output_cloud->points[0].g <= 0 and output_cloud->points[0].b <= 0) {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(output_cloud, 255, 255, 0);
    viewer->addPointCloud(output_cloud, color_handler, "transform1 rvec", PORT2);
  } else {
    viewer->addPointCloud(output_cloud, "transform1 rvec", PORT2);
  }

  pcl::io::savePCDFile("upsampled_cloud.pcd", *output_cloud);

  pcl::PointXYZ p1, p2, p3;
  p1.getArray3fMap() << 1, 0, 0;
  p2.getArray3fMap() << 0, 1, 0;
  p3.getArray3fMap() << 0, 0.1, 1;

  viewer->addCoordinateSystem(1, "original_usc", PORT1);
  viewer->addText3D("x", p1, 0.2, 1, 0, 0, "x_", PORT1);
  viewer->addText3D("y", p2, 0.2, 0, 1, 0, "y_", PORT1);
  viewer->addText3D("z", p3, 0.2, 0, 0, 1, "z_", PORT1);

  viewer->addCoordinateSystem(1, "transform_ucs", PORT2);
  viewer->addText3D("x", p1, 0.2, 1, 0, 0, "x_", PORT2);
  viewer->addText3D("y", p2, 0.2, 0, 1, 0, "y_", PORT2);
  viewer->addText3D("z", p3, 0.2, 0, 0, 1, "z_", PORT2);

  viewer->setPosition(0, 0);
  viewer->initCameraParameters();
  viewer->resetCamera();

  std::cout << "\nPress [q] to exit" << std::endl;

  while (!viewer->wasStopped()) {
    viewer->spin();
  }

  return 0;
}
