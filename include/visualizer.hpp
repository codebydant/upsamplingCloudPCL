#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <sstream>
#include <string>

void append_points_size_to_display(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                                   pcl::visualization::PCLVisualizer::Ptr &viewer, int &PORT, std::string &name) {
  // add points label to visualizer
  std::string str = "Points: ";
  std::stringstream ss;
  ss << cloud->points.size();
  str += ss.str();
  int xpos = 10;
  int ypos = 25;
  int fontSize = 13;
  double r = 1.0;
  double g = 1.0;
  double b = 1.0;
  viewer->addText(str, xpos, ypos, fontSize, r, g, b, name, PORT);
}

void display_upsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud) {
  vtkObject::GlobalWarningDisplayOff();  // Disable vtk render warning
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCL VISUALIZER"));

  int PORT1 = 0;
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, PORT1);
  viewer->setBackgroundColor(0, 0, 0, PORT1);
  viewer->addText("ORIGINAL", 10, 10, "PORT1", PORT1);

  int PORT2 = 0;
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, PORT2);
  viewer->setBackgroundColor(0, 0, 0, PORT2);
  viewer->addText("UPSAMPLING", 10, 10, "PORT2", PORT2);

  std::string name1 = "points_cloud_1";
  std::string name2 = "points_cloud_2";

  append_points_size_to_display(input_cloud, viewer, PORT1, name1);
  append_points_size_to_display(output_cloud, viewer, PORT2, name2);

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
}