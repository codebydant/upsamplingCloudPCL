/*********************************
           HEADERS
**********************************/
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/surface/bilateral_upsampling.h>
#include <pcl/surface/mls.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/search/search.h>

#include <pcl/features/feature.h>
#include <pcl/point_types.h>

#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <iostream>
#include <fstream>
#include <string>

void printUsage (const char* progName){
  std::cout << "\nUse: " << progName << " <file>"  << std::endl <<
               "support: .pcd .ply .txt .xyz" << std::endl <<
               "[q] to exit" << std::endl;
}


int main(int argc, char **argv){

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

	pcl::PolygonMesh cl;
    std::vector<int> filenames;
    bool file_is_pcd = false;
    bool file_is_ply = false;
    bool file_is_txt = false;
    bool file_is_xyz = false;


    if(argc < 2 or argc > 2){
        printUsage (argv[0]);
        return -1;
    }

    pcl::console::TicToc tt;
    pcl::console::print_highlight ("Loading ");

    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");
    if(filenames.size()<=0){
        filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
        if(filenames.size()<=0){
            filenames = pcl::console::parse_file_extension_argument(argc, argv, ".txt");
            if(filenames.size()<=0){
                filenames = pcl::console::parse_file_extension_argument(argc, argv, ".xyz");
                if(filenames.size()<=0){
                    printUsage (argv[0]);
                    return -1;
                }else if(filenames.size() == 1){
                    file_is_xyz = true;
                }
            }else if(filenames.size() == 1){
                file_is_txt = true;
            }
        }else if(filenames.size() == 1){
            file_is_pcd = true;
        }
    }else if(filenames.size() == 1){
        file_is_ply = true;
    }else{
        printUsage (argv[0]);
        return -1;
    }

    if(file_is_pcd){
        if(pcl::io::loadPCDFile(argv[filenames[0]], *input_cloud) < 0){
            std::cout << "Error loading point cloud " << argv[filenames[0]]  << "\n";
            printUsage (argv[0]);
            return -1;
        }
      pcl::console::print_info("\nFound pcd file.\n");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", input_cloud->size ());
      pcl::console::print_info (" points]\n");
    }else if(file_is_ply){
      pcl::io::loadPLYFile(argv[filenames[0]],*input_cloud);
      if(input_cloud->points.size()<=0 or input_cloud->points.at(0).x <=0 and input_cloud->points.at(0).y <=0 and input_cloud->points.at(0).z <=0){
          pcl::console::print_warn("\nloadPLYFile could not read the cloud, attempting to loadPolygonFile...\n");
          pcl::io::loadPolygonFile(argv[filenames[0]], cl);
          pcl::fromPCLPointCloud2(cl.cloud, *input_cloud);
          if(input_cloud->points.size()<=0 or input_cloud->points.at(0).x <=0 and input_cloud->points.at(0).y <=0 and input_cloud->points.at(0).z <=0){
              pcl::console::print_warn("\nloadPolygonFile could not read the cloud, attempting to PLYReader...\n");
              pcl::PLYReader plyRead;
              plyRead.read(argv[filenames[0]],*input_cloud);
              if(input_cloud->points.size()<=0 or input_cloud->points.at(0).x <=0 and input_cloud->points.at(0).y <=0 and input_cloud->points.at(0).z <=0){
                  pcl::console::print_error("\nError. ply file is not compatible.\n");
                  return -1;
              }
          }
       }

      pcl::console::print_info("\nFound ply file.\n");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", input_cloud->points.size ());
      pcl::console::print_info (" points]\n");
      
    }else if(file_is_txt){
      std::ifstream file(argv[filenames[0]]);
      if(!file.is_open()){
          std::cout << "Error: Could not find "<< argv[filenames[0]] << std::endl;
          return -1;
      }
      
      std::cout << "file opened." << std::endl;
      double x_,y_,z_;
      unsigned int r, g, b; 

      while(file >> x_ >> y_ >> z_ >> r >> g >> b){
          pcl::PointXYZRGB pt;
          pt.x = x_;
          pt.y = y_;
          pt.z= z_;            
          
          uint8_t r_, g_, b_; 
          r_ = uint8_t(r); 
          g_ = uint8_t(g); 
          b_ = uint8_t(b); 

          uint32_t rgb_ = ((uint32_t)r_ << 16 | (uint32_t)g_ << 8 | (uint32_t)b_); 
          pt.rgb = *reinterpret_cast<float*>(&rgb_);               
              
          input_cloud->points.push_back(pt);
          //std::cout << "pointXYZRGB:" <<  pt << std::endl;
      }      
     
      pcl::console::print_info("\nFound txt file.\n");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", input_cloud->points.size ());
      pcl::console::print_info (" points]\n");
      
    }else if(file_is_xyz){  
      std::ifstream file(argv[filenames[0]]);
      if(!file.is_open()){
          std::cout << "Error: Could not find "<< argv[filenames[0]] << std::endl;
          return -1;
      }
      
      std::cout << "file opened." << std::endl;
      double x_,y_,z_;

      while(file >> x_ >> y_ >> z_){
          
          pcl::PointXYZRGB pt;
          pt.x = x_;
          pt.y = y_;
          pt.z= z_;            
          
          input_cloud->points.push_back(pt);
          //std::cout << "pointXYZRGB:" <<  pt << std::endl;
      }      
     
      pcl::console::print_info("\nFound xyz file.\n");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", input_cloud->points.size ());
      pcl::console::print_info (" points]\n");
    }

    input_cloud->width = (int) input_cloud->points.size ();
    input_cloud->height = 1;
    input_cloud->is_dense = true;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dense_points (new pcl::PointCloud<pcl::PointXYZRGB> ());

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointNormal>());
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;

    double search_radius = 0.03; //0.03
    double sampling_radius = 0.005; //0.005
    double step_size = 0.005; //0.005
    double gauss_param = (double)std::pow(search_radius,2);
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
    mls.setSqrGaussParam(gauss_param);// (the square of the search radius works best in general)
    mls.setCacheMLSResults(true);//Set whether the mls results should be stored for each point in the input cloud.
    mls.setNumberOfThreads(num_threats);
    //mls.setDilationVoxelSize();//Used only in the VOXEL_GRID_DILATION upsampling method 
    //mls.setPointDensity(15); //15
    mls.process(*dense_points);

    *output_cloud = *input_cloud;
    *output_cloud += *dense_points;

    pcl::console::print_info("\nNew points: ");
    pcl::console::print_value("%d", dense_points->points.size());

    pcl::console::print_info("\nOutput cloud points: ");
    pcl::console::print_value("%d", output_cloud->points.size());
    pcl::console::print_info("\n");
	
    vtkObject::GlobalWarningDisplayOff(); // Disable vtk render warning   
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("VISUALIZER"));

	int PORT1 = 0;
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, PORT1);
	viewer->setBackgroundColor (0, 0, 0, PORT1);
	viewer->addText("ORIGINAL", 10, 10, "PORT1", PORT1);

	int PORT2 = 0;
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, PORT2);
	viewer->setBackgroundColor (0, 0, 0, PORT2);
	viewer->addText("UPSAMPLING", 10, 10, "PORT2", PORT2);
	  
	viewer->removeAllPointClouds(0);

	if(input_cloud->points[0].r <= 0 and input_cloud->points[0].g <= 0 and input_cloud->points[0].b<= 0 ){
	    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(input_cloud,255,255,0);      
	    viewer->addPointCloud(input_cloud,color_handler,"Original",PORT1);
	}else{
	    viewer->addPointCloud(input_cloud,"Original",PORT1);
	}

	if(output_cloud->points[0].r <= 0 and output_cloud->points[0].g <= 0 and output_cloud->points[0].b<= 0 ){
	    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(output_cloud,255,255,0);
	    viewer->addPointCloud(output_cloud,color_handler,"transform1 rvec",PORT2);
	}else{
	    viewer->addPointCloud(output_cloud,"transform1 rvec",PORT2);
	}

	pcl::io::savePCDFile ("upsampled_cloud.pcd", *output_cloud);
	  
	pcl::PointXYZ p1, p2, p3;
	p1.getArray3fMap() << 1, 0, 0;
	p2.getArray3fMap() << 0, 1, 0;
	p3.getArray3fMap() << 0,0.1,1;

	viewer->addCoordinateSystem(1,"original_usc",PORT1);
	viewer->addText3D("x", p1, 0.2, 1, 0, 0, "x_",PORT1);
	viewer->addText3D("y", p2, 0.2, 0, 1, 0, "y_",PORT1);
	viewer->addText3D ("z", p3,0.2, 0, 0, 1, "z_",PORT1);

	viewer->addCoordinateSystem(1,"transform_ucs",PORT2);
	viewer->addText3D("x", p1, 0.2, 1, 0, 0, "x_",PORT2);
	viewer->addText3D("y", p2, 0.2, 0, 1, 0, "y_",PORT2);
	viewer->addText3D ("z", p3,0.2, 0, 0, 1, "z_",PORT2);
	 
	viewer->setPosition(0,0);
	viewer->initCameraParameters();
	viewer->resetCamera();

	std::cout << "\nPress [q] to exit" << std::endl;

	while(!viewer->wasStopped ()) {
	    viewer->spin();
	}

    return 0;

}
