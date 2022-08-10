# upsamplingCloudPCL
Upsampling method for an input cloud using [MovingLeastSquares](https://pointclouds.org/documentation/classpcl_1_1_moving_least_squares.html) method of PCL

## Input file structure support

| Format      | Description |
| ----------- | ----------- |
| .pcd      | Point Cloud Data file format       |
| .ply   | Polygon file format        |
| .txt   | Text file format        |
| .xyz      | X Y Z Text file format       |

## Output file structure (.pcd)

* unsampled_cloud.pcd 

## Example
<p align="center">
   <img src="./example/example.png"><br>
</p>

<p align="center">
   <img src="https://user-images.githubusercontent.com/35694200/183946061-12df0269-fcc1-4fa3-a635-c2a86d5ba879.png"><br>
</p>

<p align="center">
   <img src="https://user-images.githubusercontent.com/35694200/183946790-f34f6129-6e21-4d4c-bd1c-1066c630943b.png"><br>
</p>


## Command line
```cpp
Usage: ./upsampling_cloud [options] 

Optional arguments:
-h --help         	shows help message and exits [default: false]
-v --version      	prints version information and exits [default: false]
--cloudfile       	input cloud file [required]
--search-radius   	epsilon value [default: 0.03]
--sampling-radius 	epsilon value [default: 0.005]
--step-size       	epsilon value [default: 0.005]
-o --output-dir   	output dir to save clusters [default: "-"]
-d --display      	display clusters in the pcl visualizer [default: false]
```

## Dependencies
This projects depends on the Point Cloud Library (it works with version `1.8...1.12.1`) and its dependencies.
|     Package      |   Version      |                             Description                                                                                                                  |
|     -----------       |   -----------      |                                  -----------                                                                                                                     |
|        VTK           |    9.0.0          |   Visualization toolkit                                                           |
|        PCL           |     1.12.1       |                    The Point Cloud Library (PCL)                                            |
|        Eigen        |     3.7.7         |  Eigen is a library of template headers for linear algebra                                 |
|        Flann        |     1.9.1         |      Fast Library for Approximate Nearest Neighbors                                  |
|       Boost         |    1.77.0        | Provides support for linear algebra, pseudorandom number generation, multithreading      |
|       OpenGL      |     21.2.6       | Programming interface for rendering 2D and 3D vector graphics.                    |


## Compilation
### Compile from source

1. Download source code

```bash
git clone https://github.com/danielTobon43/upsamplingCloudPCL
```

2. Create a "build" folder at the top level of the upsamplingCloudPCL

```bash
cd upsamplingCloudPCL/ && mkdir build
```

3. Compile with CMake

```bash
cd build/ && cmake ../ && make
```      
        	 
### Test
```bash
cd /build
./upsampling_cloud --cloudfile <path/to/cloud-file>
```

## Note

You can modify the parameters to obtain better results [here](https://github.com/danielTobon43/upsamplingCloudPCL/blob/master/src/main.cpp#:~:text=void%20upsampling(pcl,Ptr%26%20output_cloud)%20%7B)

```cpp
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
```
