#include <pcl/console/print.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>
#include "argparse.hpp"
//#include "parser.hpp"
#include "visualizer.hpp"

void upsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud) {
    double search_radius = 0.3; // Upsampling method.
    double sampling_radius = 0.005;// Sampling step size. Bigger values will yield less (if any) new points.
    double step_size = 0.005;
    double gauss_param = (double)std::pow(search_radius, 2);
    int pol_order = 2;
    unsigned int num_threats = 1;

    // https://pointclouds.org/documentation/classpcl_1_1_moving_least_squares.html
    // check alternative https://pointclouds.org/documentation/classpcl_1_1_bilateral_upsampling.html
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dense_points(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;

    mls.setComputeNormals(true);
    mls.setInputCloud(input_cloud);
    mls.setSearchMethod(kd_tree); // Provide a pointer to the search object
    mls.setSearchRadius(search_radius); //Set the sphere radius that is to be used for determining the k-nearest neighbors used for fitting
    mls.setUpsamplingMethod(
        pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>::UpsamplingMethod::SAMPLE_LOCAL_PLANE);
    //	Set the upsampling method to be used
    
    mls.setUpsamplingRadius(sampling_radius); // SAMPLE_LOCAL_PLANE upsampling(샘플링할 local point plane 에서 원의 반지름)
    
    mls.setUpsamplingStepSize(step_size); //Set the step size for the local plane sampling
    mls.setPolynomialOrder(pol_order);    // Set the order of the polynomial to be fit
    mls.setSqrGaussParam(gauss_param);    // Set the parameter used for distance based weighting of neighbors (the square of the search radius works best in general)
    mls.setCacheMLSResults(true);         // Set whether the mls results should be stored for each point in the input cloud.
    mls.setNumberOfThreads(num_threats);  //Set the maximum number of threads to use
    // mls.setDilationVoxelSize();//Used only in the VOXEL_GRID_DILATION upsampling method
    // mls.setPointDensity(15); //15
    mls.process(*dense_points); // Base method for surface reconstruction for all points given in <setInputCloud (), setIndices ()>

    *output_cloud = *input_cloud;
    *output_cloud += *dense_points;

    if (output_cloud->points.size() == input_cloud->points.size()) {
        pcl::console::print_warn("\ninput cloud could not be upsampled, change input parameters!");
    }

    pcl::console::print_info("\nNew points: ");
    pcl::console::print_value("%d", dense_points->points.size());

    pcl::console::print_info("\nOutput cloud points: ");
    pcl::console::print_value("%d", output_cloud->points.size());
    pcl::console::print_info("\n");
}

int main(int argc, char** argv) {

    // -----------------Read input cloud file -----------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    // cloud parser object
   // CloudParserLibrary::ParserCloudFile cloud_parser;
    //cloud_parser.load_cloudfile("C:/Users/admin/source/repos/Project2/Project2/crop.pcd", input_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("C:/Users/admin/source/repos/Project2/Project2/ph_crop.pcd", *input_cloud);
    std::cout << "Loaded " << cloud->width * cloud->height << std::endl;
    // set cloud metadata
    input_cloud->width = (int)input_cloud->points.size();
    input_cloud->height = 1;
    input_cloud->is_dense = true;

    // -----------------Upsampling -----------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    upsampling(input_cloud, output_cloud);
    pcl::io::savePCDFile("C:/Users/admin/source/repos/Project2/Project2/pred/upsampled_crop2.pcd", *output_cloud);

    // -----------------Visualize upsampling -----------------
    

    return 0;
}
