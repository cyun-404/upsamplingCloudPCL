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
    double search_radius = 0.3;
    double sampling_radius = 0.005;
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
    mls.setSearchMethod(kd_tree);
    mls.setSearchRadius(search_radius);
    mls.setUpsamplingMethod(
        pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>::UpsamplingMethod::SAMPLE_LOCAL_PLANE);
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
    pcl::io::loadPCDFile("C:/Users/admin/source/repos/Project2/Project2/ImageToStl.com_ph_crop.pcd", *input_cloud);
    std::cout << "Loaded " << cloud->width * cloud->height << std::endl;
    // set cloud metadata
    input_cloud->width = (int)input_cloud->points.size();
    input_cloud->height = 1;
    input_cloud->is_dense = true;

    // -----------------Upsampling -----------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    upsampling(input_cloud, output_cloud);
    pcl::io::savePCDFile("C:/Users/admin/source/repos/Project2/Project2/upsampled_ImageToStl.com_ph_crop.pcd", *output_cloud);

    // -----------------Visualize upsampling -----------------
    

    return 0;
}