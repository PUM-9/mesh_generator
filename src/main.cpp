#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/poisson.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include "mesh_generator/MeshPointCloud.h"


typedef mesh_generator::MeshPointCloud::Request MeshPointCloudRequest;
typedef mesh_generator::MeshPointCloud::Response MeshPointCloudResponse;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;


NormalCloud::Ptr estimate_normals(PointCloud::Ptr point_cloud)
{
    // Declare PCL objects needed to perform normal estimation
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation;
    NormalCloud::Ptr normals(new NormalCloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // Set input parameters for normal estimation
    search_tree->setInputCloud(point_cloud);
    normal_estimation.setInputCloud(point_cloud);
    normal_estimation.setSearchMethod(search_tree);
    normal_estimation.setKSearch(20);

    // Perform normal estimation algorithm
    normal_estimation.compute(*normals);

    // Reverse the direction of all normals
    for (size_t i = 0; i < normals->size(); ++i) {
        normals->points[i].normal_x *= -1;
        normals->points[i].normal_y *= -1;
        normals->points[i].normal_z *= -1;
    }

    return normals;
}

void poisson_reconstruction(pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud, pcl::PolygonMesh& mesh)
{
    std::cout << "Begin poisson reconstruction" << std::endl;

    // Initialize poisson reconstruction
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(point_cloud);

    // Perform the poisson surface reconstruction algorithm
    poisson.reconstruct(mesh);
}

bool mesh_point_cloud(MeshPointCloudRequest &request, MeshPointCloudResponse &response)
{
    // Mesh point cloud in request and output it through response
    PointCloud::Ptr point_cloud(new PointCloud);
    pcl::fromROSMsg(request.point_cloud, *point_cloud);


    NormalCloud::Ptr normals = estimate_normals(point_cloud);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*point_cloud, *normals, *cloud_with_normals);

    pcl::PolygonMesh mesh;
    poisson_reconstruction(cloud_with_normals, mesh);

    if (pcl::io::savePolygonFile("mesh.stl", mesh) && pcl::io::savePolygonFile("mesh.vtk", mesh)) {
        std::cout << "Saved file successfully" << std::endl;
    } else {
        std::cout << "Failed to save file" << std::endl;
    }

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mesh_generator");

    ros::NodeHandle node_handle;
    ros::ServiceServer service = node_handle.advertiseService("mesh_point_cloud", mesh_point_cloud);
    ROS_INFO("Ready to mesh point clouds");
    ros::spin();

    return 0;
}
