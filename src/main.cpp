//
//
//
#include <vector>
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
#include <pcl_msgs/PolygonMesh.h>


typedef mesh_generator::MeshPointCloud::Request MeshPointCloudRequest;
typedef mesh_generator::MeshPointCloud::Response MeshPointCloudResponse;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;

/**
 * Estimates the normals for a point cloud using PCL
 *
 * @param point_cloud The input point cloud that the normals are generated for
 * @return Returns a NormalCloud::Ptr with the generated normals
 */
NormalCloud::Ptr estimate_normals(PointCloud::Ptr point_cloud)
{
    ROS_INFO("Estimating normals");

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

/**
 * Uses poisson surface reconstruction algorithm to generate a 3D-mesh from a point cloud
 * with normals
 *
 * @param point_cloud The point cloud (including normals) that is used to generate the mesh
 * @param mesh The resulting 3D-mesh is saved to mesh
 */
void poisson_reconstruction(pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud, pcl::PolygonMesh& mesh)
{
    ROS_INFO("Begin poisson surface reconstruction");

    // Initialize poisson reconstruction
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(point_cloud);

    // Perform the poisson surface reconstruction algorithm
    poisson.reconstruct(mesh);
}

/**
 * Callback function for ROS service MeshPointCloud. Generates a mesh from a point cloud
 * and sends it back as a service response.
 *
 * @param request The service request. Contains a sensor_msgs/PointCloud2 object.
 * @param response The response that is sent back to the client. Contains the generated mesh.
 * @return True if successful, false otherwise
 */
bool mesh_point_cloud(MeshPointCloudRequest &request, MeshPointCloudResponse &response)
{
    // Get the point cloud from service request
    PointCloud::Ptr point_cloud(new PointCloud);
    pcl::fromROSMsg(request.point_cloud, *point_cloud);

    NormalCloud::Ptr normals = estimate_normals(point_cloud);

    // Add the normals to the point cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*point_cloud, *normals, *cloud_with_normals);

    pcl::PolygonMesh mesh;
    poisson_reconstruction(cloud_with_normals, mesh);

    // Save the generated mesh in response to send it to client
    pcl_conversions::fromPCL(mesh, response.mesh);
    response.exit_code = 0;
    response.error_message = "";

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
