//
// A client that calls the mesh service to test it
//
#include "ros/ros.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl_ros/point_cloud.h>
#include "mesh_generator/MeshPointCloud.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mesh_generator_test_client");

    ros::NodeHandle node_handle;
    ros::ServiceClient client = node_handle.serviceClient<mesh_generator::MeshPointCloud>("mesh_point_cloud");

    mesh_generator::MeshPointCloud srv;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;
    if (pcl::io::loadPCDFile("bun0.pcd", cloud_blob) == -1) {
        std::cout << "Couldn't read file bun0.pcd" << std::endl;
    }
    pcl::fromPCLPointCloud2(cloud_blob, *cloud);

    // Convert point cloud to a ROS message and send it in request
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);
    srv.request.point_cloud = ros_cloud;

    if (client.call(srv)) {
        pcl::PolygonMesh mesh;
        pcl_conversions::toPCL(srv.response.mesh, mesh);
	
        if (pcl::io::savePolygonFile("mesh-response.stl", mesh)) {
	        ROS_INFO("Saved file mesh-response.stl");
        }
    }
    else {
        ROS_ERROR("Failed to call service mesh_point_cloud");
        return 1;
    }

    return 0;
}
