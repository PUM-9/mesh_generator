//
// A client that calls the mesh service to test it
//
#include "ros/ros.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include "mesh_generator/MeshPointCloud.h"
#include <cstdlib>

typedef mesh_generator::MeshPointCloud MeshPointCloud;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mesh_generator_test_client"); //inits the node

    ros::NodeHandle node_handle;
    ros::ServiceClient client = node_handle.serviceClient<mesh_generator::MeshPointCloud>("mesh_point_cloud");

    mesh_generator::MeshPointCloud srv;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); //allocates memory to a new point cloud
    pcl::PCLPointCloud2 cloud_blob;
    if (pcl::io::loadPCDFile("bun0.pcd", cloud_blob) == -1) { // loads the file bun0.pcd into the  a tmp point cloud
        std::cout << "Couldn't read file bun0.pcd" << std::endl;
    }
    pcl::fromPCLPointCloud2(cloud_blob, *cloud); // saves the tmp point cloud cloud_blob as the point cloud cloud

    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud); // converts the point cloud from PCL to ROS
    srv.request.point_cloud = ros_cloud;

    if (client.call(srv)) {
        std::cout << "Exit code: " << srv.response.exit_code << std::endl;
    }
    else {
        ROS_ERROR("Failed to call service mesh_point_cloud");
        return 1;
    }

    return 0;
}
