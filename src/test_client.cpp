#include "ros/ros.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include "mesh_generator/MeshPointCloud.h"
#include <cstdlib>

typedef mesh_generator::MeshPointCloud MeshPointCloud;

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

    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);
    srv.request.point_cloud = ros_cloud;

    if (client.call(srv)) {
        std::cout << "Exit code: " << srv.response.exit_code << std::endl;
    }
    else {
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    }

    return 0;
}
