//
// A client that calls the mesh service to test it
// Needs the files lamppost.pcd, ism_test_cat.pcd and ism_test_wolf.pcd
// to send to the meshing service
//
#include "ros/ros.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl_ros/point_cloud.h>
#include "mesh_generator/MeshPointCloud.h"
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mesh_generator_test_client");

    ros::NodeHandle node_handle;
    ros::ServiceClient client = node_handle.serviceClient<mesh_generator::MeshPointCloud>("mesh_point_cloud");

    mesh_generator::MeshPointCloud srv;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lamp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cat(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_wolf(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::PCLPointCloud2 cloud_blob;
    if (pcl::io::loadPCDFile("lamppost.pcd", cloud_blob) == -1) {
        std::cout << "Couldn't read file bun0.pcd" << std::endl;
    }
    pcl::fromPCLPointCloud2(cloud_blob, *cloud_lamp);

    if (pcl::io::loadPCDFile("ism_test_cat.pcd", cloud_blob) == -1) {
        std::cout << "Couldn't read file bun0.pcd" << std::endl;
    }
    pcl::fromPCLPointCloud2(cloud_blob, *cloud_cat);

    if (pcl::io::loadPCDFile("ism_test_wolf.pcd", cloud_blob) == -1) {
        std::cout << "Couldn't read file bun0.pcd" << std::endl;
    }
    pcl::fromPCLPointCloud2(cloud_blob, *cloud_wolf);

    
    // Convert point cloud to a ROS message and send it in request
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud_lamp, ros_cloud);
    srv.request.point_cloud = ros_cloud;

    for (size_t i = 0; i < 3; ++i) {
        if (client.call(srv)) {
	    pcl::PolygonMesh mesh;
	    pcl_conversions::toPCL(srv.response.mesh, mesh);

	    std::stringstream file_name;
	    file_name << "mesh-response" << i << ".stl";
	    if (pcl::io::savePolygonFile(file_name.str(), mesh)) {
	        ROS_INFO("Saved file mesh-responseX.stl");
	    }

	    if (i == 0) {
	        pcl::toROSMsg(*cloud_cat, ros_cloud);
		srv.request.point_cloud = ros_cloud;
	    } else if (i == 1) {
	        pcl::toROSMsg(*cloud_wolf, ros_cloud);
		srv.request.point_cloud = ros_cloud;
	    }
	} else {
	    ROS_ERROR("Failed to call service mesh_point_cloud");
            return 1;
	}
    }

    return 0;
}
