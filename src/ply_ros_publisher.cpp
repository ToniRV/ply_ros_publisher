#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "ply_ros_publisher/ply_ros_publisher.h"

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ply_ros_publisher");

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_ ("~");

  std::string ply_filepath;
  if (!nh_private_.getParam("ply_filepath", ply_filepath)) {
    ROS_FATAL("No PLY filepath provided for! Set the \"ply_filepath\" param.");
  }
  std::string frame_id;
  if (!nh_private_.getParam("frame_id", frame_id)) {
    ROS_FATAL("No PLY filepath provided for! Set the \"frame_id\" param.");
  }
  int load_ply_type = 1;
  ROS_ASSERT(nh_private_.getParam("load_ply_type", load_ply_type));

  pcl::PointCloud<pcl::PointXYZI>::Ptr ply_intensity_cloud =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ply_cloud =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::PolygonMesh::Ptr ply_mesh = boost::make_shared<pcl::PolygonMesh>();


  ros::Publisher ply_mesh_pub =
      nh_.advertise<pcl_msgs::PolygonMesh>("ply_mesh", 1, true);
  ros::Publisher pcl_pub =
      nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("ply_pointcloud",
                                                       1, true);
  ros::Publisher pcl_intensirty_pub =
      nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("ply_intensity_pointcloud",
                                                     1, true);

  pcl::PLYReader ply_reader;


  // Load transformations.
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  XmlRpc::XmlRpcValue T_V_G_xml;
  if (nh_private_.getParam("T_V_G", T_V_G_xml)) {
    ROS_ASSERT(T_V_G_xml.size() != 4);
    // Read raw inputs
    for (size_t i = 0; i < 4; ++i) {
      ROS_ASSERT(T_V_G_xml[i].size() == 4);
      for (size_t j = 0; j < 4; ++j) {
        transform(i, j) = static_cast<double>(T_V_G_xml[i][j]);
      }
    }

    std::cout << "eigen matrix before inversion: " << transform << std::endl;
    bool invert_static_tranform = false;
    nh_private_.param("invert_T_V_G", invert_static_tranform,
                      invert_static_tranform);
    if (invert_static_tranform) {
      transform = transform.inverse();
    }
    std::cout << "eigen matrix after inversion: " << transform << std::endl;
  }

  Eigen::Vector4f origin (0.0,0.0,0.0,0.0);
  Eigen::Quaternionf quaternion = Eigen::Quaternionf::Identity();
  int ply_version = 1;
  int offset = 0;
  if (load_ply_type == 0) {
    ROS_ERROR_STREAM("Loading mesh file " << ply_filepath.c_str());
    ply_reader.read(ply_filepath, *ply_mesh, origin, quaternion, ply_version, offset);
    pcl_msgs::PolygonMesh pcl_mesh_msg;
    pcl_conversions::fromPCL(*ply_mesh, pcl_mesh_msg);
    pcl_mesh_msg.header.frame_id = frame_id;
    ply_mesh_pub.publish(pcl_mesh_msg);
  } else if (load_ply_type == 1) {
    ROS_ERROR_STREAM("Loading pointcloud file " << ply_filepath.c_str());
    ply_reader.read(ply_filepath, *ply_cloud);
    ply_cloud->header.frame_id = frame_id;
    pcl::transformPointCloud(*ply_cloud, *ply_cloud, transform);
    pcl_pub.publish(*ply_cloud);
  } else if (load_ply_type == 2) {
    ROS_ERROR_STREAM("Loading intensity pointcloud file " << ply_filepath.c_str());
    ply_reader.read(ply_filepath, *ply_intensity_cloud);
    ply_intensity_cloud->header.frame_id = frame_id;
    pcl::transformPointCloud(*ply_intensity_cloud, *ply_intensity_cloud,
                             transform);
    pcl_intensirty_pub.publish(*ply_intensity_cloud);
  }

  ros::spin();

  return EXIT_SUCCESS;
}
