#include <ros/ros.h>

#include "ply_ros_publisher/ply_ros_publisher.h"

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ply_ros_publisher");

  ros::NodeHandle nh_private_ ("~");

  std::string ply_filepath;
  if (!nh_private_.getParam("ply_filepath", ply_filepath)) {
    ROS_FATAL("No PLY filepath provided for! Set the \"ply_filepath\" param.");
  }
  std::string frame_id;
  if (!nh_private_.getParam("frame_id", frame_id)) {
    ROS_FATAL("No PLY filepath provided for! Set the \"frame_id\" param.");
  }

  pcl::PolygonMesh ply_mesh;
  pcl::PLYReader ply_reader;
  Eigen::Vector4f origin (0.0,0.0,0.0,0.0);
  Eigen::Quaternionf quaternion = Eigen::Quaternionf::Identity();
  int ply_version = 1;
  int offset = 0;
  if (ply_reader.read(ply_filepath, ply_mesh, origin, quaternion, ply_version, offset), 0) {
    ROS_FATAL("Could not load PLY file.");
  }

  pcl_msgs::PolygonMesh pcl_mesh_msg;
  pcl_conversions::fromPCL(ply_mesh, pcl_mesh_msg);

  ros::Publisher ply_pub_ =
      nh_private_.advertise<pcl_msgs::PolygonMesh>("ply_mesh", 1, true);
  //  pcl_mesh_msg.header.frame_id = frame_id;
  ply_pub_.publish(pcl_mesh_msg);

  ros::spin();

  return EXIT_SUCCESS;
}
