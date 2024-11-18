/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include "dlio/map.h"
#include <filesystem>

dlio::MapNode::MapNode(ros::NodeHandle node_handle) : nh(node_handle) {

  this->getParams();

  this->keyframe_sub = this->nh.subscribe("keyframes", 10,
      &dlio::MapNode::callbackKeyframe, this, ros::TransportHints().tcpNoDelay());
  this->map_pub = this->nh.advertise<sensor_msgs::PointCloud2>("map", 100);
  // this->save_pcd_srv = this->nh.advertiseService("save_pcd", &dlio::MapNode::savePcd, this);

  this->dlio_map = pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

}

dlio::MapNode::~MapNode() {

  raise(SIGINT);
  ros::shutdown();

  // During destruction, save the map
  pcl::PointCloud<PointType>::Ptr m =
  pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>(*this->dlio_map));

  float leaf_size = 0.01;
  std::string p = ros::package::getPath("direct_lidar_inertial_odometry") + "/data";
  
  if (p.empty()) {
    std::cout << "Could not get package path using ros::package::getPath." << std::endl;
    return;
  }

  if (!std::filesystem::exists(p.c_str()))
  {
    std::filesystem::create_directories(p);

    // set the permissions of the newly created directory
    std::filesystem::permissions(
        p,
        std::filesystem::perms::owner_all | std::filesystem::perms::group_all,
        std::filesystem::perm_options::add
    );
  }

  if (!std::filesystem::is_directory(p)) {
    std::cout << "Could not find directory " << p << std::endl;
  }
  
  std::cout << std::setprecision(2) << "Saving map to " << p + "/dlio_map.pcd"
    << " with leaf size " << to_string_with_precision(leaf_size, 2) << "... "; std::cout.flush();

  // voxelize map
  pcl::VoxelGrid<PointType> vg;
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);

  if (m->empty()) {
    std::cout << "Map is empty, nothing to save." << std::endl;
    return;
  }

  vg.setInputCloud(m);
  vg.filter(*m);

  // save map
  int ret = pcl::io::savePCDFileBinary(p + "/dlio_map.pcd", *m);
  bool success = ret == 0;

  if (success) {
    std::cout << std::endl << "Map Saved" << std::endl;
  } else {
    std::cout << std::endl << "Map Saving failed" << std::endl;
  }

  exit(0);
}

void dlio::MapNode::getParams() {

  ros::param::param<std::string>("~dlio/odom/map_frame", this->map_frame, "dlio_map");
  ros::param::param<double>("~dlio/map/sparse/leafSize", this->leaf_size_, 0.5);

  // Get Node NS and Remove Leading Character
  std::string ns = ros::this_node::getNamespace();
  std::cout << "Map Node Namespace: " << ns << std::endl;

  // if (ns != "/"){
  //   ns.erase(0,1);

  //   // Concatenate Frame Name Strings
  //   this->map_frame = ns + "/" + this->map_frame;
  // }
}

void dlio::MapNode::start() {
}

void dlio::MapNode::callbackKeyframe(const sensor_msgs::PointCloud2ConstPtr& keyframe) {

  // convert scan to pcl format
  pcl::PointCloud<PointType>::Ptr keyframe_pcl =
    pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());
  pcl::fromROSMsg(*keyframe, *keyframe_pcl);

  // voxel filter
  this->voxelgrid.setLeafSize(this->leaf_size_, this->leaf_size_, this->leaf_size_);
  this->voxelgrid.setInputCloud(keyframe_pcl);
  this->voxelgrid.filter(*keyframe_pcl);

  // save filtered keyframe to map for rviz
  *this->dlio_map += *keyframe_pcl;

  // publish full map
  if (this->dlio_map->points.size() == this->dlio_map->width * this->dlio_map->height) {
    if (this->map_pub.getNumSubscribers() > 0)
    {
      sensor_msgs::PointCloud2 map_ros;
      pcl::toROSMsg(*this->dlio_map, map_ros);
      map_ros.header.stamp = ros::Time::now();
      map_ros.header.frame_id = this->map_frame;
      this->map_pub.publish(map_ros);
    }
  }
}

// bool dlio::MapNode::savePcd(direct_lidar_inertial_odometry::save_pcd::Request& req,
//                             direct_lidar_inertial_odometry::save_pcd::Response& res) {

//   pcl::PointCloud<PointType>::Ptr m =
//     pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>(*this->dlio_map));

//   float leaf_size = req.leaf_size;
//   std::string p = req.save_path;

//   if (!std::filesystem::is_directory(p)) {
//     std::cout << "Could not find directory " << p << std::endl;
//     res.success = false;
//     return false;
//   }
  
//   std::cout << std::setprecision(2) << "Saving map to " << p + "/dlio_map.pcd"
//     << " with leaf size " << to_string_with_precision(leaf_size, 2) << "... "; std::cout.flush();

//   // voxelize map
//   pcl::VoxelGrid<PointType> vg;
//   vg.setLeafSize(leaf_size, leaf_size, leaf_size);
//   vg.setInputCloud(m);
//   vg.filter(*m);

//   // save map
//   int ret = pcl::io::savePCDFileBinary(p + "/dlio_map.pcd", *m);
//   res.success = ret == 0;

//   if (res.success) {
//     std::cout << "done" << std::endl;
//   } else {
//     std::cout << "failed" << std::endl;
//   }

//   return res.success;

// }
