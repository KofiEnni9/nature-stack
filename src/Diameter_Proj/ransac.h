#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <memory>

pcl::PointCloud<pcl::PointXYZ>::Ptr Ransac_tree_trunks(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudinput);