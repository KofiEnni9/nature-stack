#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include "kdTree.h"
#include "euclidean_cluster.h" 
#include <vector>
#include "kdTree.h"
#include <cmath>
#include <numeric>
// #include "nature/node/ros_types.h"
#include "nature/perception/minimal_pointcloud.h"
#include "nature/perception/elevation_grid.h"

using namespace std;


int main()
{


    cout<< "Loading PCD file..." << endl;

    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2);
    pcl::io::loadPCDFile ("/home/kae257/pcdstuff/16point.pcd", *cloud_blob);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (*cloud_blob, *cloud_xyz);

    vector<pcl::PointXYZ> points;

    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();

    // pass points to elevation points

    for (const auto& pt : cloud_xyz->points) {
        if (pt.z < min_z) min_z = pt.z;
        if (pt.z > max_z) max_z = pt.z;
    }

    cout << "Lowest altitude (min z): " << min_z << endl;
    cout << "Highest altitude (max z): " << max_z << endl;

    // diameter of breast height
    // float dbh = 1.2f;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // for (const auto& pt : cloud_xyz->points) {
    //     pcl::PointXYZ np;
    //     if (pt.z >= dbh) {
    //         np.x = pt.x;
    //         np.y = pt.y;
    //         np.z = pt.z;
    //         new_cloud->points.push_back(np);
    //     }
    // }


    // take  50% of eg height
    // float veg_cutoff = abs(abs(min_z)-abs(max_z)) * 0.8f;
    float veg_cutoff = 1.3716f; // breast height 4.5 feet

    float cutoff_area = 7.62f; // consider 25 feet


    for (int i = cloud_xyz->points.size() - 1; i >= 0; --i) {
        if (cloud_xyz->points[i].z < min_z + veg_cutoff) {
            cloud_xyz->points.erase(cloud_xyz->points.begin() + i);
        }
    }

    cloud_xyz->width = cloud_xyz->points.size();
    cloud_xyz->height = 1;
    cloud_xyz->is_dense = false;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr outputcloud (new pcl::PointCloud<pcl::PointXYZ>);
    // outputcloud->points = points;

    // cout << "About to start clustering..." << endl;

    // EuclideanCluster euc_clust;
    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> output_clustering = euc_clust.euclideanclusterinf(0.9, cloud_xyz);
    // cout << "Clustering completed." << endl;

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputcloud = make_colored_pcd(output_clustering);

    pcl::io::savePCDFile ("/home/kae257/pcdstuff/output3.pcd", *cloud_xyz);
    return 0;
}


