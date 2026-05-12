#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include "kdTree.h"
#include "euclidean_cluster.h"
#include <cmath>
#include <queue>

using namespace std;

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> EuclideanCluster::euclideanclusterinf(double distance_threshold, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    cout<<"begining of euclideanclusterinf"<<endl;
    // Store cloud data for get_point() to work
    this->pcd_data = cloud;
    this->kdtree->insert_points(cloud);
    cout<<"after insertion into kdtree"<<endl;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_identified;

    // int cluster_id = 0;

    std::vector<bool> processed_flag(cloud->points.size(), false);

    cout << "point size: " << cloud->points.size() << endl;
    for(int index = 0; index < cloud->points.size(); index++) {

        if(!processed_flag[index]) {
            std::vector<int> base_cluster;
            find_clusters(cloud->points[index], base_cluster, index, threshold, min_size, processed_flag);

            if(base_cluster.size() > min_size) {
                // Create a new point cloud from the indices
                pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                for(int idx : base_cluster) {
                    cluster_cloud->points.push_back(cloud->points[idx]);
        }
                cluster_cloud->width = cluster_cloud->points.size();
                cluster_cloud->height = 1;
                cluster_cloud->is_dense = true;
                clusters_identified.push_back(cluster_cloud);
            }
        }
    }
    return clusters_identified;
}

void EuclideanCluster::find_clusters(pcl::PointXYZ current_point, std::vector<int>& base_cluster, int index, double threshold, int min_size, std::vector<bool>& processed_flag)
{
    std::queue<int> to_process;

    to_process.push(index);
    processed_flag[index] = true;

    while(!to_process.empty()) {
        int curr_index = to_process.front();
        to_process.pop();

        base_cluster.push_back(curr_index);

        pcl::PointXYZ point = get_point(curr_index);
        std::vector<int>  nearby_points = kdtree->search_elements(kdtree->root, 0, point, threshold);

        for(int idx : nearby_points) {
            if(!processed_flag[idx]) {
                processed_flag[idx] = true;
                to_process.push(idx);
            }
        }
    }
}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr make_colored_pcd(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_identified) 
{
    int len = clusters_identified.size();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_pcd(new pcl::PointCloud<pcl::PointXYZRGB>);


    for (int i = 0; i < len; i++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster = clusters_identified[i];
        for (auto& point : cluster->points) {
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;
            colored_point.r = (i & 1) ? 255 : 0;
            colored_point.g = (i & 2) ? 255 : 0;
            colored_point.b = (i & 4) ? 255 : 0;
            colored_pcd->points.push_back(colored_point);
        }
    }

    // ensure width * height == points.size()
    colored_pcd->width = static_cast<uint32_t>(colored_pcd->points.size());
    colored_pcd->height = (colored_pcd->width == 0) ? 0 : 1;
    colored_pcd->is_dense = true;

    return colored_pcd;
}


// int main()
// {

//     cout<< "Loading PCD file..." << endl;

//     pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2);
//     pcl::io::loadPCDFile ("/home/kae257/pcdstuff/16point.pcd", *cloud_blob);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromPCLPointCloud2 (*cloud_blob, *cloud_xyz);

//     cout << "About to start clustering..." << endl;

//     EuclideanCluster euc_clust;
//     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> output_clustering = euc_clust.euclideanclusterinf(0.9, cloud_xyz);
//     cout << "Clustering completed." << endl;

//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputcloud = make_colored_pcd(output_clustering);

//     pcl::io::savePCDFile ("/home/kae257/pcdstuff/output.pcd", *outputcloud);
//     return 0;
// }




// g++ -std=c++17 -O2 -pthread kdTree.cpp euclidean_cluster.cpp main.cpp \
//   -I/scratch/ld212/bag_files/nature-stack/pcl-install/include/pcl-1.15 \
//   -I/scratch/ld212/bag_files/nature-stack/include \
//   -L/scratch/ld212/bag_files/nature-stack/pcl-install/lib \
//   -Wl,-rpath,/scratch/ld212/bag_files/nature-stack/pcl-install/lib \
//   -lpcl_common -lpcl_io -lpcl_search -lpcl_kdtree -lboost_system -lboost_filesystem \
//   -o outputpcd