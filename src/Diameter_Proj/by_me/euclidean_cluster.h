#include <iostream>
#include <fstream>
#include <pcl/point_types.h>
#include "kdTree.h"
#include <cmath>
#include <pcl/point_cloud.h>
#include <vector>



class EuclideanCluster
{
    private:
    int min_size;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_data;

    public:

    // pcl::PointCloud<pcl::PointXYZ> pcd_data;

    kdTree* kdtree;



    void find_clusters(pcl::PointXYZ current_point, vector<int>& base_cluster, int index, double threshold, int min_size, vector<bool>& processed_flag);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> euclideanclusterinf(double distance_threshold, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    pcl::PointXYZ get_point(int index) { return pcd_data[index].point; }

};

pcl::PointCloud<pcl::PointXYZRGB>::Ptr make_colored_pcd(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_identified);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr make_colored_pcd_label(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_identified, float* store_Diameter);
