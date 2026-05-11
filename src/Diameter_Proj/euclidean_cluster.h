#ifndef EUCLIDEAN_CLUSTER_H
#define EUCLIDEAN_CLUSTER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>

class EuclideanCluster
{
    private:
    int min_size;
    double threshold;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_data;

    public:

    EuclideanCluster() : min_size(10), threshold(0.09) {
        pcd_data = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    void find_clusters(std::vector<int>& base_cluster, int index, std::vector<bool>& processed_flag);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> euclideanclusterinf(double distance_threshold, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    pcl::PointXYZ get_point(int index) { return pcd_data->points[index]; }

};

pcl::PointCloud<pcl::PointXYZRGBL>::Ptr make_colored_pcd_label(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_identified, float* store_Diameter);

#endif