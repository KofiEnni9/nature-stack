#ifndef KDTREE_H
#define KDTREE_H

#include <iostream>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <numeric>



struct kdNode {
    kdNode* left;
    kdNode* right;
    pcl::PointXYZ point;
    int point_id;

    kdNode(pcl::PointXYZ point, int id) : point_id(id), point(point), left(nullptr), right(nullptr) {};

};

class kdTree {
    private:

    public:
    kdNode* root;

    kdTree() : root(nullptr) {}
    // ~kdTree();
    kdNode* insert_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcd_dataframe);
    kdNode* build_kdtree(kdNode* node, int depth, pcl::PointXYZ point, int point_id);
    std::vector<int> search_elements(kdNode* node, int depth, pcl::PointXYZ search_point, double distance_threshold);
    kdNode* balanced_kd_builder(const pcl::PointCloud<pcl::PointXYZ>::Ptr& dataframes, std::vector<int>& indices, int left, int right, int depth);

};

#endif // KDTREE_H