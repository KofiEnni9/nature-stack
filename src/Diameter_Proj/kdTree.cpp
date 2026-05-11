#include <iostream>
#include <fstream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "kdTree.h"
#include <cmath>
#include <numeric>

using namespace std;

kdNode* kdTree::insert_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcd_dataframe) 
{
    cout<<"size of dataframe: " << pcd_dataframe->points.size() << endl;
    int len = pcd_dataframe->points.size();

    std::vector<int> indices(len);
    std::iota(indices.begin(), indices.end(), 0);
    cout << "Building balanced KD-Tree..." << endl;
    root = balanced_kd_builder(pcd_dataframe, indices, 0, len, 0);

    // for(int i = 0; i < len; i++) {
    //     pcl::PointXYZ point = pcd_dataframe->points[i];
    //     int id = i;
    //     int level = 0;
    //     root = build_kdtree(root, level, point, id, len);
    // }
    cout << "KD-Tree construction completed." << endl;

    return root;
}

kdNode* kdTree::balanced_kd_builder(const pcl::PointCloud<pcl::PointXYZ>::Ptr& dataframes, std::vector<int>& indices, int left, int right, int depth) {
    if (left >= right) return nullptr;

    int axis = depth % 3;
    int mid = left + (right - left) / 2;

    // Partial sort: median element ends up at 'mid'
    std::nth_element(indices.begin() + left, indices.begin() + mid, indices.begin() + right,
        [axis, dataframes](int a, int b) {
            if (axis == 0) return dataframes->points[a].x < dataframes->points[b].x;
            if (axis == 1) return dataframes->points[a].y < dataframes->points[b].y;
            return dataframes->points[a].z < dataframes->points[b].z;
        });

    kdNode* node = new kdNode(dataframes->points[indices[mid]], indices[mid]);
    node->left  = balanced_kd_builder(dataframes, indices, left, mid, depth + 1);
    node->right = balanced_kd_builder(dataframes, indices, mid + 1, right, depth + 1);
    return node;
}


kdNode* kdTree::build_kdtree(kdNode* node, int depth_, pcl::PointXYZ point, int point_id) 
{
    // balanced kdtree

    // cout<<"Building KD-Tree at depth: " << node_count++ << endl;
    if(!node) {
        node = new kdNode(point, point_id);
        return node;
    }

    // kdNode* current_node = new kdNode(point, point_id);
    int depth = depth_ % 3;

    float node_val = (depth == 0) ? node->point.x : (depth == 1) ? node->point.y : node->point.z;
    float currNode_val = (depth == 0) ? point.x : (depth == 1) ? point.y : point.z;


    if(node_val <= currNode_val) {
        node->right = build_kdtree(node->right, depth + 1, point, point_id);
    }
    else{
        node->left = build_kdtree(node->left, depth + 1, point, point_id);
    }

    return node;
}


std::vector<int> kdTree::search_elements(kdNode* node, int depth, pcl::PointXYZ search_point, double distance_threshold)
{
    std::vector<int> kdtree_search_results;
    if(!node) return kdtree_search_results; 

    int axis = depth % 3;

    float dx = node->point.x - search_point.x;
    float dy = node->point.y - search_point.y;
    float dz = node->point.z - search_point.z;


    if(!((node->point.x < search_point.x + distance_threshold) and (node->point.x > search_point.x - distance_threshold)
            and (node->point.y < search_point.y + distance_threshold) and (node->point.y > search_point.y - distance_threshold) 
            and (node->point.z < search_point.z + distance_threshold) and (node->point.z > search_point.z - distance_threshold)))
        {
            return kdtree_search_results;
        }

    float point_distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2) + std::pow(dz, 2));

    if(point_distance <= distance_threshold) {
        kdtree_search_results.push_back(node->point_id);
    }

    float node_val = (axis == 0) ? node->point.x : (axis == 1) ? node->point.y : node->point.z;
    float search_val = (axis == 0) ? search_point.x : (axis == 1) ? search_point.y : search_point.z;
    float diff = search_val - node_val;

    // always search the side the point is on
    kdNode* first  = (diff >= 0) ? node->right : node->left;
    kdNode* second = (diff >= 0) ? node->left  : node->right;

    auto first_results = search_elements(first, depth + 1, search_point, distance_threshold);
    kdtree_search_results.insert(kdtree_search_results.end(), first_results.begin(), first_results.end());

    // only search the OTHER side if the sphere crosses the splitting plane
    if (std::abs(diff) <= distance_threshold) {
        auto second_results = search_elements(second, depth + 1, search_point, distance_threshold);
        kdtree_search_results.insert(kdtree_search_results.end(), second_results.begin(), second_results.end());
    }

    return kdtree_search_results;
}

