#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include "euclidean_cluster.h"
#include <cmath>
#include <queue>
#include <unordered_map>
#include <tuple>

using namespace std;

// Hash for 3D integer grid cell key
struct GridKeyHash {
    size_t operator()(const std::tuple<int,int,int>& k) const {
        auto h1 = std::hash<int>{}(std::get<0>(k));
        auto h2 = std::hash<int>{}(std::get<1>(k));
        auto h3 = std::hash<int>{}(std::get<2>(k));
        return h1 ^ (h2 * 2654435761u) ^ (h3 * 2246822519u);
    }
};

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> EuclideanCluster::euclideanclusterinf(double distance_threshold, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    cout << "beginning of euclideanclusterinf" << endl;

    this->pcd_data = cloud;
    this->threshold = distance_threshold;

    const int N = cloud->points.size();
    cout << "point size: " << N << endl;

    // ── Step 1: Bucket every point into a 3D voxel of size = threshold ──
    using GridKey = std::tuple<int,int,int>;
    std::unordered_map<GridKey, std::vector<int>, GridKeyHash> grid;
    grid.reserve(N);

    auto cell = [&](float v) { return (int)std::floor(v / distance_threshold); };

    for (int i = 0; i < N; i++) {
        auto& p = cloud->points[i];
        grid[{cell(p.x), cell(p.y), cell(p.z)}].push_back(i);
    }

    // ── Step 2: Cluster at VOXEL level — each voxel is one BFS node ──
    // Two voxels are neighbors if any of their 26 face/edge/corner neighbors exist.
    // Since voxel size = threshold, adjacent voxels always contain points within threshold.
    // This reduces 80K points → ~N_voxels (often <500) nodes to BFS over.
    cout << "Clustering " << grid.size() << " voxels..." << endl;

    std::unordered_map<GridKey, int, GridKeyHash> voxel_cluster; // voxel → cluster id
    voxel_cluster.reserve(grid.size());
    int cluster_id = 0;

    for (auto& [key, _] : grid) {
        if (voxel_cluster.count(key)) continue;

        // BFS over voxels
        std::queue<GridKey> q;
        q.push(key);
        voxel_cluster[key] = cluster_id;

        while (!q.empty()) {
            auto [cx, cy, cz] = q.front(); q.pop();
            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                    for (int dz = -1; dz <= 1; dz++) {
                        GridKey nb{cx+dx, cy+dy, cz+dz};
                        if (grid.count(nb) && !voxel_cluster.count(nb)) {
                            voxel_cluster[nb] = cluster_id;
                            q.push(nb);
                        }
                    }
        }
        cluster_id++;
    }

    // ── Step 3: Map original points back to their voxel's cluster ──
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_identified(cluster_id);
    for (int i = 0; i < cluster_id; i++)
        clusters_identified[i].reset(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < N; i++) {
        auto& p = cloud->points[i];
        GridKey k{cell(p.x), cell(p.y), cell(p.z)};
        clusters_identified[voxel_cluster[k]]->points.push_back(p);
    }

    // Remove clusters below min_size and finalize metadata
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> result;
    for (auto& c : clusters_identified) {
        if ((int)c->points.size() > min_size) {
            c->width = c->points.size();
            c->height = 1;
            c->is_dense = true;
            result.push_back(c);
        }
    }
    return result;
}

// find_clusters is now inlined above; stub kept for linkage
void EuclideanCluster::find_clusters(std::vector<int>&, int, std::vector<bool>&) {}




pcl::PointCloud<pcl::PointXYZRGBL>::Ptr make_colored_pcd_label(
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_identified,
    float* store_Diameter)
{
    int len = clusters_identified.size();
    cout << "Number of clusters: " << len << endl;

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr colored_pcd(new pcl::PointCloud<pcl::PointXYZRGBL>);

    for (int i = 0; i < len; i++) {
        
        // if (store_Diameter[i] < 0.3) {
            // only consider cluster with diameter less than 0.5m

        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster = clusters_identified[i];
        for (auto& point : cluster->points) {
            pcl::PointXYZRGBL colored_point;
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;
            colored_point.r = (i & 1) ? 255 : 0;
            colored_point.g = (i & 2) ? 255 : 0;
            colored_point.b = (i & 4) ? 255 : 0;
            colored_point.label = (store_Diameter != nullptr)
                ? static_cast<uint32_t>(store_Diameter[i] * 1000)  // store mm as integer
                : 0;
            colored_pcd->points.push_back(colored_point);
        }
        // }
    }

    colored_pcd->width  = static_cast<uint32_t>(colored_pcd->points.size());
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

// g++ -std=c++17 -O2 -pthread kdTree.cpp euclidean_cluster.cpp ransac.cpp leastsqrfitting.cpp main.cpp ../perception/elevation_grid.cpp \
//   -I/scratch/ld212/bag_files/nature-stack/pcl-install/include/pcl-1.15 \
//   -I/scratch/ld212/bag_files/nature-stack/flann-install/include \
//   -I/scratch/ld212/bag_files/nature-stack/include \
//   -L/scratch/ld212/bag_files/nature-stack/pcl-install/lib \
//   -L/scratch/ld212/bag_files/nature-stack/flann-install/lib \
//   -Wl,-rpath,/scratch/ld212/bag_files/nature-stack/pcl-install/lib \
//   -Wl,-rpath,/scratch/ld212/bag_files/nature-stack/flann-install/lib \
//   -lpcl_common -lpcl_io -lpcl_octree -lpcl_kdtree -lpcl_search \
//   -lpcl_features -lpcl_filters -lpcl_segmentation -lpcl_sample_consensus \
//   -lflann_cpp -lboost_system -lboost_filesystem \
//   -DEIGEN_MAX_ALIGN_BYTES=32 \
//   -o outputpcd