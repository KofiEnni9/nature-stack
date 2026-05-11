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
#include <cmath>
#include <numeric>
#include <algorithm>
#include <chrono>
#include "nature/perception/minimal_pointcloud.h"
#include "nature/perception/elevation_grid.h"
#include "ransac.h"
#include "leastsqrfitting.cpp"

using namespace std;
using clk = std::chrono::high_resolution_clock;
#define TICK(x) auto _t_##x = clk::now()
#define TOCK(x) cout << #x ": " << std::chrono::duration_cast<std::chrono::milliseconds>(clk::now()-_t_##x).count() << " ms\n"


// Elevation Difference Filtering. It's one of the simplest LiDAR ground removal techniques.

// void elevation_diff_filt(){
//     int res = 0.4;

//     auto cell = [&](float v) { return (int)std::floor(v / res); };

//     for(int i = 0; i < cloud_xyz->points.size(); ++i) {
//         const auto& pt = cloud_xyz->points[i];
//         int cell_x = cell(pt.x);
//         int cell_y = cell(pt.y);
//         int cell_z = cell(pt.z);
//         // Do something with the cell indices
//     }
// }



int main()
{
    cout << "Loading PCD file..." << endl;
    TICK(load);
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2);
    // pcl::io::loadPCDFile ("/home/kae257/pcdstuff/16point.pcd", *cloud_blob);
    pcl::io::loadPCDFile ("/home/kae257/pcdstuff/point000.pcd", *cloud_blob);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (*cloud_blob, *cloud_xyz);
    TOCK(load);

    // Convert PCL cloud to nature::msg::PointCloud
    nature::msg::PointCloud nat_cloud;
    nat_cloud.points.reserve(cloud_xyz->points.size());
    for (const auto& pt : cloud_xyz->points) {
        nat_cloud.points.push_back({pt.x, pt.y, pt.z});
    }

   
    // Run elevation grid ground removal
    nature::perception::ElevationGrid eg;
    eg.SetSlopeThreshold(0.3f);
    eg.SetRes(0.4f);      // cell resolution in meters
    /* nat_cloud.points will be replaced with obstacle (non-ground) points;
       surface_points are the ground returns */
    auto surface_points = eg.AddPoints(nat_cloud);

    cout << "Obstacle points after ground removal: " << nat_cloud.points.size() << endl;

    // Convert back to PCL for clustering
    cloud_xyz->clear();
    for (const auto& pt : nat_cloud.points) {
        cloud_xyz->points.push_back({pt.x, pt.y, pt.z});
    }
    cloud_xyz->width = cloud_xyz->points.size();
    cloud_xyz->height = 1;
    cloud_xyz->is_dense = false;

    float min_z = std::numeric_limits<float>::max();
    // float max_z = std::numeric_limits<float>::lowest();

    for (const auto& pt : cloud_xyz->points) {
        if (pt.z < min_z) min_z = pt.z;
        // if (pt.z > max_z) max_z = pt.z;
    }

    // cout << "Lowest altitude (min z): " << min_z << endl;
    // cout << "Highest altitude (max z): " << max_z << endl;

    float veg_cutoff = 2.7432; // breast height limit points 9 feet
    // float vas[2] = {0.9144, 1.8288};  // 3ft - 6ft
    float vas[2] = {1.0668, 1.6764};  // 3.5ft - 5.5ft
    // float vas[2] = {1.0668, 1.3716};
    // float vas[2] = {1.3716, 1.6764};
    // float vas[2] = {1.2192, 1.524};

    float min_z_thresh = min_z + vas[0];
    float max_z_thresh = min_z + vas[1];

    auto& pts = cloud_xyz->points;
    pts.erase(std::remove_if(pts.begin(), pts.end(), [&](const pcl::PointXYZ& p) {
        // return (p.z > min_z_thresh);
        return !(p.z >= min_z_thresh && p.z <= max_z_thresh);
    }), pts.end());

    cout << "About to start clustering..." << endl;
    EuclideanCluster euc_clust;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> output_clustering = euc_clust.euclideanclusterinf(0.1, cloud_xyz);
    cout << "Clustering completed." << endl;
    int len = output_clustering.size();
    float store_Diameter[len];

    for (int i = 0; i < len; i++) {
        auto cluster = output_clustering[i];
        // std::cout << "Cluster " << i << " has " << cluster->points.size() << " points." << std::endl;
        // auto& pts = cluster;
        // pts->points.erase(std::remove_if(pts->points.begin(), pts->points.end(), [&](const pcl::PointXYZ& p) {
        //     return (p.z >= 1.31064 && p.z <= 1.43256);  // remove points OUTSIDE DBH range
        // }), pts->points.end());

        // cluster = pts;

        // std::cout << "Cluster2 " << i << " has " << cluster->points.size() << " points." << std::endl;

        int pointCount = cluster->points.size();
        glm::vec3* points = (glm::vec3*)malloc(pointCount * sizeof(glm::vec3));
        for(int i = 0; i < pointCount; ++i) {
            points[i] = glm::vec3(cluster->points[i].x, cluster->points[i].y, cluster->points[i].z);
        }

        // ---- Run the cylinder fitting ----
        float rSqr;
        glm::vec3 C, W;
        
        // find circle Diameter at DBH 4.5 feet = 1.37
        // float errorCircle = FitCircle(pointCount, points, rSqr, C, W);

        // height 2.5 - 6.5 
        // fit cylinder 
        float error = FitCylinder(pointCount, points, rSqr, C, W);
        printf("\n=== Fitted Cylinder ===\n");
        printf("  Axis:   (%.3f, %.3f, %.3f)\n", W.x, W.y, W.z);
        printf("  Center: (%.3f, %.3f, %.3f)\n", C.x, C.y, C.z);
        printf("  Radius: %.3f  Diameter: %.3f meters\n", sqrt(rSqr), 2 * sqrt(rSqr));
        printf("  Error:  %e\n", error);
        store_Diameter[i] = 2 * sqrt(rSqr);

        // for (auto& cluster : output_clustering) {
        //     cluster = Ransac_tree_trunks(cluster);
        // }
    }


    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr outputcloud = make_colored_pcd_label(output_clustering, store_Diameter);
    pcl::io::savePCDFile ("/home/kae257/pcdstuff/output2.pcd", *outputcloud);
    return 0;
}
