#include <cmath>
#include <tuple>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>

std::tuple<float, float, float, float> ransac(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3) {
    // RANSAC implementation

    float x1 = p1.x, y1 = p1.y, z1 = p1.z;
    float x2 = p2.x, y2 = p2.y, z2 = p2.z;
    float x3 = p3.x, y3 = p3.y, z3 = p3.z;

    float ax = x2 - x1; float ay = y2 - y1;
    float bx = x3 - x1; float by = y3 - y1;

    float rhs_a = (x1*x1 + y1*y1) - (x2*x2 + y2*y2);
    float rhs_b = (x1*x1 + y1*y1) - (x3*x3 + y3*y3);

    float det = ax * by - ay * bx;

    if(std::abs(det) < 1e-10) {
        // Points are collinear, return sentinel
        return std::make_tuple(0.0f, 0.0f, -1.0f, 0.0f);
    }

    float cx = (rhs_a*by - rhs_b*ay)/(2*det);
    float cy = (ax * rhs_b - bx * rhs_a) / (2*det);

    float r = std::sqrt((x1 - cx)*(x1 - cx) + (y1 - cy)*(y1 - cy));

    float cylinder_length = std::max(0.1f, r);

    return std::make_tuple(cx, cy, r, cylinder_length);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr align_tree_trunks(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    int len = cloud->points.size();

    std::vector<pcl::PointXYZ> best_inliers;
    float best_cx = 0, best_cy = 0, best_r = 0;

    int iterations = 100;
    for (int iter = 0; iter < iterations; iter++) {
        pcl::PointXYZ p1 = cloud->points[rand()%len];
        pcl::PointXYZ p2 = cloud->points[rand()%len];
        pcl::PointXYZ p3 = cloud->points[rand()%len];

        auto [cx, cy, r, cylinder_length] = ransac(p1, p2, p3);
        if (r < 0) continue; // collinear, skip

        auto dist = [&](const pcl::PointXYZ& point) {
            return std::sqrt((point.x - cx) * (point.x - cx) + (point.y - cy) * (point.y - cy));
        };

        std::vector<pcl::PointXYZ> inliers;
        for (const auto& point : cloud->points) {
            if (std::abs(dist(point) - r) < 0.01) {
                inliers.push_back(point);
            }
        }

        if (inliers.size() > best_inliers.size()) {
            best_inliers = inliers;
            best_cx = cx; best_cy = cy; best_r = r;
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr res_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : best_inliers) {
        res_cloud->points.push_back(point);
    }

    return res_cloud;
}

