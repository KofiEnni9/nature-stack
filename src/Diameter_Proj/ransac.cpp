#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <random>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>


typedef pcl::PointXYZ PointT;

pcl::PointCloud<pcl::PointXYZ>::Ptr Ransac_tree_trunks(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudinput, int iterations, float thresh)
{
    // seed random
    std::srand(std::time(nullptr));

    std::cerr << "PointCloud has: " << cloudinput->size() << " data points." << std::endl;

    int n_points = static_cast<int>(cloudinput->points.size());

    std::vector<int> best_inliers;
    Eigen::Vector3f best_center(0, 0, 0);
    Eigen::Vector3f best_axis(0, 0, 1);
    float best_radius = 0.0f;

    for(int i = 0; i < iterations; i++)
    {
        // sample 3 distinct random points
        std::vector<int> id_samples(3);
        id_samples[0] = rand() % n_points;
        do { id_samples[1] = rand() % n_points; } while (id_samples[1] == id_samples[0]);
        do { id_samples[2] = rand() % n_points; } while (id_samples[2] == id_samples[0] || id_samples[2] == id_samples[1]);

        Eigen::Vector3f p0(cloudinput->points[id_samples[0]].x, cloudinput->points[id_samples[0]].y, cloudinput->points[id_samples[0]].z);
        Eigen::Vector3f p1(cloudinput->points[id_samples[1]].x, cloudinput->points[id_samples[1]].y, cloudinput->points[id_samples[1]].z);
        Eigen::Vector3f p2(cloudinput->points[id_samples[2]].x, cloudinput->points[id_samples[2]].y, cloudinput->points[id_samples[2]].z);

        // We have to find the plane equation described by those 3 points
        // We find first 2 vectors that are part of this plane
        // A = pt2 - pt1
        // B = pt3 - pt1

        Eigen::Vector3f vecA = p1 - p0;
        Eigen::Vector3f vecA_norm = vecA.normalized();
        Eigen::Vector3f vecB = p2 - p0;
        Eigen::Vector3f vecB_norm = vecB.normalized();

        Eigen::Vector3f vecC = vecA_norm.cross(vecB_norm);
        vecC.normalize();

        // Skip degenerate (collinear) samples
        if (vecC.norm() < 1e-6f) continue;

        // Now we calculate the rotation of the points with rodrigues equation
        Eigen::Matrix3f R = Eigen::Quaternionf::FromTwoVectors(vecC, Eigen::Vector3f(0, 0, 1)).toRotationMatrix();

        // Rotate the 3 sample points into the XY plane
        Eigen::Vector3f r0 = R * p0;
        Eigen::Vector3f r1 = R * p1;
        Eigen::Vector3f r2 = R * p2;

        // Find circumcenter in the XY plane
        float ma = 0.0f, mb = 0.0f;
        // Shuffle order until slopes are non-zero
        Eigen::Vector3f rA = r0, rB = r1, rC = r2;
        for (int attempt = 0; attempt < 3; ++attempt)
        {
            float denom_a = rB.x() - rA.x();
            float denom_b = rC.x() - rB.x();
            if (std::abs(denom_a) > 1e-6f && std::abs(denom_b) > 1e-6f)
            {
                ma = (rB.y() - rA.y()) / denom_a;
                mb = (rC.y() - rB.y()) / denom_b;
                break;
            }
            // rotate the sample order and try again
            Eigen::Vector3f tmp = rA; rA = rB; rB = rC; rC = tmp;
        }

        if (std::abs(mb - ma) < 1e-6f) continue; // parallel bisectors, skip

        float p_center_x = (
            ma * mb * (rA.y() - rC.y())
            + mb * (rA.x() + rB.x())
            - ma * (rB.x() + rC.x())
        ) / (2.0f * (mb - ma));

        float p_center_y = -1.0f / ma * (p_center_x - (rA.x() + rB.x()) / 2.0f) + (rA.y() + rB.y()) / 2.0f;

        Eigen::Vector3f p_center_rot(p_center_x, p_center_y, 0.0f);
        float radius = (p_center_rot - Eigen::Vector3f(rA.x(), rA.y(), 0.0f)).norm();

        // The cylinder axis in world space is vecC; cylinder center in world space
        Eigen::Vector3f center_world = R.transpose() * p_center_rot;

        // Count inliers: distance from each point to the cylinder axis
        std::vector<int> pt_id_inliers;
        for (int j = 0; j < n_points; ++j)
        {
            Eigen::Vector3f pt(cloudinput->points[j].x, cloudinput->points[j].y, cloudinput->points[j].z);
            // Vector from center_world to pt, then remove component along axis
            Eigen::Vector3f diff = pt - center_world;
            float along_axis = diff.dot(vecC);
            Eigen::Vector3f perp = diff - along_axis * vecC;
            float dist = perp.norm();
            if (std::abs(dist - radius) <= thresh)
                pt_id_inliers.push_back(j);
        }

        if (pt_id_inliers.size() > best_inliers.size())
        {
            best_inliers = pt_id_inliers;
            best_center  = center_world;
            best_axis    = vecC;
            best_radius  = radius;
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& idx : best_inliers)
    {
        result->points.push_back(cloudinput->points[idx]);
    }
    result->width  = static_cast<uint32_t>(result->points.size());
    result->height = 1;
    result->is_dense = true;

    return result;
}

