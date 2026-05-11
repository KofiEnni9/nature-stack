#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>




int main()
{

    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2);
    pcl::io::loadPCDFile ("/scratch/ld212/bag_files/backyard/pointclouds/1680633522931820011.pcd", *cloud_blob);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (*cloud_blob, *cloud_xyz);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D (*cloud_xyz, min_pt, max_pt);

    std::cout << "Min Point: " << min_pt.x << ", " << min_pt.y << ", " << min_pt.z << std::endl;
    std::cout << "Max Point: " << max_pt.x << ", " << max_pt.y << ", " << max_pt.z << std::endl;

    // pcl::io::savePCDFile ("/home/kae257/pcd/output.pcd", *cloud_blob);
    return 0;
}
