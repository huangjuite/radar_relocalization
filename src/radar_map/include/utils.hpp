#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <deque>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr_t;

Eigen::Matrix4f toMatrix(geometry_msgs::Pose pose)
{
    Eigen::Matrix4d m;
    Eigen::Affine3d a3d;
    Eigen::fromMsg(pose, a3d);
    m = a3d.matrix();
    return m.cast<float>();
}

pcl_cloud_ptr_t getTargetCloud(pcl_cloud_ptr_t &source_cloud_ptr, const Eigen::Matrix4f &source, const Eigen::Matrix4f &target)
{
    // Find the affine3d transform (P) that projects from source frame to target frame
    // So target = P * source
    Eigen::Matrix4f P;
    P = target.inverse() * source;

    // Executing the transformation
    pcl_cloud_ptr_t target_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*source_cloud_ptr, *target_cloud_ptr, P);
    return target_cloud_ptr;
}

pcl_cloud_ptr_t getDenseCloud(std::deque<pcl_cloud_ptr_t> &cloud_queue, std::deque<Eigen::Matrix4f> &transform_queue)
{
    // Return new pcl cloud pointer that merges all point clouds in queue.
    // NEED TO INITIALIZE OTHERWISE FAILED px != 0 shared pointer assumption (new pcl ...)
    pcl_cloud_ptr_t merged_clouds_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    int cloud_size = cloud_queue.size();
    int transform_size = transform_queue.size();
    if (cloud_size != transform_size)
    {
        std::cout << "Cloud size: " << cloud_size << " Transform size: " << transform_size << " mismatched" << std::endl;
        return merged_clouds_ptr;
    }

    if (cloud_size == 0)
    {
        std::cout << "No clouds yet" << std::endl;
        return merged_clouds_ptr;
    }

    pcl_cloud_ptr_t target_cloud_ptr = cloud_queue.back();
    Eigen::Matrix4f target_mat = transform_queue.back();

    if (cloud_size == 1)
    {
        return target_cloud_ptr;
    }

    for (int i = 0; i < cloud_size - 1; i++)
    {
        pcl_cloud_ptr_t projected_cloud_ptr = getTargetCloud(cloud_queue[i], transform_queue[i], target_mat);
        (*merged_clouds_ptr) += (*projected_cloud_ptr);
    }

    (*merged_clouds_ptr) += (*target_cloud_ptr);
    return merged_clouds_ptr;
}