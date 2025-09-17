#include "utils.h"
pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::velo2PCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int filter_num, double min_range, double max_range)
{

    pcl::PointCloud<pcl::PointXYZI> cloud_xyz;
    pcl::fromROSMsg(*msg, cloud_xyz);

    int point_num = msg->width * msg->height;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    cloud->reserve(cloud_xyz.size() / filter_num + 1);

    // 2. 필터링 및 pcl::PointXYZINormal 생성
    for (size_t i = 0; i < cloud_xyz.size(); i += filter_num)
    {
        const auto &pt = cloud_xyz.points[i];
        float dist_sq = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (dist_sq < min_range * min_range || dist_sq > max_range * max_range)
            continue;

        pcl::PointXYZINormal p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = pt.intensity;
        p.normal_x = 0.0f;
        p.normal_y = 0.0f;
        p.normal_z = 0.0f;
        // curvature, normal은 별도 계산이 필요함.
        p.curvature = 0.0f; // 일단 0으로 세팅하거나, 필요 시 계산

        cloud->push_back(p);
    }

    return cloud;
}

double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}
