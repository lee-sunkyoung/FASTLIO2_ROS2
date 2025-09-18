#include "utils.h"
pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::velo2PCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZI> cloud_xyz;
    pcl::fromROSMsg(*msg, cloud_xyz);
    // 필터링
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);

    filtered->reserve(cloud_xyz.size() / filter_num + 1);
    for (size_t i = 0; i < cloud_xyz.size(); i += filter_num)
    {
        const auto &pt = cloud_xyz.points[i];

        if (!pcl::isFinite(pt))
            continue;

        float dist_sq = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (dist_sq < min_range * min_range || dist_sq > max_range * max_range)
            continue;
        filtered->push_back(pt);
    }

    // 법선 추정
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    ne.setInputCloud(filtered);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    ne.setSearchMethod(tree);
    ne.setKSearch(10); // 또는 setRadiusSearch(0.2) 등으로 조정 가능

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    // XYZINormal로 병합
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZINormal>);
    output->resize(filtered->size());
    for (size_t i = 0; i < filtered->size(); ++i)
    {
        pcl::PointXYZINormal &p = output->points[i];
        const auto &pt = filtered->points[i];
        const auto &n = normals->points[i];

        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = pt.intensity;
        p.normal_x = n.normal_x;

        p.normal_y = n.normal_y;
        p.normal_z = n.normal_z;

        if (p.normal_z > 0)
        {
            p.normal_x *= -1;
            p.normal_y *= -1;
            p.normal_z *= -1;
        }

        //std::cout << "curvature: " << n.curvature << std::endl;

        p.curvature = 0.0;
    }
    std::cout << "Output point cloud size: " << output->size() << std::endl;

    return output;
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
