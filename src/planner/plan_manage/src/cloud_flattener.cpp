#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

/**
 * @brief 虚拟高墙生成器 (Virtual Wall Generator)
 * 解决 Ego-Planner 试图从扁平障碍物上方“飞越”的问题。
 * 原理：将每一个检测到的障碍点，在 Z 轴上拉伸成一根柱子。
 */

class CloudFlattener {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_cloud_;
    ros::Publisher pub_cloud_;

    double min_height_;
    double max_height_;
    double wall_height_; // 新增：人造墙的高度
    double resolution_;  // 新增：墙的垂直分辨率
    std::string input_topic_;
    std::string output_topic_;

public:
    CloudFlattener(ros::NodeHandle& nh) : nh_(nh) {
        nh_.param("cloud_flattener/min_height", min_height_, 0.2); // 滤除地面
        nh_.param("cloud_flattener/max_height", max_height_, 3.0); // 滤除天花板
        
        // [核心参数] 
        // 我们要把 Z=0 的点拉伸到多高？建议 2.0米，足够挡住规划器
        nh_.param("cloud_flattener/wall_height", wall_height_, 2.0);
        // 在 Z 轴上每隔多少米插一个点？建议 0.1米，越密越安全
        nh_.param("cloud_flattener/resolution", resolution_, 0.1); 

        nh_.param<std::string>("cloud_flattener/input_topic", input_topic_, "/livox/lidar");
        nh_.param<std::string>("cloud_flattener/output_topic", output_topic_, "/planning/flat_cloud");

        sub_cloud_ = nh_.subscribe(input_topic_, 1, &CloudFlattener::cloudCallback, this);
        pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
        
        ROS_INFO("[CloudFlattener] Virtual Wall Mode Activated!");
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud_in);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
        
        // 预估点数会膨胀 (wall_height / resolution) 倍
        int expansion_ratio = std::ceil(wall_height_ / resolution_);
        cloud_out->points.reserve(cloud_in->points.size() * expansion_ratio);

        for (const auto& pt : cloud_in->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

            // 1. 高度过滤：只处理有效范围内的障碍物
            if (pt.z >= min_height_ && pt.z <= max_height_) {
                
                // 2. [核心修改] 制造虚拟高墙
                // 在 (x, y) 处，从 Z=0 到 Z=wall_height 垂直生成一串点
                for (double h = 0.0; h <= wall_height_; h += resolution_) {
                    pcl::PointXYZ pt_wall;
                    pt_wall.x = pt.x;
                    pt_wall.y = pt.y;
                    pt_wall.z = h; // 人工设置高度
                    cloud_out->points.push_back(pt_wall);
                }
            }
        }

        cloud_out->width = cloud_out->points.size();
        cloud_out->height = 1;
        cloud_out->is_dense = true;
        cloud_out->header = cloud_in->header;

        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_out, output_msg);
        pub_cloud_.publish(output_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cloud_flattener_node");
    ros::NodeHandle nh("~");
    CloudFlattener flattener(nh);
    ros::spin();
    return 0;
}
