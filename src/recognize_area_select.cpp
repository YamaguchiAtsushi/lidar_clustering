#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

class MaskedPointCloud {
public:
    MaskedPointCloud(ros::NodeHandle& nh) : nh_(nh) {
        scan_sub_ = nh_.subscribe("/scan", 1, &MaskedPointCloud::scanCallback, this);
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_pointcloud", 1);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/mask_visualization", 1);

        // マップとマスクの読み込み
        std::string map_path, mask_path;
        nh_.param<std::string>("map_path", map_path, "/home/yamaguchi-a/catkin_ws/src/lidar_clustering/map/sb_map.pgm");
        nh_.param<std::string>("mask_path", mask_path, "/home/yamaguchi-a/catkin_ws/src/lidar_clustering/map/sb_map_.pgm");

        map_image_ = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
        mask_image_ = cv::imread(mask_path, cv::IMREAD_GRAYSCALE);

        if (map_image_.empty() || mask_image_.empty()) {
            ROS_ERROR("マップまたはマスク画像の読み込みに失敗しました！");
            ros::shutdown();
        }

        // マスクとマップのサイズが一致するか確認
        if (map_image_.size() != mask_image_.size()) {
            ROS_ERROR("マップとマスクのサイズが一致しません！");
            ros::shutdown();
        }

        // マスクを可視化
        visualizeMask();
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        sensor_msgs::PointCloud2 cloud_msg;
        cloud_msg.header = scan_msg->header;
        cloud_msg.header.stamp = ros::Time::now();

        cloud_msg.height = 1;  // 1次元点群
        cloud_msg.is_dense = true;

        // センサーデータのサイズを計算
        size_t num_points = 0;
        std::vector<float> cloud_data;

        float angle_min = scan_msg->angle_min;
        float angle_increment = scan_msg->angle_increment;
        float range_max = scan_msg->range_max;
        float range_min = scan_msg->range_min;

        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            float range = scan_msg->ranges[i];

            if (range < range_min || range > range_max || std::isnan(range)) {
                continue;
            }

            float angle = angle_min + i * angle_increment;
            float x = range * cos(angle);
            float y = range * sin(angle);
            float z = 0.0;

            // 座標変換のデバッグ出力
            int map_x = static_cast<int>((x - origin_x_) / resolution_);
            int map_y = static_cast<int>((y - origin_y_) / resolution_);

            // ROS_INFO("x: %f, y: %f, map_x: %d, map_y: %d", x, y, map_x, map_y);

            if (map_x >= 0 && map_x < map_image_.cols && map_y >= 0 && map_y < map_image_.rows) {
                if (mask_image_.at<uchar>(map_y, map_x) == 254) {
                    ROS_INFO("Point added: x=%f, y=%f", x, y);
                    cloud_data.push_back(x);
                    cloud_data.push_back(y);
                    cloud_data.push_back(z);
                    num_points++;

                    std::cout << "mask_image_.at<uchar>(map_y, map_x)" << mask_image_.at<uchar>(map_y, map_x) << std::endl;
                    unsigned char mask_value = mask_image_.at<uchar>(map_y, map_x);
                    ROS_INFO("Mask value at map_x=%d, map_y=%d: %d", map_x, map_y, mask_value);

                    ROS_INFO("マスク画像の型: %d", mask_image_.type());
                } else {
                    // ROS_WARN("Mask condition not met at map_x=%d, map_y=%d", map_x, map_y);
                    // // std::cout << "mask_image_.at<uchar>(map_y, map_x)" << mask_image_.at<uchar>(map_y, map_x) << std::endl;
                    // ROS_INFO("マスク画像の型: %d", mask_image_.type());
                }
            }
        }

        // デバッグ: 点群のサイズを表示
        ROS_INFO("Cloud Data Size: %zu", cloud_data.size());


        // PointCloud2のデータ設定
        cloud_msg.width = num_points;
        cloud_msg.point_step = 3 * sizeof(float);  // x, y, z の3つの浮動小数点値
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
        cloud_msg.data.resize(cloud_msg.row_step); // 必要なサイズのデータ領域を確保

        // cloud_dataをPointCloud2のdataにコピー
        memcpy(cloud_msg.data.data(), cloud_data.data(), cloud_msg.data.size());

        // デバッグ出力: 点群数の確認
        ROS_INFO("Publishing %zu points", num_points);

        // 点群をパブリッシュ
        pointcloud_pub_.publish(cloud_msg);
    }

    void visualizeMask() {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "mask_visualization";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 0.05;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        cv::Mat mask_binary = mask_image_.clone(); // 白い部分をそのまま使用
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mask_binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 可視化用にマスクを表示
        cv::imshow("Mask Binary", mask_binary);
        cv::waitKey(0);

        // カウントされた輪郭をMarkerとして可視化
        for (const auto& contour : contours) {
            for (size_t i = 0; i < contour.size(); ++i) {
                cv::Point p1 = contour[i];
                cv::Point p2 = contour[(i + 1) % contour.size()];

                geometry_msgs::Point gp1, gp2;

                gp1.x = p1.x * resolution_ + origin_x_;
                gp1.y = p1.y * resolution_ + origin_y_;
                gp1.z = 0.0;

                gp2.x = p2.x * resolution_ + origin_x_;
                gp2.y = p2.y * resolution_ + origin_y_;
                gp2.z = 0.0;

                marker.points.push_back(gp1);
                marker.points.push_back(gp2);
            }
        }

        marker_pub_.publish(marker);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher pointcloud_pub_;
    ros::Publisher marker_pub_;

    cv::Mat map_image_;
    cv::Mat mask_image_;

    double origin_x_ = -56.800000;
    double origin_y_ = -16.800000;
    double resolution_ = 0.050000; // 例としての解像度
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "masked_pointcloud");
    ros::NodeHandle nh("~");

    MaskedPointCloud mpc(nh);

    ros::spin();
    return 0;
}
