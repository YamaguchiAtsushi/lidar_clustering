#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

struct Person
{
    std::vector<geometry_msgs::Point> points;  // 検出された点の集まり
    double average_distance;                     // 平均距離
    double length;                               // バウンディングボックスの長さ
    double aspect_ratio;                         // アスペクト比
    geometry_msgs::Point min_point;             // バウンディングボックスの最小点
    geometry_msgs::Point max_point;             // バウンディングボックスの最大点
    int id;
};

class LidarClustering
{
public:
    LidarClustering() : next_id_(0)
    {
        scan_sub_ = nh_.subscribe("/scan", 10, &LidarClustering::scanCallback, this);
        cluster_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("lidar_clusters", 10);
        people_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("detected_people", 10);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        std::vector<std::vector<geometry_msgs::Point>> clusters;
        std::vector<geometry_msgs::Point> current_cluster;

        double distance_threshold = 0.1;
        double min_distancatkin_ws/src/lidar_clustering/bag/test3e = 100; // この値は適切な閾値に設定してください
        int min_distance_num = -1;

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float angle_rad = scan->angle_min + i * scan->angle_increment;
            float angle_deg = angle_rad * (180.0 / M_PI); // ラジアンを度数法に変換
            float distance = scan->ranges[i];

            // 無効なデータをスキップ
            if (std::isnan(distance) || distance > scan->range_max) {
                continue;
            }

            geometry_msgs::Point p;
            p.x = distance * std::cos(angle_rad); // X座標
            p.y = distance * std::sin(angle_rad); // Y座標
            p.z = 0; // Z座標は0に設定

            // 前方の障害物検出
            if (-10 < angle_deg && angle_deg < 10) {
                double distance_judge = distance * std::cos(angle_rad);
                if (-1.5 < distance_judge && distance_judge < 1.5) {
                    // std::cout << "Obstacle found: back, distance_judge = " << distance_judge << std::endl;

                    if (sqrt(pow(p.x, 2) + pow(p.y, 2)) < min_distance) {
                        min_distance_num = i;
                    }
                    current_cluster.push_back(p); // 現在のクラスタに追加
                }
            } 
            // 側面の障害物検出
            else if ((angle_deg >= -90 && angle_deg <= -10) || (angle_deg >= 10 && angle_deg <= 90)) {
                double distance_judge = distance * std::sin(angle_rad);
                if (-0.5 < distance_judge && distance_judge < 0.5) {
                    // std::cout << "Obstacle found: side, distance_judge = " << distance_judge << std::endl;

                    if (sqrt(pow(p.x, 2) + pow(p.y, 2)) < min_distance) {
                        min_distance_num = i;
                    }
                    current_cluster.push_back(p); // 現在のクラスタに追加
                }
            }
        }

        // クラスタの右端と左端の座標を取得
        if (!current_cluster.empty()) {
            geometry_msgs::Point leftmost_point = current_cluster[0];
            geometry_msgs::Point rightmost_point = current_cluster[0];

            for (const auto& point : current_cluster) {
                if (point.x < leftmost_point.x) {
                    leftmost_point = point; // 左端のポイントを更新
                }
                if (point.x > rightmost_point.x) {
                    rightmost_point = point; // 右端のポイントを更新
                }
            }

            std::cout << "Leftmost Point: (" << leftmost_point.x << ", " << leftmost_point.y << ")" << std::endl;
            std::cout << "Rightmost Point: (" << rightmost_point.x << ", " << rightmost_point.y << ")" << std::endl;
        }
        publishClusters(clusters);
    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher cluster_pub_;
    ros::Publisher people_pub_;

    std::vector<Person> tracked_people_;
    std::vector<Person> tracked_people_before_;
    std::vector<Person> lost_people;

    int next_id_;

    double distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
    {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }

    double angle(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
    {
        return atan2(abs(p1.y - p2.y),abs(p1.x - p2.y));
    }

    void publishClusters(const std::vector<std::vector<geometry_msgs::Point>>& clusters)
    {
        visualization_msgs::MarkerArray marker_array;
        for (size_t i = 0; i < clusters.size(); ++i)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "laser";
            marker.header.stamp = ros::Time::now();
            marker.ns = "clusters";
            marker.id = i;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.05;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            for (const auto& point : clusters[i])
            {
                marker.points.push_back(point);
            }

            marker_array.markers.push_back(marker);
            addTextMarker(marker_array, i, marker.points[0]);
        }

        cluster_pub_.publish(marker_array);
    }

    void addTextMarker(visualization_msgs::MarkerArray& marker_array, int id, const geometry_msgs::Point& position)
    {
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "laser";
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "cluster_text";
        text_marker.id = id + 1000;  // 一意なIDを使うため、オフセットを追加
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position = position;
        text_marker.pose.position.z += 0.2;
        text_marker.scale.z = 0.2;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.text = "ID: " + std::to_string(id);

        marker_array.markers.push_back(text_marker);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_clustering_node");
    LidarClustering lc;
    ros::spin();
    return 0;
}