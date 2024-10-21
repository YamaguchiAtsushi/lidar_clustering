#ifndef LIDAR_CLUSTERING_H
#define LIDAR_CLUSTERING_H

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
    double average_distance;                   // 平均距離
    double length;                             // バウンディングボックスの長さ
    double aspect_ratio;                       // アスペクト比
    double timeout_counter;
    geometry_msgs::Point min_point;            // バウンディングボックスの最小点
    geometry_msgs::Point max_point;            // バウンディングボックスの最大点
    int id;                                    // 人を一意に識別するID
    bool is_lost;
    bool is_tracked;                           // この人が追跡されているかどうかのフラグ
};

class LidarClustering
{
public:
    LidarClustering();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void publishClusters(const std::vector<std::vector<geometry_msgs::Point>>& clusters);
    double distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    void publishClusters(const std::vector<std::vector<geometry_msgs::Point>>& clusters);
    void addTextMarker(visualization_msgs::MarkerArray& marker_array, int id, const geometry_msgs::Point& position);
    void detectPeople(const std::vector<std::vector<geometry_msgs::Point>>& clusters);
    double calculateLength(const geometry_msgs::Point& min_point, const geometry_msgs::Point& max_point);
    double calculateAspectRatio(double length, const geometry_msgs::Point& min_point, const geometry_msgs::Point& max_point);
    bool isValidPerson(const Person& person);
    void publishPersonMarker(visualization_msgs::MarkerArray& people_markers, const Person& person, int marker_id);
    std::pair<geometry_msgs::Point, geometry_msgs::Point> calculateBoundingBox(const std::vector<geometry_msgs::Point>& cluster);
    double calculateAverageDistance(const std::vector<geometry_msgs::Point>& cluster);

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher cluster_pub_;
    ros::Publisher people_pub_;

    std::vector<Person> detected_people;
    std::vector<Person> tracked_people_; // 追跡中の人々のリスト
    std::vector<int> deleted_ids; //削除される人のID
    std::vector<std::vector<geometry_msgs::Point>> clusters;
    std::vector<geometry_msgs::Point> current_cluster;
    
    geometry_msgs::Point p;
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray people_markers;
    int marker_id_; // 次に使用するID
    double distance_threshold;
};

#endif // LIDAR_CLUSTERING_H
