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
};

class LidarClustering
{
public:
    LidarClustering()
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

        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            if (std::isfinite(scan->ranges[i]))
            {
                geometry_msgs::Point p;
                double angle = scan->angle_min + i * scan->angle_increment;
                p.x = scan->ranges[i] * cos(angle);
                p.y = scan->ranges[i] * sin(angle);

                if (current_cluster.empty() || distance(current_cluster.back(), p) < distance_threshold)
                {
                    current_cluster.push_back(p);
                }
                else
                {
                    if (!current_cluster.empty())
                    {
                        clusters.push_back(current_cluster);
                        current_cluster.clear();
                    }
                    current_cluster.push_back(p);
                }
            }
        }

        if (!current_cluster.empty())
        {
            clusters.push_back(current_cluster);
        }

        publishClusters(clusters);
        detectPeople(clusters);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher cluster_pub_;
    ros::Publisher people_pub_;

    double distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
    {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
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

    void detectPeople(const std::vector<std::vector<geometry_msgs::Point>>& clusters)
    {
        visualization_msgs::MarkerArray people_markers;
        int marker_id = 0;

        // 人の検出結果を管理するためのリスト
        std::vector<Person> detected_people;

        for (const auto& cluster : clusters)
        {
            
            Person person;
            person.points = cluster;
            person.average_distance = calculateAverageDistance(cluster);
            auto bounding_box = calculateBoundingBox(cluster);
            person.min_point = bounding_box.first;
            person.max_point = bounding_box.second;
            person.length = calculateLength(person.min_point, person.max_point);
            person.aspect_ratio = calculateAspectRatio(person.length, person.min_point, person.max_point);

            // フィルタリング条件を満たすかチェック
            if (isValidPerson(person))
            {
                detected_people.push_back(person);
                publishPersonMarker(people_markers, person, marker_id++);
            }
        }

        people_pub_.publish(people_markers);
    }

    double calculateLength(const geometry_msgs::Point& min_point, const geometry_msgs::Point& max_point)
    {
        return distance(min_point, max_point);
    }

    double calculateAspectRatio(double length, const geometry_msgs::Point& min_point, const geometry_msgs::Point& max_point)
    {
        double width_x = fabs(max_point.x - min_point.x);
        double width_y = fabs(max_point.y - min_point.y);
        return length / std::min(width_x, width_y);
    }

    bool isValidPerson(const Person& person)
    {
        // フィルタリング条件をここに記述
        double avg_distance = person.average_distance;
        int point_count = person.points.size();
        
        // min_point と max_point を使用して長さとアスペクト比を計算
        double length = calculateLength(person.min_point, person.max_point);
        double aspect_ratio = calculateAspectRatio(length, person.min_point, person.max_point);

        return (   (avg_distance <= 5.0 && point_count >= 15 && point_count <= 70 && length >= 0.25 && length <= 0.7 && aspect_ratio >= 1.0 && aspect_ratio <= 3.0) 
                || (avg_distance > 5.0 && avg_distance <= 10.0 && point_count >= 10 && point_count <= 35 && length >= 0.25 && length <= 0.65 && aspect_ratio >= 1.0 && aspect_ratio <= 3.0)
                || (avg_distance > 10.0 && point_count >= 5 && point_count <= 18 && length >= 0.25 && length <= 0.7 && aspect_ratio >= 1.0 && aspect_ratio <= 3.0)
            );
    }

    void publishPersonMarker(visualization_msgs::MarkerArray& people_markers, const Person& person, int marker_id)
    {
        visualization_msgs::Marker bbox_marker;
        bbox_marker.header.frame_id = "laser";
        bbox_marker.header.stamp = ros::Time::now();
        bbox_marker.ns = "detected_people";
        bbox_marker.id = marker_id;
        bbox_marker.type = visualization_msgs::Marker::LINE_LIST;
        bbox_marker.action = visualization_msgs::Marker::ADD;
        bbox_marker.scale.x = 0.05;
        bbox_marker.color.r = 1.0;
        bbox_marker.color.g = 0.0;
        bbox_marker.color.b = 0.0;
        bbox_marker.color.a = 1.0;

        geometry_msgs::Point p1;
        p1.x = person.min_point.x;
        p1.y = person.min_point.y;
        p1.z = 0.0;

        geometry_msgs::Point p2;
        p2.x = person.max_point.x;
        p2.y = person.min_point.y;
        p2.z = 0.0;

        geometry_msgs::Point p3;
        p3.x = person.max_point.x;
        p3.y = person.max_point.y;
        p3.z = 0.0;

        geometry_msgs::Point p4;
        p4.x = person.min_point.x;
        p4.y = person.max_point.y;
        p4.z = 0.0;

        bbox_marker.points.push_back(p1);
        bbox_marker.points.push_back(p2);
        bbox_marker.points.push_back(p2);
        bbox_marker.points.push_back(p3);
        bbox_marker.points.push_back(p3);
        bbox_marker.points.push_back(p4);
        bbox_marker.points.push_back(p4);
        bbox_marker.points.push_back(p1);

        people_markers.markers.push_back(bbox_marker);
    }

    std::pair<geometry_msgs::Point, geometry_msgs::Point> calculateBoundingBox(const std::vector<geometry_msgs::Point>& cluster)
    {
        geometry_msgs::Point min_point, max_point;
        min_point.x = std::numeric_limits<double>::max();
        min_point.y = std::numeric_limits<double>::max();
        max_point.x = std::numeric_limits<double>::lowest();
        max_point.y = std::numeric_limits<double>::lowest();

        for (const auto& point : cluster)
        {
            min_point.x = std::min(min_point.x, point.x);
            min_point.y = std::min(min_point.y, point.y);
            max_point.x = std::max(max_point.x, point.x);
            max_point.y = std::max(max_point.y, point.y);
        }

        return {min_point, max_point};
    }

    double calculateAverageDistance(const std::vector<geometry_msgs::Point>& cluster)
    {
        double total_distance = 0.0;

        for (const auto& point : cluster)
        {
            geometry_msgs::Point origin;
            origin.x = 0.0;
            origin.y = 0.0;
            origin.z = 0.0;
            total_distance += distance(origin, point); // 原点からの距離
        }

        return total_distance / cluster.size();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_clustering_node");
    LidarClustering lc;
    ros::spin();
    return 0;
}