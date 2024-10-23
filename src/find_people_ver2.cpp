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

        // publishClusters(clusters);
        detectPeople(clusters);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher cluster_pub_;
    ros::Publisher people_pub_;

    std::vector<Person> tracked_people_;
    std::vector<Person> detected_people_;
    std::vector<Person> lost_people_;

    int next_id_;

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

        updateTrackedPeople(detected_people);

        // for (size_t i = 0; i < tracked_people_.size(); ++i)
        // {
        //     publishPersonMarker(people_markers, tracked_people_[i], tracked_people_[i].id);
        // }

        people_pub_.publish(people_markers);
    }

    void updateTrackedPeople(std::vector<Person>& detected_people_)
    {
        for(auto& detected_person : detected_people_)
        {
            double min_distance = std::numeric_limits<double>::max();
            int best_match_index = -1;

            for(size_t i = 0; i < tracked_people_.size(); ++i)
            {
                geometry_msgs::Point tracked_people_xy = calculateAveragePoint(tracked_people_[i]);
                geometry_msgs::Point detected_person_xy = calculateAveragePoint(detected_person);
                double dist = distance(tracked_people_xy, detected_person_xy);

                if (dist < min_distance)
                {
                    min_distance = dist;
                    best_match_index = i;
                }
            }

            if (best_match_index != -1)
            {
                tracked_people_[best_match_index] = detected_person;  // 更新
            }
            else
            {
                detected_person.id = next_id_++;
                tracked_people_.push_back(detected_person);  // 新規追加
            }
        }
    }

    geometry_msgs::Point calculateAveragePoint(const Person& person)
    {
        geometry_msgs::Point average_point;
        average_point.x = 0.0;
        average_point.y = 0.0;
        average_point.z = 0.0;

        for (const auto& point : person.points)
        {
            average_point.x += point.x;
            average_point.y += point.y;
        }

        if (!person.points.empty())
        {
            average_point.x /= person.points.size();
            average_point.y /= person.points.size();
        }

        return average_point;
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
        double avg_distance = person.average_distance;
        int point_count = person.points.size();
        double length = calculateLength(person.min_point, person.max_point);
        double aspect_ratio = calculateAspectRatio(length, person.min_point, person.max_point);

        return (   (avg_distance <= 5.0 && point_count >= 15 && point_count <= 70 && length >= 0.25 && length <= 1.5)
                && (aspect_ratio >= 0.5 && aspect_ratio <= 3.0));
    }

    double calculateAverageDistance(const std::vector<geometry_msgs::Point>& cluster)
    {
        double total_distance = 0.0;
        for (const auto& point : cluster)
        {
            total_distance += sqrt(point.x * point.x + point.y * point.y);
        }
        return total_distance / cluster.size();
    }

    std::pair<geometry_msgs::Point, geometry_msgs::Point> calculateBoundingBox(const std::vector<geometry_msgs::Point>& cluster)
    {
        geometry_msgs::Point min_point;
        geometry_msgs::Point max_point;
        min_point.x = min_point.y = std::numeric_limits<double>::max();
        max_point.x = max_point.y = std::numeric_limits<double>::lowest();

        for (const auto& point : cluster)
        {
            if (point.x < min_point.x) min_point.x = point.x;
            if (point.y < min_point.y) min_point.y = point.y;
            if (point.x > max_point.x) max_point.x = point.x;
            if (point.y > max_point.y) max_point.y = point.y;
        }

        return {min_point, max_point};
    }

    void publishPersonMarker(visualization_msgs::MarkerArray& markers, const Person& person, int id)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "laser";
        marker.header.stamp = ros::Time::now();
        marker.ns = "people";
        marker.id = id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = (person.min_point.x + person.max_point.x) / 2.0;
        marker.pose.position.y = (person.min_point.y + person.max_point.y) / 2.0;
        marker.pose.position.z = 0.0; // 高さを 0 に設定
        marker.scale.x = person.length;
        marker.scale.y = person.length * person.aspect_ratio; // アスペクト比を使用
        marker.scale.z = 0.2; // 高さを 0.2 に設定
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 0.5;

        markers.markers.push_back(marker);

        // ID を表示するためのテキストマーカを追加
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "laser";
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "people_text";
        text_marker.id = id + 1000;  // 一意なIDを使うためのオフセット
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = marker.pose.position.x;
        text_marker.pose.position.y = marker.pose.position.y;
        text_marker.pose.position.z = marker.pose.position.z + 0.2; // テキストを上に表示
        text_marker.scale.z = 0.2;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.text = "ID: " + std::to_string(person.id);

        markers.markers.push_back(text_marker);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_clustering");
    LidarClustering lc;
    ros::spin();
    return 0;
}
