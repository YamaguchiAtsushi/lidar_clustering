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
    geometry_msgs::Point min_point;            // バウンディングボックスの最小点
    geometry_msgs::Point max_point;            // バウンディングボックスの最大点
    int id;                                    // 人を一意に識別するID
    bool is_tracked;                           // この人が追跡されているかどうかのフラグ
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

        double distance_threshold = 0.2;

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
        detectAndTrackPeople(clusters);  // 追跡のために関数名を変更
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher cluster_pub_;
    ros::Publisher people_pub_;

    std::vector<Person> tracked_people_; // 追跡中の人々のリスト
    std::vector<int> deleted_ids; //削除される人のID
    int next_id_; // 次に使用するID

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
            //addTextMarker(marker_array, i, marker.points[0]);
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


void detectAndTrackPeople(const std::vector<std::vector<geometry_msgs::Point>>& clusters)
{
    visualization_msgs::MarkerArray people_markers;

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
        person.is_tracked = false;

        // フィルタリング条件を満たすかチェック
        if (isValidPerson(person))
        {
            detected_people.push_back(person);
        }
    }

    // 既存のトラッキング情報を更新
    updateTrackedPeople(detected_people);

    // 追跡結果を可視化
    for (size_t i = 0; i < tracked_people_.size(); ++i)
    {
        //tracked_people_[i].id = i + 1;  // IDを1から始まるように設定←これを解除するとid=1が生成される
        publishPersonMarker(people_markers, tracked_people_[i], tracked_people_[i].id);
    }

    people_pub_.publish(people_markers);
}

void updateTrackedPeople(std::vector<Person>& detected_people)
{
    double tracking_threshold = 0.5;  // 人を追跡するための距離閾値

    // 既存のトラッキング情報を更新
    for (auto& tracked_person : tracked_people_)
    {
        tracked_person.is_tracked = false;
    }

    // 新しく検出された人物と、既存の追跡情報をマッチング
    for (auto& detected_person : detected_people)
    {
        double min_distance = std::numeric_limits<double>::max();
        int best_match_index = -1;

        // 過去に削除された人物も含めて再トラッキングを試みる
        for (size_t i = 0; i < tracked_people_.size(); ++i)
        {
            double dist = distance(tracked_people_[i].min_point, detected_person.min_point);
            if (dist < min_distance && dist < tracking_threshold)
            {
                min_distance = dist;
                best_match_index = i;
            }
        }

        // マッチする人が見つかった場合、その人を追跡対象として更新
        if (best_match_index != -1)
        {
            tracked_people_[best_match_index].points = detected_person.points;
            tracked_people_[best_match_index].min_point = detected_person.min_point;
            tracked_people_[best_match_index].max_point = detected_person.max_point;
            tracked_people_[best_match_index].length = detected_person.length;
            tracked_people_[best_match_index].aspect_ratio = detected_person.aspect_ratio;
            tracked_people_[best_match_index].average_distance = detected_person.average_distance;
            tracked_people_[best_match_index].is_tracked = true;
        }
        else
        {
            detected_person.id = next_id_++;

            
            // マッチしなかった場合は新しい人物として追加
            // if (!deleted_ids.empty())
            // {
            //     detected_person.id = deleted_ids.front();  // 削除されたIDを再利用
            //     deleted_ids.erase(deleted_ids.begin());   // 先頭要素を削除
            //     // detected_person.id = next_id_++;

            // }
            // else
            // {
            //     detected_person.id = next_id_++;
            // }

            detected_person.is_tracked = true;
            tracked_people_.push_back(detected_person);
        }
    }

    // 追跡されていない人物を削除し、IDを保存
    for (auto it = tracked_people_.begin(); it != tracked_people_.end(); )
    {
        if (!it->is_tracked)
        {
            deleted_ids.push_back(it->id);  // 削除する人物のIDを保存
            it = tracked_people_.erase(it);
        }
        else
        {
            ++it;
        }
    }


}


void publishPersonMarker(visualization_msgs::MarkerArray& markers, const Person& person, int marker_id)
{
    // 既存のマーカーを削除するためのマーカーを追加
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "laser";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "people";
    delete_marker.id = marker_id;  // 同じIDを使用
    delete_marker.type = visualization_msgs::Marker::CUBE;
    delete_marker.action = visualization_msgs::Marker::DELETE;

    markers.markers.push_back(delete_marker);

    // 新しい人物のための立方体マーカーを作成
    visualization_msgs::Marker marker;
    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "people";
    marker.id = marker_id;  // 同じIDを使用
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.025); 

    // バウンディングボックスの中心点を計算
    geometry_msgs::Point center;
    center.x = (person.min_point.x + person.max_point.x) / 2.0;
    center.y = (person.min_point.y + person.max_point.y) / 2.0;
    center.z = 0.0;  // Z軸は0に固定

    marker.pose.position = center;
    marker.scale.x = person.length;  // 幅
    marker.scale.y = 0.5;             // 奥行き
    marker.scale.z = 0.0;             // 高さ
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    markers.markers.push_back(marker);

    // IDを表示するためのテキストマーカーを追加
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = "laser";
    text_marker.header.stamp = ros::Time::now();
    text_marker.ns = "cluster_text";
    text_marker.id = marker_id + 1000;  // 一意なIDを使うため、オフセットを追加
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.position = center;
    text_marker.pose.position.z += 0.2;
    text_marker.scale.z = 0.4;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.lifetime = ros::Duration(0.025); 


    // detected_person.idを表示
    text_marker.text = "ID: " + std::to_string(person.id);

    markers.markers.push_back(text_marker);
}


    double calculateAverageDistance(const std::vector<geometry_msgs::Point>& points)
    {
        double sum = 0.0;
        for (const auto& point : points)
        {
            sum += sqrt(point.x * point.x + point.y * point.y);
        }
        return sum / points.size();
    }

    std::pair<geometry_msgs::Point, geometry_msgs::Point> calculateBoundingBox(const std::vector<geometry_msgs::Point>& points)
    {
        geometry_msgs::Point min_point, max_point;
        min_point.x = std::numeric_limits<double>::max();
        min_point.y = std::numeric_limits<double>::max();
        max_point.x = -std::numeric_limits<double>::max();
        max_point.y = -std::numeric_limits<double>::max();

        for (const auto& point : points)
        {
            min_point.x = std::min(min_point.x, point.x);
            min_point.y = std::min(min_point.y, point.y);
            max_point.x = std::max(max_point.x, point.x);
            max_point.y = std::max(max_point.y, point.y);
        }

        return {min_point, max_point};
    }

    double calculateLength(const geometry_msgs::Point& min_point, const geometry_msgs::Point& max_point)
    {
        return distance(min_point, max_point);
    }

    double calculateAspectRatio(double length, const geometry_msgs::Point& min_point, const geometry_msgs::Point& max_point)
    {
        double width = max_point.y - min_point.y;  // y方向の長さを幅とする
        return length / width;
    }

    bool isValidPerson(const Person& person)
    {
        //return person.length > 0.5 && person.aspect_ratio < 2.0;  // 例: 長さが0.5以上、アスペクト比が2未満であること

        int point_count = person.points.size();

        return (   (person.average_distance <= 5.0 && point_count >= 15 && point_count <= 70 && person.length >= 0.25 && person.length <= 0.7 && person.aspect_ratio >= 1.0 && person.aspect_ratio <= 3.0) 
                || (person.average_distance > 5.0 && person.average_distance <= 10.0 && point_count >= 10 && point_count <= 35 && person.length >= 0.25 && person.length <= 0.65 && person.aspect_ratio >= 1.0 && person.aspect_ratio <= 3.0)
                || (person.average_distance > 10.0 && point_count >= 5 && point_count <= 18 && person.length >= 0.25 && person.length <= 0.7 && person.aspect_ratio >= 1.0 && person.aspect_ratio <= 3.0)
            );
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_clustering_node");
    LidarClustering lc;
    ros::spin();
    return 0;
}
