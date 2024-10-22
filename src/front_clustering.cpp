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
    LidarClustering() : next_id_(0)
    {
        scan_sub_ = nh_.subscribe("/scan", 10, &LidarClustering::scanCallback, this);
        cluster_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("lidar_clusters", 10);
        people_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("detected_people", 10);
    }


    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        // クラスター用のベクトルを初期化
        std::vector<std::vector<geometry_msgs::Point>> clusters;
        std::vector<geometry_msgs::Point> current_cluster;

        // 各スキャンごとに初期化すべき変数を宣言
        double distance_threshold = 0.2;
        double cluster_sum_x = 0.0;
        double cluster_sum_y = 0.0;
        double cluster_distance_min = std::numeric_limits<double>::max();
        int cluster_distance_min_number = -1; // 最も近いクラスターの番号
        double point_y_min = std::numeric_limits<double>::max();
        double point_y_max = std::numeric_limits<double>::lowest();
        int cluster_flag = 0;

        // スキャンデータの各レンジについてクラスタリング処理
        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            if (std::isfinite(scan->ranges[i]))
            {
                geometry_msgs::Point p;
                double angle = scan->angle_min + i * scan->angle_increment;
                p.x = scan->ranges[i] * cos(angle);
                p.y = scan->ranges[i] * sin(angle);

                // 現在のクラスタに点を追加
                if (current_cluster.empty() || distance(current_cluster.back(), p) < distance_threshold)
                {
                    current_cluster.push_back(p);
                }
                else
                {
                    if (!current_cluster.empty())
                    {
                        clusters.push_back(current_cluster); // 完了したクラスタを保存
                        current_cluster.clear(); // 新しいクラスタのためにクリア
                    }
                    current_cluster.push_back(p);
                }
            }
        }

        // 残ったクラスタを追加
        if (!current_cluster.empty())
        {
            clusters.push_back(current_cluster);
        }

        // クラスタごとの処理
        for (size_t i = 0; i < clusters.size(); ++i)
        {
            float distance = scan->ranges[i];

            cluster_sum_x = 0.0; // 各クラスタごとに初期化
            cluster_sum_y = 0.0;
            cluster_flag = 0;

            //各cluster内の点に関して、短冊内にあるかどうか
            for (const auto& point : clusters[i])
            {
                double angle_rad = std::atan2(point.y, point.x); // ラジアン
                double angle_deg = angle_rad * (180.0 / M_PI);   // 度に変換

                if (-10 < angle_deg && angle_deg < 10) 
                {
                    // float distance = scan->ranges[i];
                    double distance_judge = distance * std::cos(angle_rad);

                    if (-1.5 < distance_judge && distance_judge < 1.5) 
                    {
                        // cluster_sum_x += point.x;
                        // cluster_sum_y += point.y;
                        cluster_flag = 1;
                    }
                }
                cluster_sum_x += point.x;
                cluster_sum_y += point.y;
            }

            if (cluster_flag == 1) // フラグが立っている場合
            {
                double cluster_ave_x = cluster_sum_x / clusters[i].size();
                double cluster_ave_y = cluster_sum_y / clusters[i].size();
                double cluster_distance = sqrt(pow(cluster_ave_x, 2) + pow(cluster_ave_y, 2));

                if (cluster_distance_min > cluster_distance) 
                {
                    cluster_distance_min = cluster_distance;
                    cluster_distance_min_number = i;
                }
                cluster_flag = 0;
            }
        }

        // 最も近いクラスターについて、最小・最大のy座標を計算
        if (cluster_distance_min_number != -1)
        {
            for (const auto& point : clusters[cluster_distance_min_number])
            {
                if (point.y < point_y_min)
                {
                    point_y_min = point.y;
                }
                if (point.y > point_y_max)
                {
                    point_y_max = point.y;
                }
            }

            std::cout << "point_y_min: " << point_y_min << " point_y_max: " << point_y_max << std::endl;
        }
    }


    // void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    // {
    //     std::vector<std::vector<geometry_msgs::Point>> clusters;
    //     std::vector<geometry_msgs::Point> current_cluster;

    //     double distance_threshold = 0.2;

    //     for (size_t i = 0; i < scan->ranges.size(); ++i)
    //     {
    //         if (std::isfinite(scan->ranges[i]))
    //         {
    //             geometry_msgs::Point p;
    //             double angle = scan->angle_min + i * scan->angle_increment;
    //             p.x = scan->ranges[i] * cos(angle);
    //             p.y = scan->ranges[i] * sin(angle);

    //             if (current_cluster.empty() || distance(current_cluster.back(), p) < distance_threshold)
    //             {
    //                 current_cluster.push_back(p);
    //             }
    //             else
    //             {
    //                 if (!current_cluster.empty())
    //                 {
    //                     clusters.push_back(current_cluster);
    //                     current_cluster.clear();
    //                 }
    //                 current_cluster.push_back(p);
    //             }
    //         }

    //     }

    //     // if (!current_cluster.empty())
    //     // {
    //     //     clusters.push_back(current_cluster);
    //     // }

    //     for (size_t i = 0; i < clusters.size(); ++i)
    //     {
    //         for (const auto& point : clusters[i])
    //         {
    //             double angle_rad = std::atan2(point.y, point.x); // ラジアン
    //             double angle_deg = angle_rad * (180.0 / M_PI); 
    
    //             if (-10 < angle_deg && angle_deg < 10) {
    //                 float distance = scan->ranges[i];
    //                 double distance_judge = distance * std::cos(angle_rad);

    //                 // double distance_judge = this->distance(point1, point2) * std::cos(angle_rad);
    //                 if (-1.5 < distance_judge && distance_judge < 1.5) {

    //                 cluster_sum_x += point.x;
    //                 cluster_sum_y += point.y;
    //                 cluster_flag = 1; 
    //                 }
    //             } 
    //             double cluster_ave_x = 0.0;
    //             double cluster_ave_y = 0.0;

    //             cluster_ave_x = cluster_sum_x / clusters[i].size();
    //             cluster_ave_y = cluster_sum_y / clusters[i].size();

    //             double cluster_distance = sqrt(pow(cluster_ave_x, 2) + pow(cluster_ave_y, 2));

    //             if(cluster_distance_min > cluster_distance){
    //                 cluster_distance_min = cluster_distance;
    //                 cluster_distance_min_number = i;
    //             }
    //         }

    //         if(cluster_flag = 1){
    //             for (const auto& point : clusters[i])
    //             {
    //                 cluster_sum_x += point.x;
    //                 cluster_sum_y += point.y;
    //             }
    //             double cluster_ave_x = cluster_sum_x / clusters[i].size();
    //             double cluster_ave_y = cluster_sum_y / clusters[i].size();
    //             double cluster_distance = sqrt(pow(cluster_ave_x, 2) + pow(cluster_ave_y, 2));

    //             if(cluster_distance_min > cluster_distance){
    //                 cluster_distance_min = cluster_distance;
    //                 cluster_distance_min_number = i;
    //             }
    //             cluster_flag = 0;

    //         }

    //     }


    //     for (const auto& point : clusters[cluster_distance_min_number])
    //     {
    //         if (point_y_min > point.y){
    //             point_y_min = point.y;

    //         }
    //         if (point_y_max < point.y) {
    //             point_y_max = point.y;
    //         }
            
    //     }
    //     std::cout << "point_y_min" << point_y_min << "point_y_max" << point_y_max << std::endl;





    //     // for (size_t i = 0; i < scan->ranges.size(); ++i)
    //     // {
    //     //     // 前方の障害物検出
    //     //     if (-10 < angle_deg && angle_deg < 10) {
    //     //         double distance_judge = distance * std::cos(angle_rad);
    //     //         if (-1.5 < distance_judge && distance_judge < 1.5) {
    //     //             nearCluster(clusters);

    //     //         }
    //     //     } 
    //     //     // 側面の障害物検出
    //     //     else if ((angle_deg >= -90 && angle_deg <= -10) || (angle_deg >= 10 && angle_deg <= 90)) {
    //     //         double distance_judge = distance * std::sin(angle_rad);
    //     //         if (-0.5 < distance_judge && distance_judge < 0.5) {
    //     //             nearCluster(clusters);

    //     //         }
    //     //     }
    //     // }





    //     publishClusters(clusters);
    // }

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher cluster_pub_;
    ros::Publisher people_pub_;

    std::vector<Person> tracked_people_; // 追跡中の人々のリスト
    std::vector<int> deleted_ids; //削除される人のID
    int next_id_; // 次に使用するID
    double cluster_distance_min = 100000;
    int cluster_distance_min_number;
    int cluster_flag = 0;
    double cluster_sum_x = 0.0;
    double cluster_sum_y = 0.0;
    double point_y_min;
    double point_y_max;
    

    // void nearCluster(const std::vector<std::vector<geometry_msgs::Point>>& clusters){
    //     for (size_t i = 0; i < clusters.size(); ++i)
    //     {
    //         for (const auto& point : clusters[i])
    //         {
    //             double cluster_sum_x += point.x;
    //             double cluster_sum_y += point.y;
    //         }
    //         double cluster_ave_x = cluster_sum_x / clusters[i].size();
    //         double cluster_ave_y = cluster_sum_y / clusters[i].size();
    //         double cluster_distance = sqrt(pow(cluster_ave_x, 2), pow(cluster_ave_y, 2));

    //         if(cluster_distance_min > cluster_distance){
    //             cluster_distance_min = cluster_distance;
    //             cluster_distance_min_number = i;
    //         }
    //     }
    //     return cluster_distance_min_number;
    // }

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



};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_clustering_node");
    LidarClustering lc;
    ros::spin();
    return 0;
}
