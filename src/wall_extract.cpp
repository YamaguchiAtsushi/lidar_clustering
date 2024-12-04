#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

class WallPositionVisualizer {
public:
    WallPositionVisualizer() {
        // サブスクライバーとパブリッシャーの初期化
        map_sub_ = nh_.subscribe("/map", 1, &WallPositionVisualizer::mapCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("wall_markers", 1);

        // 1秒ごとにマーカーを再パブリッシュするタイマーの設定
        timer_ = nh_.createTimer(ros::Duration(1.0), &WallPositionVisualizer::timerCallback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Publisher marker_pub_;
    ros::Timer timer_;
    visualization_msgs::Marker wall_marker_;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
        int width = map_msg->info.width;
        int height = map_msg->info.height;
        double resolution = map_msg->info.resolution;
        double origin_x = map_msg->info.origin.position.x;
        double origin_y = map_msg->info.origin.position.y;

        // マーカーの設定
        wall_marker_.header.frame_id = "map";
        wall_marker_.header.stamp = ros::Time::now();
        wall_marker_.ns = "wall_cells";
        wall_marker_.id = 0;
        wall_marker_.type = visualization_msgs::Marker::CUBE_LIST;
        wall_marker_.action = visualization_msgs::Marker::ADD;
        wall_marker_.scale.x = resolution;
        wall_marker_.scale.y = resolution;
        wall_marker_.scale.z = 0.1;  // 高さを0.1mに設定
        wall_marker_.color.r = 1.0;  // 赤色
        wall_marker_.color.g = 0.0;
        wall_marker_.color.b = 0.0;
        wall_marker_.color.a = 1.0;
        wall_marker_.points.clear();

        // 壁セルを探索して位置をマーカーに追加
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int index = x + y * width;
                int cell_value = map_msg->data[index];

                // 壁のセルの値が100と仮定
                if (cell_value == 100) {
                    geometry_msgs::Point wall_point;
                    wall_point.x = origin_x + x * resolution;
                    wall_point.y = origin_y + y * resolution;
                    wall_point.z = 0.0;  // 地面上に配置
                    wall_marker_.points.push_back(wall_point);
                }
            }
        }
    }

    void timerCallback(const ros::TimerEvent&) {
        // mapCallbackが実行された後、定期的にマーカーをパブリッシュ
        if (!wall_marker_.points.empty()) {
            marker_pub_.publish(wall_marker_);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_position_visualizer");
    WallPositionVisualizer wall_position_visualizer;
    ros::spin();
    return 0;
}
