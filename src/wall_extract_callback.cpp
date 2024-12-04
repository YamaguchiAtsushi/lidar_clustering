#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

class WallPositionVisualizer {
public:
    WallPositionVisualizer() {
        // サブスクライバーとパブリッシャーの初期化
        map_sub_ = nh_.subscribe("/map", 1, &WallPositionVisualizer::mapCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("wall_markers", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Publisher marker_pub_;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
        int width = map_msg->info.width;
        int height = map_msg->info.height;
        double resolution = map_msg->info.resolution;
        double origin_x = map_msg->info.origin.position.x;
        double origin_y = map_msg->info.origin.position.y;

        // マーカーの設定
        visualization_msgs::Marker wall_marker;
        wall_marker.header.frame_id = "map";
        wall_marker.header.stamp = ros::Time::now();
        wall_marker.ns = "wall_cells";
        wall_marker.id = 0;
        wall_marker.type = visualization_msgs::Marker::CUBE;
        wall_marker.action = visualization_msgs::Marker::ADD;
        wall_marker.scale.x = resolution;
        wall_marker.scale.y = resolution;
        wall_marker.scale.z = 0.1;  // 高さを0.1mに設定
        wall_marker.color.r = 1.0;  // 赤色
        wall_marker.color.g = 0.0;
        wall_marker.color.b = 0.0;
        wall_marker.color.a = 1.0;

        // 壁セルを探索して位置をマーカーに追加
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int index = x + y * width;
                int cell_value = map_msg->data[index];

                // cell_value:壁のセルの値
                if (cell_value >= -0.5) {
                    geometry_msgs::Point wall_point;
                    wall_point.x = origin_x + x * resolution;
                    wall_point.y = origin_y + y * resolution;
                    wall_point.z = 0.0;  // 地面上に配置
                    // ROS_INFO("Wall at (%.3f, %.3f)", wall_point.x, wall_point.y);
                    // std::cout << wall_point.x << wall_point.y << std::endl;
                    wall_marker.points.push_back(wall_point);
                }
            }

        }

        // マーカーをパブリッシュ
        marker_pub_.publish(wall_marker);
        wall_marker.points.clear();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_position_visualizer");
    WallPositionVisualizer wall_position_visualizer;
    ros::spin();
    return 0;
}