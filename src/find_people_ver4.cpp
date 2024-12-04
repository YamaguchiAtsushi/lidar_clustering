#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void transformPointToMap(const geometry_msgs::Point& laser_point, geometry_msgs::Point& map_point)
{
    // TransformListenerの作成
    static tf2_ros::Buffer tf_buffer;
    static tf2_ros::TransformListener tf_listener(tf_buffer);

    // laser座標系からmap座標系への変換を取得
    try
    {
        // "laser"座標系から"map"座標系に変換
        geometry_msgs::PointStamped laser_point_stamped;
        laser_point_stamped.point = laser_point;
        laser_point_stamped.header.frame_id = "laser"; // 変換元の座標系

        geometry_msgs::PointStamped map_point_stamped;
        map_point_stamped = tf_buffer.transform(laser_point_stamped, "map", ros::Duration(1.0));

        // 変換後のmap座標系の点を返す
        map_point = map_point_stamped.point;
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("Transform failed: %s", ex.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "coordinate_transformer");
    ros::NodeHandle nh;

    // 例: laser座標系での点
    geometry_msgs::Point laser_point;
    laser_point.x = 1.0;
    laser_point.y = 2.0;
    laser_point.z = 0.0;

    // map座標系に変換した後の点
    geometry_msgs::Point map_point;

    // 座標変換
    transformPointToMap(laser_point, map_point);

    ROS_INFO("Laser point: x=%.2f, y=%.2f, z=%.2f", laser_point.x, laser_point.y, laser_point.z);
    ROS_INFO("Map point: x=%.2f, y=%.2f, z=%.2f", map_point.x, map_point.y, map_point.z);

    return 0;
}
