#include <ros/ros.h>
// #include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "costmap");
// tf::TransformListener tf(ros::Duration(10));
  // tf2_ros::BufferとTransformListenerを作成
  tf2_ros::Buffer tfBuffer(ros::Duration(10)); // 10秒間のバッファ
  tf2_ros::TransformListener tfListener(tfBuffer);
  costmap_2d::Costmap2DROS costmap("my_costmap", tfBuffer);
  ros::NodeHandle n;
  costmap.start();
  ros::spin();
  return 0;
}