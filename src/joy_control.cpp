#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

double max_vel = 1.0; //0.5 m/s 
// double max_omega = 0.2;
// double max_omega = 0.525; //rad/s → 30 deg/s
double max_omega = 1.0472; //rad/s → 60 deg/s 


float x_joy = 0.0;
float w_joy = 0.0;
float stop_joy[3] = {0.0};




sensor_msgs::Joy joy_data;
geometry_msgs::Twist cmd_vel;

void joy_callback(const sensor_msgs::Joy &joy_msg)
{
    x_joy = joy_msg.axes[1];
    w_joy = joy_msg.axes[2];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    // シミュレーション
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 10);
    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joy_callback);

    // float v_max = 0;                        //初期化
    // float w_max = 0;                        //初期化
    // float v_acc_param = 0;                  //初期化
    // float w_acc_param = 0;                  //初期化
    // float v_dec_free = 0;                   //初期化
    // float v_dec_param = 0;                  //初期化
    // float w_dec_param = 0;                  //初期化
    // float dec_time = 0;                     //初期化
    // pnh.getParam("v_max", v_max);           //パラメータ取得
    // pnh.getParam("w_max", w_max);           //パラメータ取得
    // pnh.getParam("v_acc_max", v_acc_param); //パラメータ取得
    // pnh.getParam("w_acc_max", w_acc_param); //パラメータ取得
    // pnh.getParam("v_dec_free", v_dec_free); //パラメータ取得
    // pnh.getParam("v_dec_max", v_dec_param); //パラメータ取得
    // pnh.getParam("w_dec_max", w_dec_param); //パラメータ取得
    // pnh.getParam("dec_time", dec_time);     //パラメータ取得
    // float pre_vel = 0.0;
    // float pre_acc = 0.0;
    // int FLAG = 0;
    // float button_v = 0;
    // float button_w = 0;

    ros::Rate rate(10.0);
    while (ros::ok())
    {
        ros::spinOnce();
        printf("aaa\n");
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        
        
        if (-1 <= x_joy && x_joy <= 1)
        {
            cmd_vel.linear.x = x_joy*max_vel;
        }
    
        else 
        {
            cmd_vel.linear.x = 0.0;
        }

        if (w_joy == 1)
        {
            cmd_vel.angular.z = max_omega;
        }
        else if (w_joy == -1)
        {
            cmd_vel.angular.z = -max_omega;
        }
        else 
        {
            cmd_vel.angular.z = 0.0;
        }

        printf("x_joy:%f,  input_vel:%f\n", x_joy, cmd_vel.linear.x);
        printf("w_joy:%f,  input_omega:%f\n", w_joy, cmd_vel.angular.z);

        cmd_pub.publish(cmd_vel);
        rate.sleep();
    }
    return 0;
}
