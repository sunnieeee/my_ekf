#ifndef ODOM_IMU_PUBLISHER_H
#define ODOM_IMU_PUBLISHER_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h" //robot_pose_ekf需订阅的里程计话题
#include "tf/transform_broadcaster.h"  //发布odom坐标系到base_link坐标系的变换
#include "sensor_msgs/Imu.h"  //robot_pose_ekf需订阅的imu话题

#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"

#include <boost/assign/list_of.hpp>
#include <algorithm>

#define PI 3.14159265

namespace odom_imu_ekf
{
//以结构体形式访问下位机上传的数据
typedef struct{
  float delta_car;  //小车中心位移
  float linear_speed;  //里程计算得的小车中心线速度
  float delta_theta;  //里程计算得的偏航角增量  角度制
  float angle_speed[2];  //z方向角速度，[0]里程计，[1]imu    角度制
  float linear_acc[2];  //imu算得的线加速度，[0]x方向，[1]y方向   单位为m/s^2
  float quat[4];  //imu解析出的四元数（对应方法二）
  unsigned int time_stamp;  //下位机时间戳

}UPLOAD_STATUS;

class OdomImuPublisher
{

public:
  UPLOAD_STATUS car_status;  //用于访问结构体中数据

  OdomImuPublisher();  //默认构造函数，构造类时对象的初始化
  void Refresh();  //更新imu和里程计的数据
  nav_msgs::Odometry get_odom();  //为对象CarOdom创建外部程序可访问的接口函数
  sensor_msgs::Imu get_imu();  //为对象CarImu创建外部程序可访问的接口函数

private:

  //类的组合，将Ros类中的数据成员作为本类的对象
  ros::NodeHandle n;  //句柄
  nav_msgs::Odometry CarOdom;  //Odometry消息
  sensor_msgs::Imu CarImu;  //Imu消息
  geometry_msgs::Pose2D CarPos2D;  //需要使用其中的theta来计算分位移，生成四元数
  ros::Publisher odom_pub;  //里程计的消息发布器
  ros::Publisher IMU_pub;  //imu的消息发布器

  double base_time_;  //用于Ros中时间转化 以s为单位
};

}  //namespace odom_imu_ekf


#endif //ODOM_IMU_PUBLISHER_H
