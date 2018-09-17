#include "odom_imu_publisher.h"
#include <math.h>
#include <stdlib.h>

namespace odom_imu_ekf
{

OdomImuPublisher::OdomImuPublisher()
{

  //初始化odom
  //假设机器人的运动起点是“odom”坐标系的原点
  CarOdom.pose.pose.position.x = 0.0;
  CarOdom.pose.pose.position.y = 0.0;
  CarOdom.pose.pose.position.z = 0.0;
  //假设机器人从静止开始运动
  CarOdom.twist.twist.linear.x = 0.0;
  CarOdom.twist.twist.linear.y = 0.0;
  CarOdom.twist.twist.linear.z = 0.0;
  CarOdom.twist.twist.angular.x = 0.0;
  CarOdom.twist.twist.angular.y = 0.0;
  CarOdom.twist.twist.angular.z = 0.0;

  //初始化imu
  //四元数位姿
  CarImu.orientation.x = 0.0;
  CarImu.orientation.y = 0.0;
  CarImu.orientation.z = 0.0;
  CarImu.orientation.w = 0.0;
  //线加速度
  CarImu.linear_acceleration.x = 0.0;
  CarImu.linear_acceleration.y = 0.0;
  CarImu.linear_acceleration.z = 0.0;
  //角速度
  CarImu.angular_velocity.x = 0.0;
  CarImu.angular_velocity.y = 0.0;
  CarImu.angular_velocity.z = 0.0;

  //初始化消息发布器
  odom_pub = n.advertise<nav_msgs::Odometry>("odom_imu_ekf/Odom", 500);  //500代表发布队列的大小，如果消息发布太快，会缓冲最多500条消息
  IMU_pub = n.advertise<sensor_msgs::Imu>("odom_imu_ekf/Imu_data", 500);

  base_time_ = ros::Time::now().toSec();  //将ROS中的时刻转化为秒
}


void OdomImuPublisher::Refresh()
{

  //Time
  ros::Time current_time;



  //Odometry里程计数据
  //header
  CarOdom.header.stamp = current_time.fromSec(base_time_);
  CarOdom.header.frame_id = "odom";
  CarOdom.child_frame_id = "base_footprint";

  //pose
  float delta_x,delta_y;  //delta_t时间内x，y方向小车分位移

  if(car_status.delta_theta > 270 ) car_status.delta_theta -= 360;
  if(car_status.delta_theta < -270 ) car_status.delta_theta += 360;
  CarPos2D.theta+=car_status.delta_theta;

  delta_x=car_status.delta_car*cos(CarPos2D.theta* PI / 180.0f);
  delta_y=car_status.delta_car*sin(CarPos2D.theta* PI / 180.0f);

  CarOdom.pose.pose.position.x+= delta_x;
  CarOdom.pose.pose.position.y+= delta_y;
  CarOdom.pose.pose.position.z = 0.0f;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(CarPos2D.theta/180.0f*PI);
  CarOdom.pose.pose.orientation = odom_quat;

  //twist
  CarOdom.twist.twist.linear.x = car_status.linear_speed;
  CarOdom.twist.twist.linear.y = 0.0f;  //如果linear.y 不为0，说明小车要沿着y轴运动，这会导致两轮的差速，但是对于两轮控制的移动机器人，twist.linear.y = 0 是在move_base的配置文件base_local_planner_params.yaml中有明确指定，所以不需关注linear.y的转换
  CarOdom.twist.twist.angular.z = car_status.angle_speed[0] * PI /180.0f;  //转化为弧度制

  //covariance
  //若参考turtlebot官方协方差矩阵
  if(car_status.linear_speed==0 && car_status.angle_speed[0]==0 && car_status.angle_speed[1]==0)  //小车静止时对应的协方差矩阵
  {
    CarOdom.pose.covariance =  boost::assign::list_of(1e-9) (0) (0)  (0)  (0)  (0)
                                                      (0) (1e-3)  (1e-9)  (0)  (0)  (0)
                                                      (0)   (0)  (1e6) (0)  (0)  (0)
                                                      (0)   (0)   (0) (1e6) (0)  (0)
                                                      (0)   (0)   (0)  (0) (1e6) (0)
                                                      (0)   (0)   (0)  (0)  (0)  (1e-9) ;
    CarOdom.twist.covariance =  boost::assign::list_of(1e-9) (0) (0)  (0)  (0)  (0)
                                                      (0) (1e-3)  (1e-9)  (0)  (0)  (0)
                                                      (0)   (0)  (1e6) (0)  (0)  (0)
                                                      (0)   (0)   (0) (1e6) (0)  (0)
                                                      (0)   (0)   (0)  (0) (1e6) (0)
                                                      (0)   (0)   (0)  (0)  (0)  (1e-9) ;
  }
  else  //小车运动时对应的协方差矩阵
  {
    CarOdom.pose.covariance =  boost::assign::list_of(1e-3) (0) (0)  (0)  (0)  (0)
                                                      (0) (1e-3)  (0)  (0)  (0)  (0)
                                                      (0)   (0)  (1e6) (0)  (0)  (0)
                                                      (0)   (0)   (0) (1e6) (0)  (0)
                                                      (0)   (0)   (0)  (0) (1e6) (0)
                                                      (0)   (0)   (0)  (0)  (0)  (1e3) ;
    CarOdom.twist.covariance =  boost::assign::list_of(1e-3) (0) (0)  (0)  (0)  (0)
                                                      (0) (1e-3)  (0)  (0)  (0)  (0)
                                                      (0)   (0)  (1e6) (0)  (0)  (0)
                                                      (0)   (0)   (0) (1e6) (0)  (0)
                                                      (0)   (0)   (0)  (0) (1e6) (0)
                                                      (0)   (0)   (0)  (0)  (0)  (1e3) ;
  }


/*
  //若参考小强给出的协方差矩阵

  float wheel_radius=0.06;  //车轮半径，需根据实际小车更改  单位：m
  int encoder_ppr=4*12*64;  //车轮1转对应的编码器个数，需根据实际更改  单位：个
  float delta_time=0.01;  //底层DSP采样周期，需根据底层更改  单位：s
  float var_len,var_angle;  //协方差矩阵中的参数
  var_len=(1.0f/delta_time/car_status.encoder_ppr*2.0f*PI*wheel_radius)*(1.0f/delta_time/car_status.encoder_ppr*2.0f*PI*wheel_radius);
  var_angle=(0.01f/180.0f*PI)*(0.01f/180.0f*PI);
  CarOdom.pose.covariance =  boost::assign::list_of(var_len) (0) (0)  (0)  (0)  (0)
                                                        (0) (var_len)  (0)  (0)  (0)  (0)
                                                        (0)   (0)  (999) (0)  (0)  (0)
                                                        (0)   (0)   (0) (999) (0)  (0)
                                                        (0)   (0)   (0)  (0) (999) (0)
                                                        (0)   (0)   (0)  (0)  (0)  (var_angle) ;
  CarOdom.twist.covariance =  boost::assign::list_of(var_len) (0) (0)  (0)  (0)  (0)
                                                        (0) (var_len)  (0)  (0)  (0)  (0)
                                                        (0)   (0)  (999) (0)  (0)  (0)
                                                        (0)   (0)   (0) (999) (0)  (0)
                                                        (0)   (0)   (0)  (0) (999) (0)
                                                        (0)   (0)   (0)  (0)  (0)  (var_angle) ;
*/


  //publish odometry
  odom_pub.publish(CarOdom);



  //Imu惯性测量单元数据
  //header
  CarImu.header.stamp = current_time.fromSec(base_time_);
  CarImu.header.frame_id = "base_footprint";

  //orientation
/*
  //方法一：借tf由偏航角生成四元数
  geometry_msgs::Quaternion imu_quat = tf::createQuaternionMsgFromYaw(CarPos2D.theta/180.0f*PI);  //
  CarOdom.pose.pose.orientation = imu_quat;
*/

  //方法二：直接从imu中读取生成的四元数
  CarOdom.pose.pose.orientation.x = car_status.quat[1];
  CarOdom.pose.pose.orientation.y = car_status.quat[2];
  CarOdom.pose.pose.orientation.z = car_status.quat[3];
  CarOdom.pose.pose.orientation.w = car_status.quat[0];
  //imu的orientation协方差矩阵值，需根据imu用户手册及实际调试情况修改
  //若参考turtlebot官方orientation协方差矩阵
  CarImu.orientation_covariance[0] = 1000000;
  CarImu.orientation_covariance[1] = 0;
  CarImu.orientation_covariance[2] = 0;
  CarImu.orientation_covariance[3] = 0;
  CarImu.orientation_covariance[5] = 0;
  CarImu.orientation_covariance[6] = 0;
  CarImu.orientation_covariance[7] = 0;
  CarImu.orientation_covariance[8] = 0.000001;

  //angular_velocity
  CarImu.linear_acceleration.x = car_status.linear_acc[0];
  CarImu.linear_acceleration.y = car_status.linear_acc[1];
  CarImu.linear_acceleration.z = 0.0;
  //imu的线加速度协方差矩阵值，需根据imu用户手册及实际调试情况修改
  //若参考turtlebot官方orientation协方差矩阵
  CarImu.linear_acceleration_covariance[0] = -1;
  CarImu.linear_acceleration_covariance[1] = 0;
  CarImu.linear_acceleration_covariance[2] = 0;
  CarImu.linear_acceleration_covariance[3] = 0;
  CarImu.linear_acceleration_covariance[4] = 0;
  CarImu.linear_acceleration_covariance[5] = 0;
  CarImu.linear_acceleration_covariance[6] = 0;
  CarImu.linear_acceleration_covariance[7] = 0;
  CarImu.linear_acceleration_covariance[8] = 0;

  //linear_acceleration
  CarImu.angular_velocity.x = 0.0;
  CarImu.angular_velocity.y = 0.0;
  CarImu.angular_velocity.z = car_status.angle_speed[1] * PI /180.0f;  //转化为弧度制 //注意符号！
  //imu的角速度协方差矩阵值，需根据imu用户手册及实际调试情况修改
  //若参考turtlebot官方orientation协方差矩阵
  CarImu.angular_velocity_covariance[0] = 1000000;
  CarImu.angular_velocity_covariance[1] = 0;
  CarImu.angular_velocity_covariance[2] = 0;
  CarImu.angular_velocity_covariance[3] = 0;
  CarImu.angular_velocity_covariance[5] = 0;
  CarImu.angular_velocity_covariance[6] = 0;
  CarImu.angular_velocity_covariance[7] = 0;
  CarImu.angular_velocity_covariance[8] = 0.000001;

  //publish imu
  IMU_pub.publish(CarImu);


  ros::spinOnce();
}

nav_msgs::Odometry OdomImuPublisher::get_odom()
{
  return CarOdom;
}
sensor_msgs::Imu OdomImuPublisher::get_imu()
{
  return CarImu;
}

}  //namespace odom_imu_ekf
