#include "odom_imu_publisher.h"
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_imu_ekf");  //初始化
  ros::start();
  odom_imu_ekf::OdomImuPublisher est;  //创建odom_imu_ekf命名空间中OdomImuPublisher类的对象est
  ros::Rate r(100);  //发布周期为50hz

  while(ros::ok())
  {
    est.Refresh();  //定时发布imu和里程计话题
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
