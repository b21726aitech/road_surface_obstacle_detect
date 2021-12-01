#include "ros/ros.h"
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Time.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Float32MultiArray.h>
#include <memory>

//最大識別距離の設定[m]
#define dist 25

sensor_msgs::PointCloud2 s_pc2;             //variable for send culc point data as ros message
std_msgs::Duration Ident_Time;
std_msgs::Time pc_time;
float distance = 0;                             //distance from velodyne to point data
sensor_msgs::PointCloud t_points;

// Subscribeする対象のトピックが更新されたら呼び出されるコールバック関数
// 引数にはトピックにPublishされるメッセージの型と同じ型を定義する
void identify_front_vehicle(const sensor_msgs::PointCloud2 msg){
  sensor_msgs::PointCloud pc, pc2;                //variable for convert pointcloud2 to pointcloud and send point data as ros message
  std_msgs::Float32MultiArray pc_intensity;
  int count = 0;                                  //calced ground point size
  float distance = 0;                             //distance from velodyne to point data

  //convert pointcloud2 to pointcloud
  //sensor_msgs::convertPointCloud2ToPointCloud(sensor_msgs::PointCloud2,sensor_msgs::PointCloud)
  sensor_msgs::convertPointCloud2ToPointCloud(msg, pc);
  t_points = pc;

  //calc of point size
  geometry_msgs::Point32 pointx[pc.points.size()];
  pc_intensity.data.resize(pc.channels[0].values.size());
  pc2.channels.resize(1);

  //get header and channels
  pc2.header = pc.header;
  pc2.channels[0].name = pc.channels[0].name;
  pc_time.data = pc.header.stamp;
  ros::Time ros_begin = ros::Time::now();

  //get ground point data
  for(int i = 0; i < pc.points.size(); i++){
    //距離計算
    distance = sqrt(pow(pc.points[i].x,2) + pow(pc.points[i].y,2) + pow(pc.points[i].z,2));
    //set point data in front of car
    if(distance < dist && pc.points[i].x > 0){
      //if(pc.points[i].y < 0 && pc.points[i].y > -1* pc.points[i].x * sin(M_PI / 6)){
      if(pc.points[i].y < 0 && pc.points[i].y > -1* pc.points[i].x * tan(M_PI / 6)){
        pointx[count].x=pc.points[i].x;
        pointx[count].y=pc.points[i].y;
        pointx[count].z=pc.points[i].z;
        pc_intensity.data[count]=pc.channels[0].values[i];
        count++;
      //}else if(pc.points[i].y > 0 && pc.points[i].y < pc.points[i].x * sin(M_PI / 6)){
    }else if(pc.points[i].y > 0 && pc.points[i].y < pc.points[i].x * tan(M_PI / 6)){
        pointx[count].x=pc.points[i].x;
        pointx[count].y=pc.points[i].y;
        pointx[count].z=pc.points[i].z;
        pc_intensity.data[count]=pc.channels[0].values[i];
        count++;
      }
    }
  }

  //get total point size and height
  pc2.points.resize(count);
  pc2.channels[0].values.resize(count);
  for(int i = 0; i < count; i++){
    pc2.points[i].x = pointx[i].x;
    pc2.points[i].y = pointx[i].y;
    pc2.points[i].z = pointx[i].z;
    pc2.channels[0].values[i] = pc_intensity.data[i];
  }

  //処理時間の計測
  ros::Time calc_time = ros::Time::now();
  ros::Duration D_Time = calc_time - ros_begin;
  ros::Time c_time = ros::Time::now();
  Ident_Time.data = D_Time;
  pc2.header.stamp = c_time;
  pc2.header.frame_id="velodyne";

  //convert pointcloud to pointcloud2
  //sensor_msgs::convertPointCloudToPointCloud2(sensor_msgs::PointCloud,sensor_msgs::PointCloud2)
  sensor_msgs::convertPointCloudToPointCloud2(pc2,s_pc2);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "identify_front_vehicle");
  ros::NodeHandle n;

  //更新周期設定[Hz]
  ros::Rate loop_rate(50);
  /*subscribeノード作成　
	  NodeHandle.subscribe(ノード名, 保存するキュー数, 実行する関数)*/
  ros::Subscriber identify_front_vehicle_sub = n.subscribe("points_ground", 10, identify_front_vehicle);
  /*Publishノード作成
	  ros::Publisher 変数 = NodeHandle.advertise<メッセージ形式>(ノード名, 発信するキュー数)*/
  ros::Publisher point_pub = n.advertise<sensor_msgs::PointCloud2>("change_point", 10);
  ros::Publisher Identified_Time = n.advertise<std_msgs::Duration>("Iden_times", 10);
  ros::Publisher test_points_pub = n.advertise<sensor_msgs::PointCloud>("test_points2", 10);
  ros::Publisher pc_Time_pub = n.advertise<std_msgs::Time>("pc_time", 10);


  //variable
  std_msgs::Float32 disper_msg;
  tf::TransformListener listener;

  //send message
  //[ros::Publisherの変数].publish(変数);
  while(ros::ok()){
    point_pub.publish(s_pc2);
    Identified_Time.publish(Ident_Time);
    test_points_pub.publish(t_points);
    pc_Time_pub.publish(pc_time);

    //Subscribeにキューがある場合関数を実行
    ros::spinOnce();
    /*次の更新周期になるまで停止
    　例えば更新周期50 Hzの場合20 msごとに実行するが以下の関数まで5 msしかかからない場合15 ms停止する*/
    loop_rate.sleep();
  }

  return 0;
}
