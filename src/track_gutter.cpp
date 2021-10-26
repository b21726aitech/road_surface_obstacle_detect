#include <memory>

#include <ros/ros.h>

#include <std_msgs/Duration.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Time.h>

#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>

#define threshold 0.03 //閾値の決定[m]

//変数の設定
sensor_msgs::PointCloud2 s_pc2; //variable for send culc point data as ros message
sensor_msgs::PointCloud pc2;
std_msgs::Duration value_track_time_gutter;
float distance = 0;             //distance from velodyne to point data
tf::StampedTransform transform; //read base link to velodyne and map to base link
float tf_x, tf_y, tf_z, d_tf_x, d_tf_y, d_tf_z, d_tf_yaw;
ros::Subscriber tracker_over_sub;
ros::Publisher tracker_pub;
ros::Publisher track_time_gutter_pub_;

// 側溝のトラッキング実行コールバック関数
void tracker(const sensor_msgs::PointCloud2 msg)
{
    // トラッキング時間計測
    ros::Time track_start_time = ros::Time::now();

    sensor_msgs::PointCloud pc; //variable for convert pointcloud2 to pointcloud and send point data as ros message
    int count = 0;              //calced ground point size
    float distance = 0;         //distance from velodyne to point data
    int check = 0, check_count = 0, check_m = 0;
    float yaw = tf::getYaw(transform.getRotation());

    sensor_msgs::convertPointCloud2ToPointCloud(msg, pc); // 型変換

    //pc2.channels.resize(1);
    // printf("yaw1: %f\nh", yaw);
    yaw = yaw * -1;

    check = pc2.points.size() + count;

    // std::cout << check << std::endl;

    // geometry_msgs::Point32 pointx[pc.points.size()];
    geometry_msgs::Point32 pointx[pc.points.size() + pc2.points.size()];

    //座標の数値が全て0でない場合点群データを保持し続ける
    for (int i = 0; i < pc2.points.size(); i++)
    {
        if (pc2.points[i].x != 0.0 && pc2.points[i].y != 0.0 && pc2.points[i].z != 0.0)
        {

            pointx[count].x = pc2.points[i].x;
            pointx[count].y = pc2.points[i].y;
            pointx[count].z = pc2.points[i].z;
            count++;

            // //if(pc2.points[i].y - tf_y < 0 && pc2.points[i].y - tf_y > -1* (pc2.points[i].x - tf_x) * sin(M_PI / 6 - yaw)){
            // if (pc2.points[i].y - tf_y < 0 && pc2.points[i].y - tf_y > -1 * (pc2.points[i].x - tf_x) * tan(M_PI / 6 - yaw))
            // {
            //     pointx[count].x = pc2.points[i].x;
            //     pointx[count].y = pc2.points[i].y;
            //     pointx[count].z = pc2.points[i].z;
            //     count++;
            //     //}else if(pc2.points[i].y - tf_y > 0 && pc2.points[i].y - tf_y < (pc2.points[i].x - tf_x)* sin(M_PI / 6 - yaw)){
            // }
            // else if (pc2.points[i].y - tf_y > 0 && pc2.points[i].y - tf_y < (pc2.points[i].x - tf_x) * tan(M_PI / 6 - yaw))
            // {
            //     pointx[count].x = pc2.points[i].x;
            //     pointx[count].y = pc2.points[i].y;
            //     pointx[count].z = pc2.points[i].z;
            //     count++;
            // }
        }
    }

    //get ground point data
    for (int i = 0; i < pc.points.size(); i++)
    {
        //set point data in front of car
        float x = 0, y = 0, z = 0;
        check_m = 0;
        /*座標変換 ここではvelodyne座標系外にあるmap座標系に変換　平行移動と回転移動から変換　論文を参考*/
        x = tf_x + (pc.points[i].x * cos(yaw) + pc.points[i].y * sin(yaw));
        y = tf_y + (-pc.points[i].x * sin(yaw) + pc.points[i].y * cos(yaw));
        z = tf_z + pc.points[i].z;
        //新たに取得した点データを保持している点データの位置が閾値内にある場合保持しない
        for (int j = 0; j < pc2.points.size(); j++)
        {
            if (std::abs(x - pc2.points[j].x) < threshold && std::abs(y - pc2.points[j].y) < threshold && std::abs(z - pc2.points[j].z) < threshold)
            {
                check_m = 1;
                break;
            }
        }
        //点データを保持
        if (check_m == 0 || check == 0)
        {
            pointx[count].x = x;
            pointx[count].y = y;
            pointx[count].z = z;
            count++;
        }
    }

    //点データ数を確認
    check = count;
    // printf("count: %d\nh", count);
    //get total point size and height
    check_count = pc2.points.size();
    pc2.points.resize(check);
    //トラッキング結果の点群データを送信するためにgeometry_msgs::Point32からsensor_msgs::PointCloudに変換
    for (int i = 0; i < count; i++)
    {
        if (pointx[i].z < 0.3) ///velodyne座標に点が誤って記録されるバグの応急処置
        {
            pc2.points[i].x = pointx[i].x;
            pc2.points[i].y = pointx[i].y;
            pc2.points[i].z = pointx[i].z;
        }
    }

    // トラッキング時間計測
    ros::Time track_end_time = ros::Time::now();
    ros::Duration duration_tracking_gutter = track_end_time - track_start_time;
    value_track_time_gutter.data = duration_tracking_gutter;
    pc2.header.stamp = track_end_time;
    pc2.header.frame_id = "map";

    //convert pointcloud to pointcloud2
    sensor_msgs::convertPointCloudToPointCloud2(pc2, s_pc2);

    //いらない
    // d_tf_x = tf_x;
    // d_tf_y = tf_y;
    // d_tf_z = tf_z;
    // d_tf_yaw = yaw;

    tracker_pub.publish(s_pc2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "track_gutter");
    ros::NodeHandle nh;

    tracker_over_sub = nh.subscribe("cluster_gutter", 1, tracker);
    tracker_pub = nh.advertise<sensor_msgs::PointCloud2>("tracked_gutter", 10);
    track_time_gutter_pub_ = nh.advertise<std_msgs::Duration>("track_time_gutter", 1000);

    tf::TransformListener listener;
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        //get coordinate from map to baselink and get Coordinate base link to velodyne
        while (true)
        {
            try
            {
                //座標系間の位置を取得　ここではmap座標系内にあるvelodyne座標系の位置情報を取得する
                listener.lookupTransform("map", "velodyne", ros::Time(0), transform);
                tf_x = transform.getOrigin().x(); //distance from base link to velodyne
                tf_y = transform.getOrigin().y();
                tf_z = transform.getOrigin().z();
                ROS_INFO("Transformed!");
                break;
            }
            catch (tf::TransformException ex)
            {
                ROS_INFO("Transform Exception");
                ros::Duration(0.1).sleep();
            }
        }

        track_time_gutter_pub_.publish(value_track_time_gutter);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
