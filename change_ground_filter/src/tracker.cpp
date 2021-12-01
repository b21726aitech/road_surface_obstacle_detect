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
//閾値の決定[m]
#define threshold 0.03
//変数の設定
sensor_msgs::PointCloud2 s_pc2, s_pc3; //variable for send culc point data as ros message
sensor_msgs::PointCloud pc2, pc3;
std_msgs::Duration tracker_time_o, tracker_time_u;
float distance = 0;             //distance from velodyne to point data
tf::StampedTransform transform; //read base link to velodyne and map to base link
float tf_x, tf_y, tf_z, d_tf_x, d_tf_y, d_tf_z, d_tf_yaw;
int tf_check = 0;

//ハンプのトラッキング実行関数
void tracker(const sensor_msgs::PointCloud2 msg)
{
    sensor_msgs::PointCloud pc; //variable for convert pointcloud2 to pointcloud and send point data as ros message
    int count = 0;              //calced ground point size
    float distance = 0;         //distance from velodyne to point data
    int check = 0, check_count = 0, check_m = 0;
    float yaw = tf::getYaw(transform.getRotation());

    //convert pointcloud2 to pointcloud
    sensor_msgs::convertPointCloud2ToPointCloud(msg, pc);

    //calc of point size
    geometry_msgs::Point32 pointx[pc.points.size() + pc2.points.size()];
    //pc2.channels.resize(1);
    printf("yaw1: %f\n", yaw);
    yaw = yaw * -1;

    ros::Time ros_begin_o = ros::Time::now();

    check = pc2.points.size() + count;

    //座標の数値が全て0でない場合店データを保持し続ける
    for (int i = 0; i < pc2.points.size(); i++)
    {
        pointx[count].x = pc2.points[i].x;
        pointx[count].y = pc2.points[i].y;
        pointx[count].z = pc2.points[i].z;
        count++;

        // if (pc2.points[i].x != 0.0 && pc2.points[i].y != 0.0 && pc2.points[i].z != 0.0)
        // {
        //     //if(pc2.points[i].y - tf_y < 0 && pc2.points[i].y - tf_y > -1* (pc2.points[i].x - tf_x) * sin(M_PI / 6 - yaw)){
        //     if (pc2.points[i].y - tf_y < 0 && pc2.points[i].y - tf_y > -1 * (pc2.points[i].x - tf_x) * tan(M_PI / 6 - yaw))
        //     {
        //         pointx[count].x = pc2.points[i].x;
        //         pointx[count].y = pc2.points[i].y;
        //         pointx[count].z = pc2.points[i].z;
        //         count++;
        //         //}else if(pc2.points[i].y - tf_y > 0 && pc2.points[i].y - tf_y < (pc2.points[i].x - tf_x)* sin(M_PI / 6 - yaw)){
        //     }
        //     else if (pc2.points[i].y - tf_y > 0 && pc2.points[i].y - tf_y < (pc2.points[i].x - tf_x) * tan(M_PI / 6 - yaw))
        //     {
        //         pointx[count].x = pc2.points[i].x;
        //         pointx[count].y = pc2.points[i].y;
        //         pointx[count].z = pc2.points[i].z;
        //         count++;
        //     }
        // }
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
    printf("count: %d\n", count);
    //get total point size and height
    check_count = pc2.points.size();
    pc2.points.resize(check);
    //トラッキング結果の点群データを送信するためにgeometry_msgs::Point32からsensor_msgs::PointCloudに変換
    for (int i = 0; i < count; i++)
    {
        pc2.points[i].x = pointx[i].x;
        pc2.points[i].y = pointx[i].y;
        pc2.points[i].z = pointx[i].z;
    }

    //処理時間の計測
    ros::Time calc_time = ros::Time::now();
    ros::Duration D_Time = calc_time - ros_begin_o;
    ros::Time c_time = ros::Time::now();
    tracker_time_o.data = D_Time;
    pc2.header.stamp = c_time;
    pc2.header.frame_id = "map";

    //convert pointcloud to pointcloud2
    sensor_msgs::convertPointCloudToPointCloud2(pc2, s_pc2);

    //いらない
    d_tf_x = tf_x;
    d_tf_y = tf_y;
    d_tf_z = tf_z;
    d_tf_yaw = yaw;
}

//ポットホールのトラッキングを実行関数
void tracker2(const sensor_msgs::PointCloud2 msg)
{
    sensor_msgs::PointCloud pc; //variable for convert pointcloud2 to pointcloud and send point data as ros message
    int count = 0;              //calced ground point size
    float distance = 0;         //distance from velodyne to point data
    int check = 0, check_count = 0, check_m = 0;
    float yaw = tf::getYaw(transform.getRotation());

    //convert pointcloud2 to pointcloud
    sensor_msgs::convertPointCloud2ToPointCloud(msg, pc);

    //calc of point size
    geometry_msgs::Point32 pointx[pc.points.size() + pc3.points.size()];
    //pc2.channels.resize(1);
    printf("yaw: %f\n", yaw);
    yaw = yaw * -1;

    //get header and channels
    //pc3.header = pc.header;
    //pc3.channels[0].name = pc.channels[0].name;
    ros::Time ros_begin_u = ros::Time::now();

    check = pc3.points.size() + count;

    //座標の数値が全て0でなく車両前方の60度の扇型の範囲にある場合店データを保持し続ける
    for (int i = 0; i < pc3.points.size(); i++)
    {
        pointx[count].x = pc3.points[i].x;
        pointx[count].y = pc3.points[i].y;
        pointx[count].z = pc3.points[i].z;
        count++;

        // if (pc3.points[i].x != 0.0 && pc3.points[i].y != 0.0 && pc3.points[i].z != 0.0)
        // {
        //     //if(pc3.points[i].y - tf_y < 0 && pc3.points[i].y - tf_y > -1* (pc3.points[i].x - tf_x) * sin(M_PI / 6 - yaw)){
        //     if (pc3.points[i].y - tf_y < 0 && pc3.points[i].y - tf_y > -1 * (pc3.points[i].x - tf_x) * tan(M_PI / 6 - yaw))
        //     {
        //         pointx[count].x = pc3.points[i].x;
        //         pointx[count].y = pc3.points[i].y;
        //         pointx[count].z = pc3.points[i].z;
        //         count++;
        //         //}else if(pc3.points[i].y - tf_y > 0 && pc3.points[i].y - tf_y < (pc3.points[i].x - tf_x)* sin(M_PI / 6 - yaw)){
        //     }
        //     else if (pc3.points[i].y - tf_y > 0 && pc3.points[i].y - tf_y < (pc3.points[i].x - tf_x) * tan(M_PI / 6 - yaw))
        //     {
        //         pointx[count].x = pc3.points[i].x;
        //         pointx[count].y = pc3.points[i].y;
        //         pointx[count].z = pc3.points[i].z;
        //         count++;
        //     }
        // }
    }

    //get ground point data
    for (int i = 0; i < pc.points.size(); i++)
    {
        //set point data in front of car
        float x, y, z;
        check_m = 0;
        /*座標変換 ここではmap座標系内にあるvelodyne座標系の位置情報を取得する 平行移動と回転移動から変換　論文を参考*/
        x = tf_x + (pc.points[i].x * cos(yaw) + pc.points[i].y * sin(yaw));
        y = tf_y + (-pc.points[i].x * sin(yaw) + pc.points[i].y * cos(yaw));
        z = pc.points[i].z + tf_z;
        //新たに取得した点データを保持している点データの位置が閾値内にある場合保持しない
        for (int j = 0; j < pc3.points.size(); j++)
        {
            if (std::abs(x - pc3.points[j].x) < threshold && std::abs(y - pc3.points[j].y) < threshold && std::abs(z - pc3.points[j].z) < threshold)
            {
                check_m = 1;
                //break;があるといいかも
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
    printf("count1: %d\n", count);
    //get total point size and height
    check_count = pc3.points.size();
    pc3.points.resize(check);
    //トラッキング結果の点群データを送信するためにgeometry_msgs::Point32からsensor_msgs::PointCloudに変換
    for (int i = 0; i < count; i++)
    {
        pc3.points[i].x = pointx[i].x;
        pc3.points[i].y = pointx[i].y;
        pc3.points[i].z = pointx[i].z;
    }

    //処理時間の計測
    ros::Time calc_time = ros::Time::now();
    ros::Duration D_Time = calc_time - ros_begin_u;
    ros::Time c_time = ros::Time::now();
    tracker_time_u.data = D_Time;
    pc3.header.stamp = c_time;
    pc3.header.frame_id = "map";

    //convert pointcloud to pointcloud2
    sensor_msgs::convertPointCloudToPointCloud2(pc3, s_pc3);

    //いらない
    d_tf_x = tf_x;
    d_tf_y = tf_y;
    d_tf_z = tf_z;
    d_tf_yaw = yaw;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracker");
    ros::NodeHandle n;
    tf::TransformListener listener;

    //更新周期設定[Hz]
    ros::Rate loop_rate(50);

    /*subscribeノード作成　
	  NodeHandle.subscribe(ノード名, 保存するキュー数, 実行する関数)*/
    ros::Subscriber tracker_over_sub = n.subscribe("clusters_over", 10, tracker);
    ros::Subscriber tracker_under_sub = n.subscribe("clusters_under", 10, tracker2);

    /*Publishノード作成
	  ros::Publisher 変数 = NodeHandle.advertise<メッセージ形式>(ノード名, 発信するキュー数)*/
    ros::Publisher tracker_pub = n.advertise<sensor_msgs::PointCloud2>("tracker_over", 10);
    ros::Publisher tracker_u_pub = n.advertise<sensor_msgs::PointCloud2>("tracker_under", 10);
    ros::Publisher tracker_Time_pub = n.advertise<std_msgs::Duration>("tracker_over_time", 10);
    ros::Publisher tracker_Time_u_pub = n.advertise<std_msgs::Duration>("tracker_under_time", 10);

    //variable
    std_msgs::Float32 disper_msg;

    //send message
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
        //send message
        //[ros::Publisherの変数].publish(変数);
        tracker_pub.publish(s_pc2);
        tracker_u_pub.publish(s_pc3);
        tracker_Time_pub.publish(tracker_time_o);
        tracker_Time_u_pub.publish(tracker_time_u);

        //Subscribeにキューがある場合関数を実行
        ros::spinOnce();
        /*次の更新周期になるまで停止
    　例えば更新周期50 Hzの場合20 msごとに実行するが以下の関数まで5 msしかかからない場合15 ms停止する*/
        loop_rate.sleep();
    }

    return 0;
}
