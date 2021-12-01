/*
RANSAC参考資料
(https://aoyagisouko.blogspot.com/2020/05/pcl-plane-coefficients.html)
*/

#include "ros/ros.h"
#include <stdio.h>
#include <time.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/conversions.h>
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
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Time.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Float32MultiArray.h>
#include <jsk_recognition_msgs/SimpleOccupancyGrid.h>
#include <jsk_recognition_msgs/SimpleOccupancyGridArray.h>

//閾値の決定[m]
#define threshold 0.03
#define o_threshold 0.13

//変数の設定
float dispersion = 0; //variable for send dispersion as ros message
float plane[4] = {0, 0, 0, 0};
std_msgs::Float32 average_height;
std_msgs::Float32 deviation_height;
sensor_msgs::PointCloud2 s_pc2, s_pc3;         //variable for send culc point data as ros message
sensor_msgs::PointCloud2 s_o_u_pc2, s_o_o_pc2; //variable for send culc point data as ros message
sensor_msgs::PointCloud2 s_o_u_pc3, s_o_o_pc3; //variable for send culc point data as ros message
sensor_msgs::PointCloud t_points;
std_msgs::Duration Ident_Time, Ident_Time_Plane;
std_msgs::Float32MultiArray height;
std_msgs::Float32MultiArray intensity;
jsk_recognition_msgs::SimpleOccupancyGrid grid;
// jsk_recognition_msgs::SimpleOccupancyGridArray gridarray;
float culc_height = 0;
std_msgs::Float32MultiArray pc2_intensity, pc3_intensity;

//平面近似をMSACで行なう
void calc_planer(const sensor_msgs::PointCloud2 msg)
{
    /*sensor_msgs::PointCloud2をpcl::PointCloud<pcl::PointXYZ>に変換するための変数*/
    pcl::PointCloud<pcl::PointXYZ> cloud;

    /*sensor_msgs::PointCloud2をpcl::PointCloud<pcl::PointXYZ>に変換
    pcl::fromROSMsg(sensor_msgs::PointCloud2,pcl::PointCloud<pcl::PointXYZ>)*/
    pcl::fromROSMsg(msg, cloud);
    /*pcl::PointCloud<pcl::PointXYZ>からpcl::PointCloud<pcl::PointXYZ>::Ptrに変換するための変数
  　近似プログラムはポインタを用いるためPtrに変換しないと動作しない*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    /*pcl::PointCloud<pcl::PointXYZ>からpcl::PointCloud<pcl::PointXYZ>::Ptrに変換*/
    *cloud_ptr = cloud;

    ros::Time ros_begin_planer = ros::Time::now();

    //RANSACを用いた平面モデル抽出処理
    //抽出結果保持用モデル
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    //モデル抽出時に使用された点群のインデックスを保持する変数のポインタ
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    //segmentationオブジェクトの生成
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    //RANSACにおいて最適化を実施するかどうか
    seg.setOptimizeCoefficients(true);
    //抽出モデルに平面を指定
    /*今回は平面での近似を行ったがpclには他にも様々な近似モデルが設定できる. 以下参考
    (https://pointclouds.org/documentation/model__types_8h_source.html)*/
    seg.setModelType(pcl::SACMODEL_PLANE);
    //抽出モデルにMSACを指定
    /*今回は近似手法をMSACを使用したがpclには他にも様々な禁じ手法が設定できる. 以下参考
    (https://pointclouds.org/documentation/method__types_8h_source.html)*/
    seg.setMethodType(pcl::SAC_MSAC);
    //許容する誤差しきい値
    //LiDARの誤差が3 cmのため0.03 mに指定
    seg.setDistanceThreshold(0.03);
    //モデル抽出対象点群のセット
    seg.setInputCloud(cloud_ptr);
    //モデル抽出の実行
    seg.segment(*inliers, *coefficients);
    //抽出結果モデルの表示
    /*平面の方程式は ax +by + cz + d = 0で表される
    coefficients->value[0-3]には それぞれ a,b,c,dの順で格納される*/

    ROS_INFO("coefficients : [0, 1, 2, 3] = [%5.3lf, %5.3lf, %5.3lf, %5.3lf]",
             coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
    plane[0] = coefficients->values[0];
    plane[1] = coefficients->values[1];
    plane[2] = coefficients->values[2];
    plane[3] = coefficients->values[3];

    ros::Time c_time = ros::Time::now();

    /*いらない*/
    /*
    gridarray.header.frame_id = "velodyne";
    gridarray.header.stamp = c_time;
    grid.header.stamp = c_time;
    grid.header.frame_id = "velodyne";
    grid.coefficients[0] = plane[0];
    grid.coefficients[1] = plane[1];
    grid.coefficients[2] = plane[2];
    grid.coefficients[3] = plane[3];
    grid.resolution = 1;
    grid.cells[0].x = 0;
    grid.cells[0].y = 0;
    grid.cells[0].z = 0;
    std::cout << "[ debug]" << std::endl;
    gridarray.grids[0] = grid;
    std::cout << "[ debug2]" << std::endl;
    */

    //処理時間計測
    ros::Time calc_times = ros::Time::now();
    ros::Duration D_Times = calc_times - ros_begin_planer;
    Ident_Time_Plane.data = D_Times;
}

// Subscribeする対象のトピックが更新されたら呼び出されるコールバック関数
// 引数にはトピックにPublishされるメッセージの型と同じ型を定義する
void identify_surface(const sensor_msgs::PointCloud2 msg)
{
    sensor_msgs::PointCloud pc2, pc3;         //variable for convert pointcloud2 to pointcloud and send point data as ros message
    sensor_msgs::PointCloud o_u_pc2, o_o_pc2; //variable for convert pointcloud2 to pointcloud and send point data as ros message
    sensor_msgs::PointCloud o_u_pc3, o_o_pc3; //variable for convert pointcloud2 to pointcloud and send point data as ros message
    int count = 0;                            //calced ground point size
    int o_u_count = 0, o_o_count = 0;         //calced ground point size
    int o_u_count2 = 0, o_o_count2 = 0;       //calced ground point size
    float total = 0;                          //sum of ground height
    float ave = 0;                            //average of ground height
    float ave_intensity = 0, dev_intensity = 0;
    dispersion = 0;

    //convert pointcloud2 to pointcloud
    sensor_msgs::PointCloud2ConstPtr pc2_ptr;
    //pc2_ptr = msg;
    //平面近似実行
    calc_planer(msg);

    ros::Time ros_begin = ros::Time::now();

    //sensor_msgs::PointCloud2からsensor_msgs::PointCloudに変換
    //sensor_msgs::convertPointCloud2ToPointCloud(sensor_msgs::PointCloud2,sensor_msgs::PointCloud)
    sensor_msgs::convertPointCloud2ToPointCloud(msg, pc2);
    //sensor_msgs::PointCloudの設定
    t_points = pc2;
    pc3.channels.resize(1);
    pc3.header = pc2.header;
    pc3.channels[0].name = pc2.channels[0].name;
    o_u_pc2.header = pc2.header;
    o_u_pc2.channels = pc2.channels;
    o_o_pc2.header = pc2.header;
    o_o_pc2.channels = pc2.channels;
    o_u_pc3.header = pc2.header;
    o_u_pc3.channels = pc2.channels;
    o_o_pc3.header = pc2.header;
    o_o_pc3.channels = pc2.channels;

    /*路面識別後の点データを格納するための変数設定
  pointx2は路面の点データを格納
  o_u_pointxは路面下部の点データを格納
  o_u_pointx2はポットホールの範囲内の点データを格納
  o_o_pointxは路面上部の点データを格納
  o_o_pointx2はハンプのは担いの点データを格納*/
    geometry_msgs::Point32 pointx2[pc2.points.size()];
    geometry_msgs::Point32 o_u_pointx[pc2.points.size()];
    geometry_msgs::Point32 o_o_pointx[pc2.points.size()];
    geometry_msgs::Point32 o_u_pointx2[pc2.points.size()];
    geometry_msgs::Point32 o_o_pointx2[pc2.points.size()];
    pc2_intensity.data.resize(pc2.channels[0].values.size());

    for (int i = 0; i < pc2.points.size(); i++)
    {
        /*LiDARのx,y座標から平面近似結果の高さデータを算出
    　この結果とLiDARの高さデータを比較して路面か路面障害物かを識別
    　平面の方程式 ax + by + cz + d = 0から変換
    　z = (-1*(ax + by + d)) / c*/
        culc_height = (-1 * (plane[0] * pc2.points[i].x + plane[1] * pc2.points[i].y + plane[3])) / plane[2];
        //路面範囲内の場合
        if (pc2.points[i].z - culc_height > -threshold && pc2.points[i].z - culc_height < threshold)
        {
            pointx2[count].x = pc2.points[i].x;
            pointx2[count].y = pc2.points[i].y;
            pointx2[count].z = pc2.points[i].z;
            //反射強度を取得
            pc2_intensity.data[count] = pc2.channels[0].values[i];
            count++;
            dispersion = dispersion + pow(pc2.points[i].z - culc_height, 2);
        }
        //路面下部範囲内の場合
        if (pc2.points[i].z - culc_height <= -o_threshold)
        {
            o_u_pointx[o_u_count].x = pc2.points[i].x;
            o_u_pointx[o_u_count].y = pc2.points[i].y;
            o_u_pointx[o_u_count].z = pc2.points[i].z;
            o_u_count++;
        }
        //ポットホールの範囲内の場合
        if (pc2.points[i].z - culc_height > -o_threshold && pc2.points[i].z - culc_height <= -threshold)
        {
            o_u_pointx2[o_u_count2].x = pc2.points[i].x;
            o_u_pointx2[o_u_count2].y = pc2.points[i].y;
            o_u_pointx2[o_u_count2].z = pc2.points[i].z;
            o_u_count2++;
        }
        //路面上部範囲内の場合
        if (pc2.points[i].z - culc_height >= o_threshold)
        {
            o_o_pointx[o_o_count].x = pc2.points[i].x;
            o_o_pointx[o_o_count].y = pc2.points[i].y;
            o_o_pointx[o_o_count].z = pc2.points[i].z;
            o_o_count++;
        }
        //ハンプの範囲内の場合
        if (pc2.points[i].z - culc_height < o_threshold && pc2.points[i].z - culc_height > threshold)
        {
            o_o_pointx2[o_o_count2].x = pc2.points[i].x;
            o_o_pointx2[o_o_count2].y = pc2.points[i].y;
            o_o_pointx2[o_o_count2].z = pc2.points[i].z;
            o_o_count2++;
        }
    }

    /*geometry_msgs::Point32からsensor_msgs::PointCloudに変換
    ここでは, 路面の点群データを処理 反射強度の合計も取得*/
    pc3.points.resize(count);
    pc3.channels[0].values.resize(count);
    pc3_intensity.data.resize(count);
    for (int i = 0; i < count; i++)
    {
        pc3.points[i].x = pointx2[i].x;
        pc3.points[i].y = pointx2[i].y;
        pc3.points[i].z = pointx2[i].z;
        pc3.channels[0].values[i] = pc2_intensity.data[i];
        pc3_intensity.data[i] = pc2_intensity.data[i];
        ave_intensity = ave_intensity + pc2_intensity.data[i];
        total = total + pointx2[i].z;
    }
    /*geometry_msgs::Point32からsensor_msgs::PointCloudに変換
    ここでは, 路面下部の点群データを処理*/
    o_u_pc2.points.resize(o_u_count);
    for (int i = 0; i < o_u_count; i++)
    {
        o_u_pc2.points[i].x = o_u_pointx[i].x;
        o_u_pc2.points[i].y = o_u_pointx[i].y;
        o_u_pc2.points[i].z = o_u_pointx[i].z;
    }
    /*geometry_msgs::Point32からsensor_msgs::PointCloudに変換
    ここでは, ポットホールの点群データを処理*/
    o_u_pc3.points.resize(o_u_count2);
    for (int i = 0; i < o_u_count2; i++)
    {
        o_u_pc3.points[i].x = o_u_pointx2[i].x;
        o_u_pc3.points[i].y = o_u_pointx2[i].y;
        o_u_pc3.points[i].z = o_u_pointx2[i].z;
    }
    /*geometry_msgs::Point32からsensor_msgs::PointCloudに変換
    ここでは, 路面上部の点群データを処理*/
    o_o_pc2.points.resize(o_o_count);
    for (int i = 0; i < o_o_count; i++)
    {
        o_o_pc2.points[i].x = o_o_pointx[i].x;
        o_o_pc2.points[i].y = o_o_pointx[i].y;
        o_o_pc2.points[i].z = o_o_pointx[i].z;
    }
    /*geometry_msgs::Point32からsensor_msgs::PointCloudに変換
    ここでは, ハンプの点群データを処理*/
    o_o_pc3.points.resize(o_o_count2);
    for (int i = 0; i < o_o_count2; i++)
    {
        o_o_pc3.points[i].x = o_o_pointx2[i].x;
        o_o_pc3.points[i].y = o_o_pointx2[i].y;
        o_o_pc3.points[i].z = o_o_pointx2[i].z;
    }

    ave = total / count;
    //反射強度の平均を計算
    ave_intensity = ave_intensity / count;

    //反射強度の分散を計算
    for (int i = 0; i < count; i++)
    {
        dev_intensity = dev_intensity + std::pow(pc2_intensity.data[i] - ave_intensity, 2);
    }
    if (dispersion != 0 && count != 0 && dev_intensity != 0)
    {
        dev_intensity = dev_intensity / count;
    }

    height.data[0] = dispersion;
    height.data[1] = dispersion / count;
    //反射強度の分散格納
    intensity.data[0] = ave_intensity;
    //反射強度の標準偏差を格納
    intensity.data[1] = sqrt(dev_intensity);

    //verification
    printf("variance %f, ave %f int_ave: %f int_dev: %f\n", dispersion, dispersion / count, ave_intensity, intensity.data[1]);
    //反射強度が閾値以上の場合砂利路面, 閾値以下の時舗装路面と識別
    if (intensity.data[1] > 9.79)
    {
        printf("gravel surface\n");
        intensity.data[2] = 1;
    }
    else
    {
        intensity.data[2] = 0;
    }

    //get ros time
    ros::Time calc_time = ros::Time::now();
    ros::Duration D_Time = calc_time - ros_begin;
    ros::Time c_time = ros::Time::now();
    Ident_Time.data = D_Time;
    o_u_pc2.header.stamp = c_time;
    o_o_pc2.header.stamp = c_time;
    pc3.header.stamp = c_time;

    //convert pointcloud to pointcloud2
    //sensor_msgs::convertPointCloudToPointCloud2(sensor_msgs::PointCloud, sensor_msgs::PointCloud2);
    sensor_msgs::convertPointCloudToPointCloud2(pc3, s_pc3);
    sensor_msgs::convertPointCloudToPointCloud2(o_u_pc2, s_o_u_pc2);
    sensor_msgs::convertPointCloudToPointCloud2(o_o_pc2, s_o_o_pc2);
    sensor_msgs::convertPointCloudToPointCloud2(o_u_pc3, s_o_u_pc3);
    sensor_msgs::convertPointCloudToPointCloud2(o_o_pc3, s_o_o_pc3);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_filter2");
    ros::NodeHandle n;
    height.data.resize(2);
    intensity.data.resize(3);
    //更新周期設定[Hz]
    ros::Rate loop_rate(50);

    /*subscribeノード作成　
	  NodeHandle.subscribe(ノード名, 保存するキュー数, 実行する関数)*/
    ros::Subscriber subp = n.subscribe("change_point", 10, identify_surface);

    /*Publishノード作成
	  ros::Publisher 変数 = NodeHandle.advertise<メッセージ形式>(ノード名, 発信するキュー数)*/
    ros::Publisher disper_pub = n.advertise<std_msgs::Float32>("dispersion2", 10);
    ros::Publisher o_dist_pub = n.advertise<std_msgs::Float32>("o_distance2", 10);
    ros::Publisher u_dist_pub = n.advertise<std_msgs::Float32>("u_distance2", 10);
    ros::Publisher height_pub = n.advertise<std_msgs::Float32MultiArray>("height2", 10);
    ros::Publisher intensity_pub = n.advertise<std_msgs::Float32MultiArray>("intensity2", 10);
    ros::Publisher intensity2_pub = n.advertise<std_msgs::Float32MultiArray>("intensity", 10);
    ros::Publisher point2_pub = n.advertise<sensor_msgs::PointCloud2>("change_point2", 10);
    ros::Publisher obst_under_pub = n.advertise<sensor_msgs::PointCloud2>("obstacle_under", 10);
    ros::Publisher obst_under2_pub = n.advertise<sensor_msgs::PointCloud2>("obstacle_under2", 10);
    ros::Publisher obst_over_pub = n.advertise<sensor_msgs::PointCloud2>("obstacle_over", 10);
    ros::Publisher obst_over2_pub = n.advertise<sensor_msgs::PointCloud2>("obstacle_over2", 10);
    ros::Publisher test_points_pub = n.advertise<sensor_msgs::PointCloud>("test_points2", 10);
    ros::Publisher Identified_Time = n.advertise<std_msgs::Duration>("Iden_time2", 10);
    ros::Publisher Plane_Time = n.advertise<std_msgs::Duration>("Iden_Time_Plane", 10);
    // ros::Publisher grid_pub = n.advertise<jsk_recognition_msgs::SimpleOccupancyGrid>("grid", 10);
    // ros::Publisher grid_pub = n.advertise<jsk_recognition_msgs::SimpleOccupancyGridArray>("gridarray", 10);

    //variable
    std_msgs::Float32 disper_msg, pitch_msg, o_dist_msg, u_dist_msg;

    //send message
    //[ros::Publisherの変数].publish(変数);
    while (ros::ok())
    {
        point2_pub.publish(s_pc3);
        obst_under_pub.publish(s_o_u_pc2);
        obst_over_pub.publish(s_o_o_pc2);
        obst_under2_pub.publish(s_o_u_pc3);
        obst_over2_pub.publish(s_o_o_pc3);
        height_pub.publish(height);
        intensity_pub.publish(intensity);
        intensity2_pub.publish(pc3_intensity);
        Identified_Time.publish(Ident_Time);
        Plane_Time.publish(Ident_Time_Plane);
        test_points_pub.publish(t_points);
        // grid_pub.publish(gridarray);

        //Subscribeにキューがある場合関数を実行
        ros::spinOnce();
        /*次の更新周期になるまで停止
    　例えば更新周期50 Hzの場合20 msごとに実行するが以下の関数まで5 msしかかからない場合15 ms停止する*/
        loop_rate.sleep();
    }

    return 0;
}
