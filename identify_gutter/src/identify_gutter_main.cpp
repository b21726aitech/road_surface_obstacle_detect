/*stl*/
#include <iostream>
#include <time.h>
/*ros*/
#include <ros/ros.h>
/*pcl*/
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
/*tf*/
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
/*geometry_msgs*/
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
/*sensor_msgs*/
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
/*std_msgs*/
#include <std_msgs/Duration.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Time.h>
/*original*/
#include "identify_gutter/PlaneStatus.h"
// ==========================================================================

class IdentifyGutter
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber cloud_sub_;                      // 受け取る点群
    ros::Subscriber raw_cloud_sub_;                  // 受け取る点群
    ros::Publisher points_gutter_inner_edge_L_pub_;  // 道路右端の点群
    ros::Publisher points_gutter_inner_edge_R_pub_;  // 道路左端の点群
    ros::Publisher points_gutter_inner_edge_LR_pub_; // 道路両端の点群(試験的)
    ros::Publisher points_plane_estimate_pub_;       // 平面近似対象の点群
    ros::Publisher plane_status_pub_;                // 平面近似の平面方程式4係数+平面の高さ

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    identify_gutter::PlaneStatus pStatus_;
    float distance_limit_msac_;
    float distance_limit_gutter_;

    std::string base_frame_;                        // TF先frame
    std::string sub_topic_;                         // Subscribeするトピック名
    std::string sub_allpoints_topic_;               // Subscribeするトピック名
    std::string points_gutter_inner_edge_L_topic_;  // Publishするトピック名
    std::string points_gutter_inner_edge_R_topic_;  // Publishするトピック名
    std::string points_gutter_inner_edge_LR_topic_; // Publishするトピック名(試験的)
    std::string points_plane_estimate_topic_;       // Publishするトピック名
    std::string plane_status_topic_;                // Publishするトピック名

    float threshold_road_; // 0.03
    float threshold_obj_;  // 0.13
    float plane_[4] = {0}; // MSAC結果の平面方程式 ax+by+cz+d+0 の係数 a,b,c,d
    float calc_height_;    // 平面からの高さ

    /**
    * msgのPublish
    * @param pub ros::Publisher
    * @param in_sensor_cloud Publishする点を選択するための入力点群
    * @param in_selector 点群のポインタ
    */
    inline void publishMsg(ros::Publisher pub,
                           const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud,
                           const std::vector<void *> &in_selector);

    /**
    * in_selectorで指定した点をin_origin_cloudから抽出、out_no_ground_ptrsに出力
    * @param in_origin_cloud 入力点群
    * @param in_selector 選択された点のアドレス
    * @param out_filtered_msg 出力点群
    */
    inline void filterROSMsg(const sensor_msgs::PointCloud2ConstPtr in_origin_cloud,
                             const std::vector<void *> &in_selector,
                             const sensor_msgs::PointCloud2::Ptr out_filtered_msg);

    /**
     * 平面近似
     * @param in_sensor_cloud 入力点群
     */
    inline void estimatePlane(const sensor_msgs::PointCloud2 in_sensor_cloud);

    /**
    * 側溝端の点の抽出
    * @param in_cloud 入力点群
    * @param out_cloud_L 道路左端の点群
    * @param out_cloud_R 道路右端の点群
    */
    inline void extractGutterEdge(sensor_msgs::PointCloud2 in_cloud,
                                  sensor_msgs::PointCloud2 out_cloud_L,
                                  sensor_msgs::PointCloud2 out_cloud_R, sensor_msgs::PointCloud2 out_cloud_LR);

    /**
     * 点群のTF実行
     * @param in_target_frame TF先Frame
     * @param in_cloud_ptr 入力点群
     * @param out_cloud_ptr 出力点群
     * @retval true: TF成功
     * @retval false: TF失敗
     */
    inline bool transformPointCloud(const std::string &in_target_frame,
                                    const sensor_msgs::PointCloud2::ConstPtr &in_cloud_ptr,
                                    const sensor_msgs::PointCloud2::Ptr &out_cloud_ptr);

    /**
     * points_groundのSubscribe時
     * @param in_sensor_cloud sensor_msgs::PointCloud2ConstPtr
     */
    void gutterCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud);

    /**
     * points_rawのSubscribe時
     * @param in_sensor_cloud sensor_msgs::PointCloud2ConstPtr
     */
    void pointsRawCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud);

public:
    IdentifyGutter(); // コンストラクタ
    void Run();       // 実行時関数
};

// ==========================================================================

// Global変数
std_msgs::Duration Ident_Time, Ident_Time_Plane;

// inline void IdentifyGutter::publishMsg(ros::Publisher pub,
//                                        const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud,
//                                        const std::vector<void *> &in_selector)
// {
//     sensor_msgs::PointCloud2::Ptr output_cloud(new sensor_msgs::PointCloud2); // 出力用の点群
//     filterROSMsg(in_sensor_cloud, in_selector, output_cloud);                 // フィルタリング実行
//     pub.publish(*output_cloud);                                               // Publish実行
// }

// inline void IdentifyGutter::filterROSMsg(const sensor_msgs::PointCloud2ConstPtr in_origin_cloud,
//                                          const std::vector<void *> &in_selector,
//                                          const sensor_msgs::PointCloud2::Ptr out_filtered_msg)
// {
//     size_t point_size = in_origin_cloud->row_step / in_origin_cloud->width; // Byte単位
//     size_t data_size = point_size * in_selector.size();                     // ポインタで指定した点群のデータサイズ
//     out_filtered_msg->data.resize(data_size);                               // 出力する点群のサイズ決定
//     size_t offset = 0;
//     for (auto it=in_selector.cbegin();it!=in_selector.cend();it++)
//     {
//         memcpy(out_filtered_msg->data.data() + offset, *it, point_size); // memcpy(コピー先のメモリブロック, コピー元のメモリブロック, コピーByte数)
//         offset += point_size;                                            // アドレスシフト
//     }
//     /* 点群の2次元構造 */ // 点群が順番に並んでいない場合、widthは点群の長さ、heightは1となる
//     out_filtered_msg->width = (uint32_t)in_selector.size();
//     out_filtered_msg->height = 1;
//     /* msgの設定 */
//     out_filtered_msg->fields = in_origin_cloud->fields;                                                           // バイナリデータブロブ内のチャネルとそのレイアウトについて説明
//     out_filtered_msg->header.frame_id = base_frame_;                                                              // frame id
//     out_filtered_msg->header.stamp = in_origin_cloud->header.stamp;                                               // time stamp
//     out_filtered_msg->point_step = in_origin_cloud->point_step;                                                   // 点の長さ(Byte)
//     out_filtered_msg->row_step = point_size * in_selector.size();                                                 // 行の長さ(Byte)
//     out_filtered_msg->is_dense = in_origin_cloud->is_dense && in_origin_cloud->data.size() == in_selector.size(); // 無効なポイントがない場合は真
//     std::cout << "out_filtered_msg->header.stamp=" << out_filtered_msg->header.stamp << std::endl;
// }

inline void IdentifyGutter::extractGutterEdge(sensor_msgs::PointCloud2 in_cloud,
                                              sensor_msgs::PointCloud2 out_cloud_L,
                                              sensor_msgs::PointCloud2 out_cloud_R, sensor_msgs::PointCloud2 out_cloud_LR)
{
    /* PointCloud2 から PointCloud へ型変換 */
    sensor_msgs::PointCloud pc_1;
    sensor_msgs::convertPointCloud2ToPointCloud(in_cloud, pc_1); // PointCloud2 から PointCloud へ変換

    /* 道路端点群抽出 */
    float distance = 0; //distance from velodyne to point data
    int count_L = 0;
    int count_R = 0;
    bool isUnder = false;
    const int pc_1_size = pc_1.points.size();
    geometry_msgs::Point32 geoPoint32_L[pc_1_size];
    geometry_msgs::Point32 geoPoint32_R[pc_1_size];

    // std::cout << pc_1_size << std::endl;

    for (int i = 0; i < pc_1_size; i++)
    {
        // calc_height_ = (plane_[0] * pc_1.points[i].x + plane_[1] * pc_1.points[i].y + plane_[2] * pc_1.points[i].z + plane_[3]) / sqrt(pow(plane_[0], 2) + pow(plane_[1], 2) + pow(plane_[2], 2));
        calc_height_ = (-1 * (plane_[0] * pc_1.points[i].x + plane_[1] * pc_1.points[i].y + plane_[3])) / plane_[2]; // 路面高さ

        // if (-threshold_road_ <= pc_1.points[i].z - calc_height_ && pc_1.points[i].z - calc_height_ <= threshold_road_)
        if (calc_height_ - threshold_road_ <= pc_1.points[i].z && pc_1.points[i].z <= calc_height_ + threshold_road_) // 路面範囲内の場合
        {
            if (isUnder) // 一つ前の点が路面下の場合
            {
                geoPoint32_R[count_R].x = pc_1.points[i].x;
                geoPoint32_R[count_R].y = pc_1.points[i].y;
                geoPoint32_R[count_R].z = pc_1.points[i].z;
                count_R++;
            }
            isUnder = false; // 現在の点は路面
        }
        // else if (pc_1.points[i].z - calc_height_ < -threshold_road_)
        else if (pc_1.points[i].z < calc_height_ - threshold_road_) // 路面範囲より低い場合
        {
            if (!isUnder) // 一つ前の点が路面の場合
            {
                if (i - 1 >= 0)
                {
                    geoPoint32_L[count_L].x = pc_1.points[i - 1].x;
                    geoPoint32_L[count_L].y = pc_1.points[i - 1].y;
                    geoPoint32_L[count_L].z = pc_1.points[i - 1].z;
                    count_L++;
                }
            }
            isUnder = true; // 現在の点は路面下
        }
    }

    pStatus_.header.stamp = ros::Time::now();
    pStatus_.a = plane_[0];
    pStatus_.b = plane_[1];
    pStatus_.c = plane_[2];
    pStatus_.d = plane_[3];
    // pStatus_.height = calc_height_;
    // ROS_INFO("plane height: %5.3f", pStatus_.height);
    plane_status_pub_.publish(pStatus_);

    /* geometry_msgs::Point32 から sensor_msgs::PointCloud へ型変換 */
    sensor_msgs::PointCloud pc_2_L;
    sensor_msgs::PointCloud pc_2_R;
    sensor_msgs::PointCloud pc_2_LR;
    pc_2_L.header = pc_1.header;
    pc_2_R.header = pc_1.header;
    pc_2_LR.header = pc_1.header;
    pc_2_L.points.resize(count_L);
    pc_2_R.points.resize(count_R);
    pc_2_LR.points.resize(count_L + count_R);
    // pc_2_L.channels[0].values.resize(count_L);
    // pc_2_R.channels[0].values.resize(count_R);

    int countLR = 0;
    for (int i = 0; i < count_L; i++)
    {
        distance = sqrt(pow(geoPoint32_L[i].x, 2) + pow(geoPoint32_L[i].y, 2) + pow(geoPoint32_L[i].z, 2));
        if (distance < distance_limit_gutter_)
        {

            pc_2_L.points[i].x = geoPoint32_L[i].x;
            pc_2_L.points[i].y = geoPoint32_L[i].y;
            pc_2_L.points[i].z = geoPoint32_L[i].z;

            pc_2_LR.points[countLR].x = geoPoint32_L[i].x;
            pc_2_LR.points[countLR].y = geoPoint32_L[i].y;
            pc_2_LR.points[countLR].z = geoPoint32_L[i].z;
            countLR++;
        }
    }
    for (int i = 0; i < count_R; i++)
    {
        distance = sqrt(pow(geoPoint32_R[i].x, 2) + pow(geoPoint32_R[i].y, 2) + pow(geoPoint32_R[i].z, 2));
        if (distance < distance_limit_gutter_)
        {

            pc_2_R.points[i].x = geoPoint32_R[i].x;
            pc_2_R.points[i].y = geoPoint32_R[i].y;
            pc_2_R.points[i].z = geoPoint32_R[i].z;

            pc_2_LR.points[countLR].x = geoPoint32_R[i].x;
            pc_2_LR.points[countLR].y = geoPoint32_R[i].y;
            pc_2_LR.points[countLR].z = geoPoint32_R[i].z;
            countLR++;
        }
    }
    pc_2_L.header.stamp = pc_2_R.header.stamp = pc_2_LR.header.stamp = ros::Time::now();

    /* PointCloud から PointCloud2 へ型変換 */
    sensor_msgs::convertPointCloudToPointCloud2(pc_2_L, out_cloud_L);
    sensor_msgs::convertPointCloudToPointCloud2(pc_2_R, out_cloud_R);
    sensor_msgs::convertPointCloudToPointCloud2(pc_2_LR, out_cloud_LR);

    /* Publish実行 */
    points_gutter_inner_edge_L_pub_.publish(out_cloud_L);
    points_gutter_inner_edge_R_pub_.publish(out_cloud_R);
    points_gutter_inner_edge_LR_pub_.publish(out_cloud_LR);
}

inline void IdentifyGutter::estimatePlane(const sensor_msgs::PointCloud2 in_sensor_cloud)
{
    /* 平面近似のための変換処理 */
    pcl::PointCloud<pcl::PointXYZ> in_sensor_cloud_pcl;
    pcl::fromROSMsg(in_sensor_cloud, in_sensor_cloud_pcl); // sensor_msgs::PointCloud2 を pcl::PointCloud<pcl::PointXYZ> に変換
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_sensor_cloud_pcl_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    *in_sensor_cloud_pcl_ptr = in_sensor_cloud_pcl; // 平面近似でポインタを用いるためPtrに変換

    ros::Time time_planer_begin_ = ros::Time::now(); // 平面近似の開始時間

    /* MSACを用いた平面モデル抽出 */
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); // 抽出結果保持用モデル
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);                // モデル抽出時に使用された点群のインデックス(indeices)を保持する変数のポインタ
    pcl::SACSegmentation<pcl::PointXYZ> seg;                              // segmentationオブジェクトの生成
    seg.setOptimizeCoefficients(true);                                    // RANSACにおいて最適化を実施するかどうか(true:最適化する)
    seg.setModelType(pcl::SACMODEL_PLANE);                                // 抽出モデル: Plane
    seg.setMethodType(pcl::SAC_MSAC);                                     // 抽出メソッド: MSAC
    seg.setDistanceThreshold(0.03);                                       // 許容する誤差しきい値(VLP-16の誤差が0.03 mのため)
    seg.setInputCloud(in_sensor_cloud_pcl_ptr);                           // モデル抽出対象点群のセット
    seg.segment(*inliers, *coefficients);                                 // モデル抽出の実行

    ros::Time time_planer_end_ = ros::Time::now(); // 平面近似の終了時間

    /* 抽出結果モデルの表示 */
    plane_[0] = coefficients->values[0]; // a
    plane_[1] = coefficients->values[1]; // b
    plane_[2] = coefficients->values[2]; // c
    plane_[3] = coefficients->values[3]; // d
    ROS_INFO("coefficients:[a,b,c,d]=[%5.3lf,%5.3lf,%5.3lf,%5.3lf]", plane_[0], plane_[1], plane_[2], plane_[3]);

    /* 処理時間計測 */
    ros::Duration D_Times = time_planer_end_ - time_planer_begin_;
    Ident_Time_Plane.data = D_Times;
}

inline bool IdentifyGutter::transformPointCloud(const std::string &in_target_frame,
                                                const sensor_msgs::PointCloud2::ConstPtr &in_cloud_ptr,
                                                const sensor_msgs::PointCloud2::Ptr &out_cloud_ptr)
{
    if (in_target_frame == in_cloud_ptr->header.frame_id)
    {
        *out_cloud_ptr = *in_cloud_ptr;
        return true;
    }

    geometry_msgs::TransformStamped transform_stamped;
    try
    {
        /** フレームIDで2つのフレーム間の変換を取得
         * geometry_msgs::TransformStamped tf2_ros::Buffer::lookupTransform(const std::string &target_frame,
         *                                                                  const std::string &source_frame, 
         *                                                                  const ros::Time &time, 
         *                                                                  const ros::Duration timeout);
         * @param in_target_frame データの変換先のフレーム
         * @param source_frame データが発生したフレーム
         * @param time 変換の値が必要な時間(0が最新)
         * @param timeout 失敗する前にブロックする時間
         * @retval geometry_msgs::TransformStamped フレーム間の変換
         */
        transform_stamped = tf_buffer_.lookupTransform(in_target_frame,
                                                       in_cloud_ptr->header.frame_id,
                                                       in_cloud_ptr->header.stamp,
                                                       ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return false;
    }
    // tf2::doTransform(*in_cloud_ptr, *out_cloud_ptr, transform_stamped);
    Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
    pcl_ros::transformPointCloud(mat, *in_cloud_ptr, *out_cloud_ptr);
    out_cloud_ptr->header.frame_id = in_target_frame;
    return true;
}

void IdentifyGutter::gutterCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud)
{
    /* 入力された点群のTF */
    sensor_msgs::PointCloud2::Ptr trans_sensor_cloud(new sensor_msgs::PointCloud2);                               // TF用点群の宣言
    const bool succeeded = IdentifyGutter::transformPointCloud(base_frame_, in_sensor_cloud, trans_sensor_cloud); // TF実行
    if (!succeeded)                                                                                               // TF失敗の場合は以下を実行
    {
        ROS_ERROR_STREAM_THROTTLE(10, "Failed transform from " << base_frame_ << " to " << in_sensor_cloud->header.frame_id);
        return;
    }

    int count = 0;      //calced ground point size
    float distance = 0; //distance from velodyne to point data
    sensor_msgs::PointCloud cloud, cloud2;
    sensor_msgs::PointCloud2::Ptr near_cloud(new sensor_msgs::PointCloud2);
    std_msgs::Float32MultiArray cloud_intensity;
    sensor_msgs::convertPointCloud2ToPointCloud(*trans_sensor_cloud, cloud);
    geometry_msgs::Point32 pointx[cloud.points.size()];
    cloud_intensity.data.resize(cloud.channels[0].values.size());
    cloud2.channels.resize(1);
    cloud2.header = cloud.header;
    cloud2.channels[0].name = cloud.channels[0].name;

    for (int i = 0; i < cloud.points.size(); i++)
    {
        //距離計算
        distance = sqrt(pow(cloud.points[i].x, 2) + pow(cloud.points[i].y, 2) + pow(cloud.points[i].z, 2));
        if (distance < distance_limit_msac_)
        {
            pointx[count].x = cloud.points[i].x;
            pointx[count].y = cloud.points[i].y;
            pointx[count].z = cloud.points[i].z;
            cloud_intensity.data[count] = cloud.channels[0].values[i];
            count++;
        }
    }
    cloud2.points.resize(count);
    cloud2.channels[0].values.resize(count);
    for (int i = 0; i < count; i++)
    {
        cloud2.points[i].x = pointx[i].x;
        cloud2.points[i].y = pointx[i].y;
        cloud2.points[i].z = pointx[i].z;
        cloud2.channels[0].values[i] = cloud_intensity.data[i];
    }
    sensor_msgs::convertPointCloudToPointCloud2(cloud2, *near_cloud);

    /* 平面近似 */
    estimatePlane(*near_cloud); // 平面近似実行
    points_plane_estimate_pub_.publish(*near_cloud);

    // /* 側溝端の点群を抽出 */
    // sensor_msgs::PointCloud2 gutter_cloud_L; // 道路左端の点群
    // sensor_msgs::PointCloud2 gutter_cloud_R; // 道路右端の点群
    // extractGutterEdge(*trans_sensor_cloud, gutter_cloud_L, gutter_cloud_R);

    // /* Publish実行 */
    // TODO: 間接参照に修正して高速化
    // std::vector<void *> gutter_L_ptrs; // 道路左端の点のポインタ
    // std::vector<void *> gutter_R_ptrs; // 道路右端の点のポインタ
    // publishMsg(points_gutter_inner_edge_L_pub_, in_sensor_cloud, gutter_L_ptrs);
    // publishMsg(points_gutter_inner_edge_R_pub_, in_sensor_cloud, gutter_R_ptrs);

    // points_gutter_inner_edge_L_pub_.publish(gutter_cloud_L);
    // points_gutter_inner_edge_R_pub_.publish(gutter_cloud_R);
}

void IdentifyGutter::pointsRawCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud)
{
    /* 入力された点群のTF */
    sensor_msgs::PointCloud2::Ptr trans_sensor_cloud(new sensor_msgs::PointCloud2);                               // TF用点群の宣言
    const bool succeeded = IdentifyGutter::transformPointCloud(base_frame_, in_sensor_cloud, trans_sensor_cloud); // TF実行
    if (!succeeded)                                                                                               // TF失敗の場合は以下を実行
    {
        ROS_ERROR_STREAM_THROTTLE(10, "Failed transform from " << base_frame_ << " to " << in_sensor_cloud->header.frame_id);
        return;
    }

    /* 側溝端の点群を抽出 */
    sensor_msgs::PointCloud2 gutter_cloud_L;  // 道路右端の点群
    sensor_msgs::PointCloud2 gutter_cloud_R;  // 道路左端の点群
    sensor_msgs::PointCloud2 gutter_cloud_LR; // 道路両端の点群
    extractGutterEdge(*trans_sensor_cloud, gutter_cloud_L, gutter_cloud_R, gutter_cloud_LR);
}

IdentifyGutter::IdentifyGutter() : nh_(), pnh_("~"), tf_listener_(tf_buffer_)
{
    /* Initialize */
    base_frame_ = "velodyne";                                           // TF先frame
    sub_topic_ = "points_ground";                                       // Subscribeするトピック名
    sub_allpoints_topic_ = "points_raw";                                // Subscribeするトピック名
    points_gutter_inner_edge_L_topic_ = "points_gutter_inner_edge_L";   // Publishするトピック名
    points_gutter_inner_edge_R_topic_ = "points_gutter_inner_edge_R";   // Publishするトピック名
    points_gutter_inner_edge_LR_topic_ = "points_gutter_inner_edge_LR"; // Publishするトピック名
    points_plane_estimate_topic_ = "points_plane_estimate";             // Publishするトピック名
    plane_status_topic_ = "plane_status";                               // Publishするトピック名
    this->threshold_road_ = 0.03;                                       // 路面閾値
    this->threshold_obj_ = 0.13;                                        // 路面障害物閾値
    this->calc_height_ = 0;                                             // 平面からの高さ
    // this->plane_[4] = {0, 0, 0, 0};                                   // MSAC結果の平面方程式 ax+by+cz+d=0 の係数 a,b,c,d
    this->distance_limit_msac_ = 8.0;    //MSACに用いる点群の半径範囲
    this->distance_limit_gutter_ = 25.0; //側溝として識別する点群の半径範囲

    this->pStatus_.a = 0;
    this->pStatus_.b = 0;
    this->pStatus_.c = 0;
    this->pStatus_.d = 0;
    // pStatus_.height = 0;
}

void IdentifyGutter::Run()
{
    /* Show info */
    ROS_INFO("Initializing 'identify_gutter', please wait...");
    ROS_INFO("Frame: %s", base_frame_.c_str());
    ROS_INFO("Input topic: %s", sub_topic_.c_str());
    ROS_INFO("Input topic: %s", sub_allpoints_topic_.c_str());
    ROS_INFO("Output topic: %s", points_gutter_inner_edge_L_topic_.c_str());
    ROS_INFO("Output topic: %s", points_gutter_inner_edge_R_topic_.c_str());
    ROS_INFO("Output topic: %s", points_gutter_inner_edge_LR_topic_.c_str());
    ROS_INFO("Output topic: %s", points_plane_estimate_topic_.c_str());
    ROS_INFO("Output topic: %s", plane_status_topic_.c_str());

    /* Subscriber */
    cloud_sub_ = nh_.subscribe(sub_topic_, 10, &IdentifyGutter::gutterCallback, this);
    raw_cloud_sub_ = nh_.subscribe(sub_allpoints_topic_, 10, &IdentifyGutter::pointsRawCallback, this);

    /* Publisher */
    points_gutter_inner_edge_L_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(points_gutter_inner_edge_L_topic_, 10);
    points_gutter_inner_edge_R_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(points_gutter_inner_edge_R_topic_, 10);
    points_gutter_inner_edge_LR_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(points_gutter_inner_edge_LR_topic_, 10);
    points_plane_estimate_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(points_plane_estimate_topic_, 10);
    plane_status_pub_ = nh_.advertise<identify_gutter::PlaneStatus>(plane_status_topic_, 1000);

    ROS_INFO("Ready.");

    /* Subscribe受付 */
    ros::spin();
    // ros::Rate loop_rate(10);
    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
}

// ==========================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "identify_gutter");
    IdentifyGutter app;
    app.Run();
    return 0;
}
