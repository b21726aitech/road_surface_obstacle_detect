#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h> // sensor_msgs::convertPointCloud2ToPointCloudを使用するためのヘッダ
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h> // pcl::toROSMsg, pcl::fromROSMsgを使用するためのヘッダ
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class ClusterTrackGutter
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber tracked_gutter_sub_;

    ros::Publisher cluster_track_gutter_pub_;
    ros::Publisher cluster_time_track_gutter_pub_;
    ros::Publisher distance_clustertrack_gutter_pub_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>};
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    sensor_msgs::PointCloud2 cluster_track_gutter_;
    sensor_msgs::PointCloud pc;
    pcl::PointCloud<pcl::PointXYZ> clouds;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr2{new pcl::PointCloud<pcl::PointXYZ>};
    int check_count;
    std_msgs::Duration o_cluster_Time, all_time;
    std_msgs::Float32MultiArray o_distance;
    tf::StampedTransform transform, transform2; //read base link to velodyne and map to base link
    double time_start;
    ros::Time ros_begins;

    /*parameters*/
    double cluster_tolerance;
    int min_cluster_size;

public:
    ClusterTrackGutter();
    void callbackClusterTrackGutter(const sensor_msgs::PointCloud2ConstPtr &msg);
    void Clustering(void);
};

/**
 * クラスタリング実行関数 */
void ClusterTrackGutter::Clustering(void)
{
    ros_begins = ros::Time::now();
    time_start = ros::Time::now().toSec();

    float tf_x = transform.getOrigin().x();
    float tf_y = transform.getOrigin().y();
    float tf_z = transform.getOrigin().z();
    float yaw = tf::getYaw(transform.getRotation());
    float tf2_x = transform2.getOrigin().x();
    float tf2_y = transform2.getOrigin().y();
    float tf2_z = transform2.getOrigin().z();

    /*clustering*/
    /*kd-treeクラスを宣言*/
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    /*クラスタリング後の点群データが格納されるベクトル*/
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
    if (cloud->points.size() != 0)
    {
        /*探索する点群をinput*/
        tree->setInputCloud(cloud);
        /*距離の閾値を設定*/
        ece.setClusterTolerance(cluster_tolerance);
        /*各クラスタのメンバの最小数を設定*/
        ece.setMinClusterSize(min_cluster_size);
        /*各クラスタのメンバの最大数を設定*/
        ece.setMaxClusterSize(cloud->points.size());
        /*探索方法を設定*/
        ece.setSearchMethod(tree);
        /*クラスリング対象の点群をinput*/
        ece.setInputCloud(cloud);
        /*クラスリング実行*/
        ece.extract(cluster_indices);
    }

    std::cout << "cluster_indices.size() = " << cluster_indices.size() << std::endl;

    /*クラスタリングごとに点群を分割*/
    pcl::ExtractIndices<pcl::PointXYZ> ei;

    o_distance.data.resize(cluster_indices.size() + 1);
    o_distance.data[0] = cluster_indices.size();

    *cloud_ptr = *cloud_ptr2;

    ei.setInputCloud(cloud);
    ei.setNegative(false);
    for (size_t i = 0; i < cluster_indices.size(); i++)
    {
        /*extract*/
        // 分割した点群データを保存する変数を設定
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_clustered_points(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointIndices::Ptr tmp_clustered_indices(new pcl::PointIndices);
        *tmp_clustered_indices = cluster_indices[i];
        ei.setIndices(tmp_clustered_indices);
        ei.filter(*tmp_clustered_points);
        clusters.push_back(tmp_clustered_points);
        /*input*/
        pcl::PointCloud<pcl::PointXYZ> cloud_check;
        cloud_check = *tmp_clustered_points;

        pcl::toROSMsg(cloud_check, cluster_track_gutter_);                      // pcl::PointCloud<pcl::PointXYZ> -> sensor_msgs::PointCloud2
        sensor_msgs::convertPointCloud2ToPointCloud(cluster_track_gutter_, pc); // sensor_msgs::PointCloud2 -> sensor_msgs::PointCloud

        //障害物の面積と距離を取得するための変数宣言
        float min_y = 0, max_y = 0, min_x = 0, max_x = 0, dist = 0, ave_z = 0, ave2_z = 0, m_y = 0, m_z = 0, x = 0, y = 0, z = 0, o_d = 30.0;

        /*クラスタリングの面積と距離の計算*/
        for (int j = 0; j < pc.points.size(); j++)
        {
            if (j == 0)
            {
                //初期値設定
                min_y = pc.points[j].y;
                min_x = pc.points[j].x;
                max_y = pc.points[j].y;
                max_x = pc.points[j].x;
                ave_z = pc.points[j].z;
                ave2_z = pc.points[j].z;
            }
            /*車両距離方向の最小値の点データを取得*/
            if (min_x > pc.points[j].x)
            {
                min_x = pc.points[j].x;
                m_y = pc.points[j].y;
                m_z = pc.points[j].z;
                /*一応最大も取得*/
            }
            else if (max_x < pc.points[j].x)
            {
                max_x = pc.points[j].x;
            }
            /*車両左右方向の最小値の点データを取得*/
            if (min_y > pc.points[j].y)
            {
                min_y = pc.points[j].y;
                /*一応最大も取得*/
            }
            else if (max_y < pc.points[j].y)
            {
                max_y = pc.points[j].y;
            }
            /*高さ方向の最小値の点データを取得*/
            if (ave_z > pc.points[j].z)
            {
                ave_z = pc.points[j].z;
                /*一応最大も取得*/
            }
            else if (ave2_z < pc.points[j].z)
            {
                ave2_z = pc.points[j].z;
            }
        }

        /*なくても良い*/
        dist = sqrt(pow(max_y - min_y, 2) + pow(max_x - min_x, 2));

        /*velodyne座標系からmap座標系へ座標変換　トラッキングの結果はmap座標系で保存*/
        x = tf_x + (min_x * cos(yaw) + m_y * sin(yaw));
        y = tf_y + (-min_x * sin(yaw) + m_y * cos(yaw));
        z = tf_z + m_z;
        ave_z = tf_z - ave_z;
        ave2_z = tf_z - ave2_z;

        /* 側溝と車両との距離を計算 */
        o_distance.data[i + 1] = sqrt(pow(y - tf2_y, 2) + pow(x - tf2_x, 2));
        printf("dist: %f distance: %f x_dist: %f y_dist: %f z_dist: %f\n", dist, o_distance.data[i + 1], std::abs(max_x - min_x), std::abs(max_y - min_y), std::abs(ave_z - ave2_z));

        *cloud_ptr = *cloud_ptr + *tmp_clustered_points; // クラスタリングの結果をひとつの変数に格納 今後, ある条件でまとめたくない時にif分内に分岐する変数を入れると良い
    }

    clouds = *cloud_ptr;                           // pcl::PointCloud<pcl::PointXYZ>::Ptr -> pcl::PointCloud<pcl::PointXYZ>
    pcl::toROSMsg(clouds, cluster_track_gutter_);  // pcl::PointCloud<pcl::PointXYZ> -> sensor_msgs::PointCloud2
    cluster_track_gutter_.header.frame_id = "map"; // TFの座標系を決定　トラッキング結果はmap座標系に置くことで車両が移動しても変化しないためmap座標系で保存

    // 処理時間の計測
    ros::Time calc_time = ros::Time::now();
    ros::Duration D_Times = calc_time - ros_begins;

    o_cluster_Time.data = D_Times;

    std::cout << "clustering time [s] = " << ros::Time::now().toSec() - time_start << std::endl;
}

/**
 * コールバック関数
 * @param msg SubscribeしたPointCloud2型の点群 */
void ClusterTrackGutter::callbackClusterTrackGutter(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // 路面上部の障害物のクラスタリング設定
    pcl::fromROSMsg(*msg, *cloud); // sensor_msgs::PointCloud2 -> pcl::PointCloud<pcl::PointXYZ>

    std::cout << "=====street gutter=====" << std::endl;
    std::cout << "cloud->points.size() = " << cloud->points.size() << std::endl;

    check_count = cloud->points.size();

    pnh_.param("cluster_tolerance", cluster_tolerance, 0.3); // クラスタとなる距離を設定[m]

    clusters.clear();
    Clustering();
}

/**
 * コンストラクタ */
ClusterTrackGutter::ClusterTrackGutter() : pnh_("~")
{
    tracked_gutter_sub_ = nh_.subscribe("tracked_gutter", 1, &ClusterTrackGutter::callbackClusterTrackGutter, this);

    cluster_track_gutter_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cluster_track_gutter", 10);
    cluster_time_track_gutter_pub_ = nh_.advertise<std_msgs::Duration>("cluster_time_track_gutter", 1000);
    distance_cluster_track_gutter_pub = nh_.advertise<std_msgs::Float32MultiArray>("distance_cluster_track_gutter", 1000);

    pnh_.param("min_cluster_size", min_cluster_size, 2); // クラスタリングとなる点データ数の最小数を設定

    std::cout << "cluster_tolerance = " << cluster_tolerance << std::endl;
    std::cout << "min_cluster_size = " << min_cluster_size << std::endl;

    //ros::AsyncSpinner spinner(2);
    //spinner.start();

    ros::Rate loop_rate(50);
    tf::TransformListener listener, listener2;
    while (ros::ok())
    {
        //get coordinate from map to baselink and get Coordinate base link to velodyne
        while (true)
        {
            try
            {
                ROS_INFO("Transforming...");
                listener.lookupTransform("velodyne", "map", ros::Time(0), transform);         // 座標系間の位置を取得　velodyne座標系外にあるmap座標系の位置情報を取得する
                listener2.lookupTransform("base_link", "velodyne", ros::Time(0), transform2); // 座標系間の位置を取得　base_link座標系内にあるvelodyne座標系の位置情報を取得する
                ROS_INFO("Transformed!");
                break;
            }
            catch (tf::TransformException ex)
            {
                ROS_INFO("TransformException");
                ros::Duration(1.0).sleep();
            }
        }

        cluster_track_gutter_pub_.publish(cluster_track_gutter_); // クラスタリングした点群
        distance_cluster_track_gutter_pub.publish(o_distance);    // 識別距離
        cluster_time_track_gutter_pub_.publish(o_cluster_Time);   // クラスタリング時間

        ros::spinOnce();
        loop_rate.sleep();
    }
}

/**
 * main */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cluster_track_gutter");
    ClusterTrackGutter app;
    //ros::spin();
}
