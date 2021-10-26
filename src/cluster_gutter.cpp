#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h> //sensor_msgs::convertPointCloud2ToPointCloudを使用するためのヘッダ
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h> //pcl::toROSMsg, pcl::fromROSMsgを使用するためのヘッダ
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class ClusterGutter
{
private:
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	ros::Subscriber gutter_L_sub_, gutter_R_sub_, gutter_LR_sub_;

	ros::Publisher clustered_gutter_pub_;	 // 側溝のクラスタリング結果
	ros::Publisher distance_gutter_pub_;	 // 側溝の識別距離
	ros::Publisher cluster_time_gutter_pub_; // 側溝の識別時間

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>}; // クラスタリングの入力点群
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
	sensor_msgs::PointCloud2 cluster_gutter_;
	sensor_msgs::PointCloud pc;
	pcl::PointCloud<pcl::PointXYZ> clouds;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr{new pcl::PointCloud<pcl::PointXYZ>};
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr2{new pcl::PointCloud<pcl::PointXYZ>};
	int check_count;
	std_msgs::Float32MultiArray value_distance_gutter_;
	std_msgs::Duration value_cluster_time_gutter_;
	tf::StampedTransform transform_; //read base link to velodyne and map to base link

	double cluster_tolerance_;
	int min_cluster_size_;

public:
	ClusterGutter();
	void callbackClusterGutter(const sensor_msgs::PointCloud2ConstPtr &msg);
	void Clustering(void);
};

/**
 * クラスタリング実行関数 */
void ClusterGutter::Clustering(void)
{
	// 時間計測
	ros::Time time_start_clustering = ros::Time::now();

	float tf_x = transform_.getOrigin().x();
	float tf_y = transform_.getOrigin().y();
	float tf_z = transform_.getOrigin().z();

	/*clustering*/
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>); // kd-treeクラスを宣言
	std::vector<pcl::PointIndices> cluster_indices;										  // クラスタリング後の点群データが格納されるベクトル
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
	if (cloud->points.size() != 0)
	{
		tree->setInputCloud(cloud);					 // 探索する点群を入力
		ece.setClusterTolerance(cluster_tolerance_); // 距離の閾値
		ece.setMinClusterSize(min_cluster_size_);	 // 各クラスタのメンバの最小数
		ece.setMaxClusterSize(cloud->points.size()); // 各クラスタのメンバの最大数
		ece.setSearchMethod(tree);					 // 探索方法
		ece.setInputCloud(cloud);					 // クラスリング対象の点群を入力
		ece.extract(cluster_indices);				 // クラスリング実行
	}

	std::cout << "cluster_indices.size() = " << cluster_indices.size() << std::endl;

	/*クラスタリングごとに点群を分割*/
	pcl::ExtractIndices<pcl::PointXYZ> ei;
	ei.setInputCloud(cloud);
	ei.setNegative(false);

	value_distance_gutter_.data.resize(cluster_indices.size() + 1);
	value_distance_gutter_.data[0] = cluster_indices.size();

	*cloud_ptr = *cloud_ptr2;

	for (size_t i = 0; i < cluster_indices.size(); i++)
	{
		/*extract*/
		//分割した点群データを保存する変数を設定
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_clustered_points(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointIndices::Ptr tmp_clustered_indices(new pcl::PointIndices);
		*tmp_clustered_indices = cluster_indices[i];
		ei.setIndices(tmp_clustered_indices);
		ei.filter(*tmp_clustered_points);
		clusters.push_back(tmp_clustered_points);
		/*input*/
		pcl::PointCloud<pcl::PointXYZ> cloud_check;
		cloud_check = *tmp_clustered_points;

		pcl::toROSMsg(cloud_check, cluster_gutter_);					  // pcl::PointCloud<pcl::PointXYZ> -> sensor_msgs::PointCloud2
		sensor_msgs::convertPointCloud2ToPointCloud(cluster_gutter_, pc); // sensor_msgs::PointCloud2 -> sensor_msgs::PointCloud

		// 障害物の面積と距離を取得するための変数宣言
		float min_y, max_y, min_x, max_x, dist, ave_z, ave2_z, m_y, m_z, x, y, z;
		int min_j = 0, max_j = 0, cnt = 0;

		sensor_msgs::PointCloud pc2;					 // 面積や距離を計算した点群データを保存.今回は点群データの削減のために使用 今後距離や面積からクラスタリング結果の点群データを削減するときに使うと良い
		sensor_msgs::PointCloud2 pc2_2;					 // sensor_msgs::PointCloudからsensor_msgs::PointCloud2に変換するときに使用する変数
		geometry_msgs::Point32 pointx[pc.points.size()]; // クラスタリングの点群データの座標を見るための変数

		/*pc2のヘッダ宣言*/
		pc2.channels.resize(2);
		pc2.header = pc.header;
		pc2.points.resize(2);
		pc2.channels[0].values.resize(2);

		/*クラスタリングの面積と距離の計算*/
		for (int j = 0; j < pc.points.size(); j++)
		{
			//初期値設定
			if (j == 0)
			{
				min_y = pc.points[j].y;
				min_x = pc.points[j].x;
				max_y = pc.points[j].y;
				max_x = pc.points[j].x;
				ave_z = pc.points[j].z;
				ave2_z = pc.points[j].z;
				pointx[cnt].x = pc.points[j].x;
				pointx[cnt].y = pc.points[j].y;
				pointx[cnt].z = pc.points[j].z;
				cnt++;
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
				min_j = j;
				/*一応最大も取得*/
			}
			else if (max_y < pc.points[j].y)
			{
				max_y = pc.points[j].y;
				max_j = j;
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

			/*点間距離を計算*/
			// dist = sqrt(pow(pc.points[j].y - pointx[cnt - 1].y, 2) + pow(pc.points[j].x - pointx[cnt - 1].x, 2));
			/*点データを0.3 m以下になるように削減　これがないとトラッキング後のクラスタリングで処理時間が大幅に増える*/
			// if (dist > 0.3)
			// {
			pointx[cnt].x = pc.points[j - 1].x;
			pointx[cnt].y = pc.points[j - 1].y;
			pointx[cnt].z = pc.points[j - 1].z;
			cnt++;
			// }
		}

		/*なくても良い*/
		dist = sqrt(pow(max_y - min_y, 2) + pow(max_x - min_x, 2));

		/*側溝と車両との距離を計算(トラッキングで計算しているためなくても良い)*/
		value_distance_gutter_.data[i + 1] = sqrt(pow(m_y - tf_y, 2) + pow(min_x - tf_x, 2) + pow(m_z - tf_z, 2));
		printf("dist: %f distance: %f x_dist: %f y_dist: %f max_z: %f min_z: %f\n", dist, value_distance_gutter_.data[i + 1], max_x - min_x, max_y - min_y, ave_z, ave2_z);

		/*削減した点群データをsensor_msgs::PointCloud形式で保存*/
		pc2.points.resize(cnt);
		pc2.channels[0].values.resize(cnt);
		for (int i = 0; i < cnt; i++)
		{
			pc2.points[i].x = pointx[i].x;
			pc2.points[i].y = pointx[i].y;
			pc2.points[i].z = pointx[i].z;
		}

		sensor_msgs::convertPointCloudToPointCloud2(pc2, pc2_2); // sensor_msgs::PointCloud -> sensor_msgs::PointCloud2
		pcl::fromROSMsg(pc2_2, *tmp_clustered_points);			 // sensor_msgs::PointCloud2 -> pcl::PointCloud<pcl::PointXYZ>
		*cloud_ptr = *cloud_ptr + *tmp_clustered_points;		 // クラスタリングの結果をひとつの変数に格納
	}

	clouds = *cloud_ptr;						  // pcl::PointCloud<pcl::PointXYZ>::Ptr -> pcl::PointCloud<pcl::PointXYZ>
	pcl::toROSMsg(clouds, cluster_gutter_);		  // pcl::PointCloud<pcl::PointXYZ> -> sensor_msgs::PointCloud2
	cluster_gutter_.header.frame_id = "velodyne"; // TFの座標系を決定　AutowareのLiDARデータはすべてvelodyneで統一されている

	// クラスタリングに要した時間計測
	ros::Time time_end_clustering = ros::Time::now();
	ros::Duration duration_clustering = time_end_clustering - time_start_clustering;
	value_cluster_time_gutter_.data = duration_clustering;
	std::cout << "clustering_time[sec](street gutter)=" << ros::Time::now().toSec() - time_start_clustering.toSec() << std::endl;
}

/**
 * コールバック関数
 * @param msg SubscribeしたPointCloud2型の点群 */
void ClusterGutter::callbackClusterGutter(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// 路面上部の障害物のクラスタリング設定
	pcl::fromROSMsg(*msg, *cloud); // sensor_msgs::PointCloud2 -> pcl::PointCloud<pcl::PointXYZ>

	ROS_INFO("===== street gutter =====");
	std::cout << "cloud->points.size() = " << cloud->points.size() << std::endl;

	check_count = cloud->points.size();

	pnh_.param("cluster_tolerance_", cluster_tolerance_, 1.2); // クラスタとなる距離を設定[m]

	// クラスタリング
	clusters.clear();
	Clustering();

	// Publish
	clustered_gutter_pub_.publish(cluster_gutter_);				  // クラスタリングした点群
	distance_gutter_pub_.publish(value_distance_gutter_);		  // 識別距離
	cluster_time_gutter_pub_.publish(value_cluster_time_gutter_); // クラスタリング時間
}

/**
 * コンストラクタ */
ClusterGutter::ClusterGutter() : pnh_("~")
{
	// gutter_L_sub_ = nh_.subscribe("points_gutter_inner_edge_L", 1, &ClusterGutter::callbackClusterGutter, this);
	// gutter_R_sub_ = nh_.subscribe("points_gutter_inner_edge_R", 1, &ClusterGutter::callbackClusterGutter, this);
	gutter_LR_sub_ = nh_.subscribe("points_gutter_inner_edge_LR", 1, &ClusterGutter::callbackClusterGutter, this);

	clustered_gutter_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cluster_gutter", 10);
	distance_gutter_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("distance_cluster_gutter", 1000);
	cluster_time_gutter_pub_ = nh_.advertise<std_msgs::Duration>("cluster_time_gutter", 1000);

	pnh_.param("min_cluster_size_", min_cluster_size_, 4); //クラスタとなる点データ数の最小数

	std::cout << "cluster_tolerance_ = " << cluster_tolerance_ << std::endl;
	std::cout << "min_cluster_size_ = " << min_cluster_size_ << std::endl;

	/* map座標からbase_link座標を取得、base_link座標からvelodyne座標を取得 */
	tf::TransformListener listener;
	while (true)
	{
		try
		{
			ROS_INFO("Transforming...");
			listener.lookupTransform("base_link", "velodyne", ros::Time(0), transform_); // 座標系間の位置を取得: base_link座標系内のvelodyne座標系の位置情報を取得
			ROS_INFO("Transformed!");
			break;
		}
		catch (tf::TransformException ex)
		{
			ROS_INFO("TransformException");
			ros::Duration(1.0).sleep();
		}
	}

	/* Spin */
	ros::Rate loop_rate(50);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

/**
 * main */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "cluster_gutter");
	ClusterGutter app;
	// ros::spin();
}
