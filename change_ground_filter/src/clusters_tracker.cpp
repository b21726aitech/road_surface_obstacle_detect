/*参考資料
(https://lilaboc.work/archives/20136695.html)
*/

//使用するヘッダの宣言
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//sensor_msgs::convertPointCloud2ToPointCloudを使用するためのヘッダ
#include <sensor_msgs/point_cloud_conversion.h>
//pcl::toROSMsg, pcl::fromROSMsgを使用するためのヘッダ
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <std_msgs/Time.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>


class EuclideanClustering{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscribe*/
		ros::Subscriber sub_pc, sub_pc2, sub_pc3;
		/*pcl objects*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
		sensor_msgs::PointCloud2 o_pc2, u_pc2;
		sensor_msgs::PointCloud pc;
		pcl::PointCloud<pcl::PointXYZ> clouds;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr{new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr2{new pcl::PointCloud<pcl::PointXYZ>};
		int check, check_count, check_time;
		std_msgs::Duration o_cluster_Time, u_cluster_Time, all_time;
		std_msgs::Float32MultiArray o_distance, u_distance;
		std_msgs::Time pc_time;
		tf::StampedTransform transform, transform2; //read base link to velodyne and map to base link
		double time_start;
		ros::Time ros_begins;

		/*parameters*/
		double cluster_tolerance;
		int min_cluster_size;
	public:
		EuclideanClustering();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
		void CallbackPCu(const sensor_msgs::PointCloud2ConstPtr &msg);
		void Clustering(void);
		void time_check(const std_msgs::Time msg);
};

EuclideanClustering::EuclideanClustering()
	:nhPrivate("~")
{
	//更新周期設定[Hz]
	ros::Rate loop_rate(50);

	/*subscribeノード作成　
	  NodeHandle.subscribe(ノード名, 保存するキュー数, 実行する関数) thisはなくても良い*/
	sub_pc = nh.subscribe("tracker_over", 1, &EuclideanClustering::CallbackPC, this);
	sub_pc2 = nh.subscribe("tracker_under", 1, &EuclideanClustering::CallbackPCu, this);
	sub_pc3 = nh.subscribe("pc_time", 1, &EuclideanClustering::time_check, this);

	/*Publishノード作成
	  ros::Publisher 変数 = NodeHandle.advertise<メッセージ形式>(ノード名, 発信するキュー数)*/
	ros::Publisher pubp = nh.advertise<sensor_msgs::PointCloud2>("clusters_over_t", 1000);
	ros::Publisher pubp2 = nh.advertise<sensor_msgs::PointCloud2>("clusters_under_t", 1000);
	ros::Publisher clusters_Time_over = nh.advertise<std_msgs::Duration>("cluster_time_over_t", 1000);
	ros::Publisher clusters_Time_under = nh.advertise<std_msgs::Duration>("cluster_time_under_t", 1000);
	ros::Publisher distance_over = nh.advertise<std_msgs::Float32MultiArray>("distance_over_t", 1000);
	ros::Publisher distance_under = nh.advertise<std_msgs::Float32MultiArray>("distance_under_t", 1000);
	ros::Publisher all_time_pub = nh.advertise<std_msgs::Duration>("all_time_t", 1000);

	//クラスタリングとなる点データ数の最小数を設定
	nhPrivate.param("min_cluster_size", min_cluster_size, 2);

	std::cout << "cluster_tolerance = " << cluster_tolerance << std::endl;
	std::cout << "min_cluster_size = " << min_cluster_size << std::endl;
	tf::TransformListener listener, listener2;

	//ros::AsyncSpinner spinner(2);
	//spinner.start();


	while(ros::ok())
	{
		//get coordinate from map to baselink and get Coordinate base link to velodyne
		while(true){
			try{
				//座標系間の位置を取得　ここではvelodyne座標系外にあるmap座標系の位置情報を取得する
				listener.lookupTransform("velodyne", "map", ros::Time(0), transform);
				//座標系間の位置を取得　ここではbase_link座標系内にあるvelodyne座標系の位置情報を取得する
				listener2.lookupTransform("base_link", "velodyne", ros::Time(0), transform2);
				break;
			}
			catch(tf::TransformException ex){
				ros::Duration(1.0).sleep();
			}
		}

		//send message
		//[ros::Publisherの変数].publish(変数);
		pubp.publish(o_pc2);
		clusters_Time_over.publish(o_cluster_Time);
		distance_over.publish(o_distance);

		pubp2.publish(u_pc2);
		clusters_Time_under.publish(u_cluster_Time);
		distance_under.publish(u_distance);
		if(check_time == 1)
			all_time_pub.publish(all_time);

		ros::spinOnce();
		loop_rate.sleep();
	}
}

//路面上部の障害物のクラスタリング設定
void EuclideanClustering::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	/* std::cout << "CALLBACK PC" << std::endl; */

	//路面上部の障害物のクラスタリング設定
	/*sensor_msgs::PointCloud2からpcl::PointCloud<pcl::PointXYZ>に変換
	pcl::fromROSMsg(sensor_msgs::PointCloud2,pcl::PointCloud<pcl::PointXYZ>)*/
	pcl::fromROSMsg(*msg, *cloud);
	std::cout << "=====over=====" << std::endl;
	std::cout << "cloud->points.size() = " << cloud->points.size() << std::endl;

	check = 0;
	check_count = cloud->points.size();
	//クラスタリングとなる距離を設定[m]
	nhPrivate.param("cluster_tolerance", cluster_tolerance, 0.3);

	clusters.clear();
	Clustering();
}

//路面下部の障害物のクラスタリング設定
void EuclideanClustering::CallbackPCu(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	/* std::cout << "CALLBACK PC" << std::endl; */
	/*sensor_msgs::PointCloud2からpcl::PointCloud<pcl::PointXYZ>に変換
	pcl::fromROSMsg(sensor_msgs::PointCloud2,pcl::PointCloud<pcl::PointXYZ>)*/
	pcl::fromROSMsg(*msg, *cloud);
	std::cout << "=====under=====" << std::endl;
	std::cout << "cloud->points.size() = " << cloud->points.size() << std::endl;

	check=1;
	check_count = cloud->points.size();
	//クラスタリングとなる距離を設定[m]
	nhPrivate.param("cluster_tolerance", cluster_tolerance, 0.3);

	clusters.clear();
	Clustering();
}

//動作はしているが使用していない
void EuclideanClustering::time_check(const std_msgs::Time msg){
	if(pc_time.data != msg.data){
		pc_time.data = msg.data;
		ros::Time ros_begin = ros::Time::now();
		all_time.data = ros_begin - pc_time.data;
		check_time = 1;
	}else{
		check_time = 0;
	}
}

//クラスタリング実行関数
void EuclideanClustering::Clustering(void)
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
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	/*クラスタリング後の点群データが格納されるベクトル*/
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
	if(cloud->points.size() != 0){
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
	if(check == 0){
		o_distance.data.resize(cluster_indices.size() + 1);
		o_distance.data[0]=cluster_indices.size();
	}else{
		u_distance.data.resize(cluster_indices.size() + 1);
		u_distance.data[0]=cluster_indices.size();
	}
	*cloud_ptr = *cloud_ptr2;

	ei.setInputCloud(cloud);
	ei.setNegative(false);
	for(size_t i=0;i<cluster_indices.size();i++){
		/*extract*/
		//分割した点群データを保存する変数を設定
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_clustered_points (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointIndices::Ptr tmp_clustered_indices (new pcl::PointIndices);
		*tmp_clustered_indices = cluster_indices[i];
		ei.setIndices(tmp_clustered_indices);
		ei.filter(*tmp_clustered_points);
		clusters.push_back(tmp_clustered_points);
		/*input*/
		pcl::PointCloud<pcl::PointXYZ> cloud_check;
		cloud_check = *tmp_clustered_points;
		if(check==0){
			//pcl::PointCloud<pcl::PointXYZ>からsensor_msgs::PointCloud2に変換
			//pcl::toROSMsg(pcl::PointCloud<pcl::PointXYZ>,sensor_msgs::PointCloud2)
			pcl::toROSMsg(cloud_check, o_pc2);
			//sensor_msgs::PointCloud2からsensor_msgs::PointCloudに変換
			//sensor_msgs::convertPointCloud2ToPointCloud(sensor_msgs::PointCloud2,sensor_msgs::PointCloud)
			sensor_msgs::convertPointCloud2ToPointCloud(o_pc2, pc);
		}else{
			//pcl::PointCloud<pcl::PointXYZ>からsensor_msgs::PointCloud2に変換
			//pcl::toROSMsg(pcl::PointCloud<pcl::PointXYZ>,sensor_msgs::PointCloud2)
			pcl::toROSMsg(cloud_check, u_pc2);
			//sensor_msgs::PointCloud2からsensor_msgs::PointCloudに変換
			//sensor_msgs::convertPointCloud2ToPointCloud(sensor_msgs::PointCloud2,sensor_msgs::PointCloud)
			sensor_msgs::convertPointCloud2ToPointCloud(u_pc2, pc);
		}

		//障害物の面積と距離を取得するための変数宣言
		float min_y = 0, max_y = 0, min_x = 0, max_x = 0, dist = 0, ave_z = 0, ave2_z = 0, m_y = 0, m_z = 0, x = 0, y = 0, z = 0, o_d = 30.0;
		/*クラスタリングの面積と距離の計算*/
		for(int j = 0; j < pc.points.size(); j++){
			if(j == 0){
				//初期値設定
				min_y = pc.points[j].y;
				min_x = pc.points[j].x;
				max_y = pc.points[j].y;
				max_x = pc.points[j].x;
				ave_z = pc.points[j].z;
				ave2_z = pc.points[j].z;
			}
			/*車両距離方向の最小値の点データを取得*/
			if(min_x > pc.points[j].x){
				min_x = pc.points[j].x;
				m_y = pc.points[j].y;
				m_z = pc.points[j].z;
			/*一応最大も取得*/
			}else if(max_x < pc.points[j].x){
				max_x = pc.points[j].x;
			}
			/*車両左右方向の最小値の点データを取得*/
			if(min_y > pc.points[j].y){
				min_y = pc.points[j].y;
			/*一応最大も取得*/
			}else if(max_y < pc.points[j].y){
				max_y = pc.points[j].y;
			}
			/*高さ方向の最小値の点データを取得*/
			if(ave_z > pc.points[j].z){
				ave_z = pc.points[j].z;
			/*一応最大も取得*/
			}else if(ave2_z < pc.points[j].z){
				ave2_z = pc.points[j].z;
			}
		}
		/*なくても良い*/
		dist = sqrt(pow(max_y - min_y,2) + pow(max_x - min_x,2));

		if(check == 0){
			/*velodyne座標系からmap座標系へ座標変換　トラッキングの結果はmap座標系で保存*/
			x = tf_x + (min_x * cos(yaw) + m_y * sin(yaw));
      y = tf_y + (- min_x * sin(yaw) + m_y * cos(yaw));
			z = tf_z + m_z;
			ave_z = tf_z - ave_z;
			ave2_z = tf_z - ave2_z;
			/*路面上部の障害物と車両との距離を計算*/
			o_distance.data[i + 1] = sqrt(pow(y - tf2_y,2) + pow(x - tf2_x,2));
			printf("dist: %f distance: %f x_dist: %f y_dist: %f z_dist: %f\n", dist, o_distance.data[i + 1], std::abs(max_x - min_x), std::abs(max_y - min_y), std::abs(ave_z - ave2_z));
		}else{
			/*velodyne座標系からmap座標系へ座標変換　トラッキングの結果はmap座標系で保存*/
			x = tf_x + (min_x * cos(yaw) + m_y * sin(yaw));
      y = tf_y + (- min_x * sin(yaw) + m_y * cos(yaw));
			z = tf_z + m_z;
			ave_z = tf_z - ave_z;
			ave2_z = tf_z - ave2_z;
			/*路面下部の障害物と車両との距離を計算*/
			u_distance.data[i + 1] = sqrt(pow(y - tf2_y,2) + pow(x - tf2_x,2));
			printf("dist: %f distance: %f x_dist: %f y_dist: %f z_dist: %f\n", dist, u_distance.data[i + 1], std::abs(max_x - min_x), std::abs(max_y - min_y), std::abs(ave_z - ave2_z));
		}
		/*クラスタリングの結果をひとつの変数に格納 今後, ある条件でまとめたくない時にif分内に分岐する変数を入れると良い*/
		if(check == 0){
			*cloud_ptr = *cloud_ptr + *tmp_clustered_points;
		}else if(check == 1){
			*cloud_ptr = *cloud_ptr + *tmp_clustered_points;
		}
	}

	//pcl::PointCloud<pcl::PointXYZ>::Ptrからpcl::PointCloud<pcl::PointXYZ>に変換
	clouds = *cloud_ptr;
	if(check == 0){
		/*pcl::PointCloud<pcl::PointXYZ>からsensor_msgs::PointCloud2に変換
			LiDARのデータの送信はAutowareに合わせてsensor_msgs::PointCloud2のため行なう
			ここでは路面上部の障害物の変数に格納
			pcl::toROSMsg(pcl::PointCloud<pcl::PointXYZ>,sensor_msgs::PointCloud2)*/
		pcl::toROSMsg(clouds, o_pc2);
		/*TFの座標系を決定　トラッキング結果はmap座標系に置くことで車両が移動しても変化しないためmap座標系で保存*/
		o_pc2.header.frame_id="map";
	}else{
		/*pcl::PointCloud<pcl::PointXYZ>からsensor_msgs::PointCloud2に変換
			LiDARのデータの送信はAutowareに合わせてsensor_msgs::PointCloud2のため行なう
			ここでは路面下部の障害物の変数に格納
			pcl::toROSMsg(pcl::PointCloud<pcl::PointXYZ>,sensor_msgs::PointCloud2)*/
		pcl::toROSMsg(clouds, u_pc2);
		/*TFの座標系を決定　トラッキング結果はmap座標系に置くことで車両が移動しても変化しないためmap座標系で保存*/
		u_pc2.header.frame_id="map";
	}

	//処理時間の計測
	ros::Time calc_time = ros::Time::now();
	ros::Duration D_Times = calc_time - ros_begins;
	if(check == 0){
		o_cluster_Time.data = D_Times;
	}else{
		u_cluster_Time.data = D_Times;
	}
	std::cout << "clustering time [s] = " << ros::Time::now().toSec() - time_start << std::endl;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "euclidean_clustering_tracker");

	EuclideanClustering euclidean_clustering;

	//ros::spin();
}
