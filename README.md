# road_surface_obstacle_detect
路面障害物の識別を実行する
- change_ground_filter：ポットホール、ハンプの識別
- identify_gutter：側溝の識別

## ビルド方法
任意のワークスペース内のsrc内にファイルを配置し、ワークスペースにて以下を実行
```
catkin_make
```

## 使用方法
"/points_ground"トピックをPublishした後以下を各端末で実行

### ポットホール・ハンプ識別
```
source devel/setup.bash & rosrun change_ground_filter identify_front_vehicle
```
```
source devel/setup.bash & rosrun change_ground_filter ground_filter2
```
```
source devel/setup.bash & rosrun change_ground_filter tracker
```
```
source devel/setup.bash & rosrun change_ground_filter clusters
```
```
source devel/setup.bash & rosrun change_ground_filter clusters_tracker
```

### 側溝識別
```
source devel/setup.bash & rosrun identify_gutter identify_gutter
```
```
source devel/setup.bash & rosrun identify_gutter cluster_gutter
```
```
source devel/setup.bash & rosrun identify_gutter track_gutter
```
