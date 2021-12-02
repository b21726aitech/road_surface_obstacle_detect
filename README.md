# road_surface_obstacle_detect
## 概要

未整備な路面環境での運転支援を行うため, LiDARを用いてリアルタイムで路面環境識別を行う．  
路面障害物の識別は, ポットホール、ハンプ、側溝を対象とする．  
このシステムでは未整備道路環境で, 低速での遠隔運転による運転支援を提供することを目的としている．  

- change_ground_filter：ポットホール、ハンプの識別
- identify_gutter：側溝の識別

## 実行例

LGSVL Simulatorでの実行例  
仮想環境はUnityを用いて作成

![Example Gif](/images/virtual-10.gif)

## 実行環境

| Hardware | Description |
| :---: | :---: |
| CPU | Intel®Core i9-9900K CPU @ 3.6GHz x 16 |
| Memory | 32GB |
| GPU | NVIDIA GeForce RTX 2080 |
| LiDAR | VLP-16 |

| Software | Description |
| :---: | :---: |
| OS | Ubuntu 18.04 (LTS) Bionic beaver |
| CUDA | 10.0 |
| ROS | Melodic Morenia |
| Autoware | Autoware.AI 1.14.0 |
| PCL | 1.7.2-14ubuntu0.1 |
| LGSVL Simulator | 2019.04 |
| Unity | 2018 2.4 |



## ビルド方法

任意のワークスペース内のsrc内にgit cloneし、ワークスペースにて以下を実行
```
catkin_make
```

## 使用方法

1. LiDARからの点群"/points_raw"トピックをPublish
2. Autowareで"voxel_grid_filter"を実行
3. Autowareで"ndt_mapping"を実行
4. Autowareで"ndt_matching"を実行
5. Autowareで"ray_ground_filter"を実行し、"/points_ground"トピックをPublish
6. 以下を各端末で実行

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
