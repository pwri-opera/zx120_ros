# zx120_ros
OPERA対応油圧ショベルzx120の土木研究所公開ROSパッケージ群

## 概説
- 国立研究開発法人土木研究所が公開するOPERA（Open Platform for Eathwork with Robotics Autonomy）対応の油圧ショベルであるZX120用のROSパッケージ群である
- 本パッケージに含まれる各launchファイルを起動することで、実機やシミュレータを動作させるのに必要なROSノード群が立ち上がる
- 動作確認済のROS Version : ROS Melodic Morenia + Ubuntu 18.04 LTS

## ビルド方法
- ワークスペースの作成（既にwsを作成済の場合は不要．以下、新規作成するワークスペースの名称を"catkin_ws"と仮定して表記）
  ```bash
  $ cd ~/
  $ mkdir --parents catkin_ws/src
  $ cd catkin_ws
  $ catkin init
  $ catkin build
  ```

- パッケージのビルドと自分のワークスペースをインストール環境上にOverlayする
  ```bash
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/pwri-opera/zx120_ros.git
  $ catkin build
  $ source ../devel/setup.bash
  ```

## 含有するサブパッケージ
### zx120_bringup:

### zx120_control:

### zx120_description:

### zx120_gazebo:

### zx120_ikfast_plugin:

### zx120_moveit_config:

### zx120_unity:

## 各ROSノード群の起動方法
- 実機動作に必要なROSノード群の起動方法
  ```bash
  $ roslaunch zx120_bringup zx120_vehicle.launch
  ```
- Unityシミュレータとの連携に必要なROSノード群の起動方法
  ```bash
  $ roslaunch zx120_unity zx120_standby.launch
  ```

## ハードウェアシステム
zx120のハードウェアのシステム構成を以下のブロック図へ示します
![zx120_hardware_system](https://user-images.githubusercontent.com/24404939/159678741-7f0b94bb-2b62-4af4-be3b-30581bb08a46.jpg)

## ソフトウェアシステム
### roslaunch zx120_unity zx120_standy.launch実行時のノード/トピックパイプライン（rqt_graph）
![correct_rosgraph](https://user-images.githubusercontent.com/24404939/160327979-1281697d-6322-4fd6-82ed-0c5332fea4eb.png)

### roslaunch zx120_bringup zx120_vehicle.launch実行時のノード/トピックパイプライン（rqt_graph）
（UNDER CONSTRUCTION）
