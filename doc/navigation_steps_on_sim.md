# ROS2 Jazzyでの自律走行シミュレーション

このドキュメントでは、ROS2 JazzyとGazeboを使用してTurtleBot3の自律走行シミュレーションを実行するために必要なパッケージと手順について説明します。

## 必要なROS2パッケージ

TurtleBot3をベースにNav2とSLAMを使用した自律走行シミュレーションには、以下のパッケージが必要です：

### TurtleBot3関連パッケージ
- `ros-jazzy-turtlebot3` - TurtleBot3の基本パッケージ（URDFモデル、制御インターフェースなど）
- `ros-jazzy-turtlebot3-simulations` - Gazeboでのシミュレーション用パッケージ
- `ros-jazzy-turtlebot3-msgs` - TurtleBot3固有のメッセージ定義

### ナビゲーション関連パッケージ
- `ros-jazzy-navigation2` - ROS2のナビゲーションスタック（Nav2）
- `ros-jazzy-nav2-bringup` - Nav2の主要コンポーネントを起動するためのパッケージ

### SLAM関連パッケージ
- `ros-jazzy-slam-toolbox` - 2Dマッピングのための標準的なSLAMツール

### 可視化・操作ツール
- `ros-jazzy-rviz2` - 可視化ツール
- `ros-jazzy-teleop-twist-keyboard` - キーボードでロボットを操作するためのツール

### その他の便利なパッケージ
- `ros-jazzy-xacro` - XMLマクロ処理ツール
- `ros-jazzy-joint-state-publisher` - ロボットの関節状態を公開するパッケージ
- `ros-jazzy-robot-state-publisher` - ロボットの状態を公開するパッケージ

## Dockerfileへの追加方法

以下のようにDockerfileを修正して、必要なパッケージをインストールします：

```dockerfile
# 既存のROS2パッケージインストール部分に追加
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    # Gazebo関連
    ros-jazzy-ros-gz \
    # TurtleBot3関連
    ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-simulations \
    ros-jazzy-turtlebot3-msgs \
    # Nav2関連
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    # SLAM関連
    ros-jazzy-slam-toolbox \
    # 可視化・操作ツール
    ros-jazzy-rviz2 \
    ros-jazzy-teleop-twist-keyboard \
    # その他の便利なパッケージ
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# TurtleBot3の環境変数設定*
ENV TURTLEBOT3_MODEL=waffle_pi
```

## 自律走行シミュレーションの手順

### 1. 環境設定

コンテナ内で以下の環境変数を設定します(Dockefileで設定済みの場合は不要)：
```bash
export TURTLEBOT3_MODEL=waffle_pi  # または burger, waffle
```
この環境変数を設定することで、シミュレーション環境で使用するTurtleBot3のモデルを指定します。
これにより、適切なURDFモデル、物理特性、センサー構成などが読み込まれます。
モデルによって、サイズ、重量、センサーの種類と配置、運動能力などが異なるため、シミュレーション結果に影響します。

### 2. Gazeboでのシミュレーション世界の起動

まず、シミュレーション環境を起動します：
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
これにより、TurtleBot3と事前定義された世界（障害物を含む環境）がGazeboで起動します。

他の環境オプションもあります：
- `turtlebot3_house.launch.py` - 家の中の環境
- `turtlebot3_empty_world.launch.py` - 空の環境

### 3. SLAMによるマッピング

新しいターミナルを開き、SLAMを起動してマップを作成します：
```bash
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true \
  params_file:=/opt/ros/jazzy/share/turtlebot3_navigation2/param/waffle_pi/slam_params.yaml
```

### 4. ロボットの遠隔操作でマップ作成

マップを作成するために、ロボットを環境内で動かす必要があります。新しいターミナルで：
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
キーボードの指示に従って、ロボットを環境内で動かし、マップを作成します。

### 5. マップの保存

十分な領域をマッピングしたら、マップを保存します：
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

### 6. ナビゲーションの起動

マップを作成した後、新しいターミナルでナビゲーションを起動します：
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  map:=$HOME/map.yaml
```

### 7. RViz2での初期位置設定

ナビゲーションが起動したら、RViz2インターフェースで：
1. 上部ツールバーの「2D Pose Estimate」ボタンをクリック
2. マップ上でロボットの実際の位置をクリックし、向きをドラッグして設定

### 8. 目標位置の設定と自律走行

初期位置を設定したら、目標位置を設定して自律走行を開始します：
1. 上部ツールバーの「Navigation2 Goal」ボタンをクリック
2. マップ上で目標位置をクリックし、向きをドラッグして設定

ロボットは自動的に経路を計画し、障害物を回避しながら目標位置まで移動します。

## 一連の流れをまとめたlaunchファイル（オプション）

開発が進んだら、以下のようなカスタムlaunchファイルを作成して、一連の流れを自動化することもできます：

```python
# turtlebot3_autonomous_nav.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_dir = LaunchConfiguration('map_dir', default=os.path.expanduser('~'))
    map_yaml = LaunchConfiguration('map', default=map_dir + '/map.yaml')
    
    # Gazeboの起動
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('turtlebot3_gazebo'), 
            '/launch/turtlebot3_world.launch.py'
        ])
    )
    
    # ナビゲーションの起動
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('turtlebot3_navigation2'),
            '/launch/navigation2.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml
        }.items()
    )
    
    return LaunchDescription([
        gazebo_launch,
        navigation_launch
    ])
```

## トラブルシューティング

### 一般的な問題と解決策

1. **Gazeboが起動しない場合**
   - グラフィックドライバの問題の可能性があります。WSLg（Windows Subsystem for Linux GUI）の設定を確認してください。
   - メモリ不足の可能性もあります。他のアプリケーションを閉じてリソースを解放してみてください。

2. **「TF timeout」エラーが発生する場合**
   - `use_sim_time:=true`パラメータが正しく設定されているか確認してください。
   - 複数のターミナルで実行している場合、すべてのターミナルで同じ環境変数が設定されているか確認してください。

3. **ロボットが正しく動かない場合**
   - `TURTLEBOT3_MODEL`環境変数が正しく設定されているか確認してください。
   - teleop_twist_keyboardの入力が正しく処理されているか確認してください。

4. **マップが正しく作成されない場合**
   - ロボットをゆっくりと動かし、センサーが環境を十分にスキャンできるようにしてください。
   - SLAMパラメータを調整する必要がある場合があります。

5. **ナビゲーションが正しく機能しない場合**
   - 初期位置が正しく設定されているか確認してください。
   - コストマップパラメータを調整する必要がある場合があります。

### パフォーマンス向上のためのヒント

1. コンテナに十分なリソース（CPU、メモリ）を割り当ててください。
2. グラフィック処理が重い場合は、Gazeboの物理シミュレーションの精度を下げることを検討してください。
3. 大きな環境でのシミュレーションは、より多くのリソースを必要とします。必要に応じて環境の複雑さを調整してください。
