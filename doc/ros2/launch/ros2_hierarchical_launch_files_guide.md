# ROS2 Hierarchical Launch Files ガイド

## 概要

ROS2の**Hierarchical Launch Files**（階層的launchファイル）は、`IncludeLaunchDescription()`を使用して複数のlaunchファイルを一度に起動する仕組みです。この機能により、複雑なロボットシステムを構成する複数のlaunchファイルを階層的に組み合わせて管理できます。

## 公式な名称

この仕組みには以下の特別な名称が付与されています：

1. **Hierarchical Launch Files**（階層的launchファイル）- 最も一般的な公式名称
2. **Launch Composition**（launch構成）- より単純なシステムの構成による複雑さの管理
3. **Tree Structure**（ツリー構造）- 実行可能ファイルとlaunchファイルのツリー構造

## 階層構造の例
```
full_system.launch.py (親)
├── hardware.launch.py (子)
│   ├── motors.launch.py (孫)
│   └── sensors.launch.py (孫)
├── software.launch.py (子)
│   ├── perception.launch.py (孫)
│   └── planning.launch.py (孫)
└── visualization.launch.py (子)
```

## 実装例

### 子のlaunchファイル（robot_bringup.py）
```python
from launch import LaunchDescription
from launch.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_driver',
            executable='robot_driver_node',
            name='robot_driver'
        ),
        Node(
            package='sensor_driver',
            executable='camera_node',
            name='camera'
        )
    ])
```

### 親のlaunchファイル（main_system.py）
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # 他のlaunchファイルを包含
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('my_package'),
                '/launch/robot_bringup.py'
            ])
        ),
        
        # 追加でこのファイル独自のノードも起動
        Node(
            package='navigation',
            executable='navigation_node',
            name='navigation'
        )
    ])
```

## Launch Configurationとの連携

### 1. 条件付きインクルード
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_navigation',
            default_value='true',
            description='Whether to launch navigation'
        ),
        
        # 条件付きでlaunchファイルを包含
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('nav2_bringup'),
                '/launch/navigation_launch.py'
            ]),
            condition=IfCondition(LaunchConfiguration('use_navigation'))
        )
    ])
```

### 2. 引数の受け渡し
```python
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='my_robot'
        ),
        
        # 親のconfigurationを子のlaunchファイルに渡す
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('my_package'),
                '/launch/robot_bringup.py'
            ]),
            launch_arguments={
                'robot_name': LaunchConfiguration('robot_name'),
                'use_sim_time': 'true'
            }
        )
    ])
```

### 3. 設定の一元管理
```python
def generate_launch_description():
    # 共通設定
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='my_robot'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        # 全ての子launchファイルに共通設定を渡す
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('hardware_package'),
                '/launch/hardware.launch.py'
            ]),
            launch_arguments={
                'robot_name': robot_name,
                'use_sim_time': use_sim_time
            }
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('software_package'),
                '/launch/software.launch.py'
            ]),
            launch_arguments={
                'robot_name': robot_name,
                'use_sim_time': use_sim_time
            }
        )
    ])
```

## 実行時の動作

### 個別に起動する場合（従来の方法）
```bash
# 3つのターミナルで個別に起動
ros2 launch robot_bringup robot.launch.py
ros2 launch sensor_package sensors.launch.py
ros2 launch nav2_bringup navigation.launch.py
```

### まとめて起動する場合（Hierarchical Launch Files使用）
```bash
# 1つのコマンドで全て起動
ros2 launch my_system full_system.launch.py

# 引数付きで起動
ros2 launch my_system full_system.launch.py robot_name:=robot1 use_navigation:=false
```

## 実用的な例

### 自動運転ロボットシステム
```python
def generate_launch_description():
    return LaunchDescription([
        # システム全体の設定
        DeclareLaunchArgument('robot_name', default_value='autonomous_robot'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        # ハードウェア制御
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('hardware_bringup'),
                '/launch/hardware.launch.py'
            ]),
            launch_arguments={
                'robot_name': LaunchConfiguration('robot_name'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ),
        
        # センサー類（カメラ・LiDAR）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('sensor_bringup'),
                '/launch/sensors.launch.py'
            ]),
            launch_arguments={
                'robot_name': LaunchConfiguration('robot_name')
            }
        ),
        
        # 自己位置推定
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('localization_package'),
                '/launch/localization.launch.py'
            ])
        ),
        
        # 経路計画・ナビゲーション
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('nav2_bringup'),
                '/launch/navigation.launch.py'
            ]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ),
        
        # 可視化ツール
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('visualization_package'),
                '/launch/rviz.launch.py'
            ])
        )
    ])
```

## メリット

### 1. 運用の簡素化
- 複数のlaunchファイルを一度に起動
- 起動順序の管理が不要
- エラー発生時の対応が容易

### 2. システムの階層化
- 機能ごとにlaunchファイルを分割
- 責任の分離による保守性向上
- 再利用可能なコンポーネント化

### 3. 設定の一元管理
- 共通設定を親ファイルで管理
- 子ファイルへの設定継承
- 実行時の柔軟な設定変更

### 4. 開発効率の向上
- モジュール化による開発の並行化
- テスト用・本番用の構成切り替え
- 段階的なシステム構築

## 注意点

1. **スコープの管理**: 各launchファイルは独自のlaunch configurationスコープを持つ
2. **依存関係**: 子launchファイルの依存関係を適切に管理する必要がある
3. **デバッグの複雑化**: 階層が深くなるとデバッグが困難になる場合がある
4. **設定の重複**: 同じ設定が複数箇所で定義されないよう注意

## まとめ

Hierarchical Launch Filesは、ROS2において複雑なロボットシステムを効率的に管理するための重要な機能です。`IncludeLaunchDescription()`とLaunch Configurationシステムを組み合わせることで、柔軟で保守性の高いシステム構成を実現できます。