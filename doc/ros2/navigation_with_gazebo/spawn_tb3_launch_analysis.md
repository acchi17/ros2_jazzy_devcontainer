# spawn_tb3.launch.py の機能解析

## 概要
`spawn_tb3.launch.py`は、Gazeboシミュレーション環境において、下記を行うためのlaunchファイルです。
- TurtleBot3 Waffleロボットをスポーン（生成）する
- ROS2とGazebo間の通信を確立する
このファイルは、Nav2（Navigation2）パッケージを使用したロボットナビゲーションのシミュレーション環境を構築する際の基盤となります。

## Launch引数

このlaunchファイルでは、以下の引数を設定できます：

| 引数名 | デフォルト値 | 説明 |
|--------|--------------|------|
| `namespace` | `''`（空文字） | トップレベルの名前空間 |
| `robot_name` | `'turtlebot3_waffle'` | ロボットの名前 |
| `robot_sdf` | `os.path.join(bringup_dir, 'urdf', 'gz_waffle.sdf.xacro')` | ロボットのSDFファイルパス |
| `x_pose` | `-2.00` | X座標の初期位置 |
| `y_pose` | `-0.50` | Y座標の初期位置 |
| `z_pose` | `0.01` | Z座標の初期位置 |
| `roll` | `0.00` | ロール角（X軸周りの回転） |
| `pitch` | `0.00` | ピッチ角（Y軸周りの回転） |
| `yaw` | `0.00` | ヨー角（Z軸周りの回転） |

## 起動されるROS2パッケージ

### 1. ros_gz_bridge

```python
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    namespace=namespace,
    parameters=[
        {
            'config_file': os.path.join(
                bringup_dir, 'configs', 'turtlebot3_waffle_bridge.yaml'
            ),
            'expand_gz_topic_names': True,
            'use_sim_time': True,
        }
    ],
    output='screen',
)
```

**役割**:
- ROS 2とGazeboの**データ通信**の橋渡し
- ROSトピック ↔ Gazeboトピック間のメッセージ変換
- センサーデータ（LiDAR、カメラ、IMUなど）の転送
- 制御コマンド（cmd_vel、joint_statesなど）の転送
- 継続的なデータストリーミングの維持

**パラメータ**:
- `config_file`: ブリッジの設定ファイル（トピックマッピングを定義）
- `expand_gz_topic_names`: Gazeboトピック名を展開するかどうか
- `use_sim_time`: シミュレーション時間を使用するかどうか

### 2. ros_gz_sim

```python
spawn_model = Node(
    package='ros_gz_sim',
    executable='create',
    output='screen',
    namespace=namespace,
    arguments=[
        '-name', robot_name,
        '-string', Command([
            FindExecutable(name='xacro'), ' ', 'namespace:=',
            LaunchConfiguration('namespace'), ' ', robot_sdf]),
        '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
        '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']]
)
```

**役割**:
- Gazeboシミュレーション環境の**管理・制御**
- モデルのスポーン（生成）
- 指定された位置・姿勢でロボットを配置

**処理**:
- `xacro`コマンドを使用してSDFファイルを処理
- 処理されたSDFデータをGazeboに送信
- 指定された位置・姿勢でロボットモデルを生成

## 環境変数設定

```python
set_env_vars_resources = AppendEnvironmentVariable(
    'GZ_SIM_RESOURCE_PATH', os.path.join(bringup_dir, 'models'))
set_env_vars_resources2 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(os.path.join(bringup_dir)).parent.resolve()))
```

**目的**:
- Gazeboがカスタムモデルを見つけられるようにリソースパスを設定
- `GZ_SIM_RESOURCE_PATH`環境変数にモデルディレクトリを追加
- 親ディレクトリも追加して関連リソースにアクセス可能にする

## 実行フロー

1. **Launch引数の宣言**
   ```python
   ld.add_action(declare_namespace_cmd)
   ld.add_action(declare_robot_name_cmd)
   ld.add_action(declare_robot_sdf_cmd)
   ```

2. **環境変数の設定**
   ```python
   ld.add_action(set_env_vars_resources)
   ld.add_action(set_env_vars_resources2)
   ```

3. **ROS-Gazeboブリッジの起動**
   ```python
   ld.add_action(bridge)
   ```

4. **ロボットモデルのスポーン**
   ```python
   ld.add_action(spawn_model)
   ```

## アーキテクチャ図

```
ROS 2 Application
    ↓ (制御コマンド)
ros_gz_bridge ←→ Gazebo Simulation
    ↑ (センサーデータ)
ROS 2 Application

ROS 2 Launch
    ↓ (管理コマンド)
ros_gz_sim → Gazebo Simulation
```

## 使用例

このlaunchファイルは通常、より上位のlaunchファイルから呼び出されます。単独で実行する場合は以下のようになります：

```bash
ros2 launch nav2_minimal_tb3_sim spawn_tb3.launch.py
```

パラメータを指定して実行する例：

```bash
ros2 launch nav2_minimal_tb3_sim spawn_tb3.launch.py robot_name:=my_robot x_pose:=1.0 y_pose:=2.0 yaw:=1.57
```

## まとめ

`spawn_tb3.launch.py`は、以下の主要な機能を提供します：

1. TurtleBot3 Waffleロボットモデルをシミュレーション環境に生成
2. ROS 2とGazebo間のデータ通信を確立
3. ロボットの初期位置・姿勢を設定
4. Gazeboリソースパスを適切に構成

このlaunchファイルは、Nav2を使用したナビゲーションシミュレーションの基盤となる重要なコンポーネントです。
