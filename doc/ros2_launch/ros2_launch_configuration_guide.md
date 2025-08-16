# ROS2 Launch Configuration ガイド

## 概要

ROS2のlaunch configurationは、launchファイル内で設定値を動的に管理するためのシステムです。これにより、launchファイルを再利用可能にし、実行時に柔軟な設定変更が可能になります。

## 基本概念

Launch configurationは、launchファイルの実行時に値を設定・変更できる仕組みで、主に以下の目的で使用されます：

- **パラメータ化**: launchファイルを再利用可能にする
- **条件分岐**: 設定値に基づいて異なる動作を実現
- **外部からの制御**: `ros2 launch`コマンド実行時のコマンドライン引数やデフォルト値で動作を変更

## 主要なコンポーネント

### LaunchConfiguration
- launchファイル内で使用する変数のような存在
- 実行時に値が決定される
- `LaunchConfiguration('variable_name')`で取得

### DeclareLaunchArgument
- launch引数を宣言し、デフォルト値や説明を定義
- コマンドラインから値を受け取る入り口

### LaunchDescriptionへの登録 (ld.add_action)
- 宣言した引数をLaunchDescriptionに登録するための重要なステップ
- `ld.add_action(declare_arg_cmd)`の形式で使用
- 登録しないと引数はlaunchファイルで認識されない

引数の使用フロー：
1. **引数の宣言**: `DeclareLaunchArgument`で引数を定義
2. **LaunchDescriptionへの登録**: `ld.add_action()`で宣言した引数をランチファイルに追加
3. **引数の使用**: `LaunchConfiguration`で引数の値を取得して使用

登録により可能になること：
- **コマンドライン引数として使用可能**になる
  ```bash
  ros2 launch your_package your_launch.py arg_name:=custom_value
  ```
- **ランチファイルのヘルプ表示**で引数の説明が表示される
  ```bash
  ros2 launch your_package your_launch.py --show-args
  ```
- **デフォルト値の適用**：引数が指定されない場合、宣言時に設定したデフォルト値が使用される

コード例：
```python
def generate_launch_description():
    # LaunchDescriptionオブジェクトを作成
    ld = LaunchDescription()
    
    # 1. 引数の宣言
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='default_world.sdf',
        description='Full path to world model file to load',
    )
    
    # 2. LaunchDescriptionに登録（重要）
    ld.add_action(declare_world_cmd)
    
    # 3. 引数の使用
    world = LaunchConfiguration('world')
    
    # 登録した引数を使用したアクションを追加
    ld.add_action(
        ExecuteProcess(
            cmd=['simulator', world]
        )
    )
    
    return ld
```

## 使用例

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 引数を宣言
        DeclareLaunchArgument(
            'robot_name',
            default_value='my_robot',
            description='Name of the robot'
        ),
        
        # 設定値を使用
        Node(
            package='my_package',
            executable='my_node',
            name=LaunchConfiguration('robot_name'),
            parameters=[{
                'robot_id': LaunchConfiguration('robot_name')
            }]
        )
    ])
```

## 実行時の動作

### 引数を指定しない場合
```bash
ros2 launch my_package my_launch.py
```

この場合の動作：
- `robot_name`は`'my_robot'`（デフォルト値）になる
- ノード名は`'my_robot'`
- パラメータ`robot_id`も`'my_robot'`

### 引数を指定した場合
```bash
ros2 launch my_package my_launch.py robot_name:=robot1
```

この場合の動作：
- `robot_name`は`'robot1'`になる
- ノード名は`'robot1'`
- パラメータ`robot_id`も`'robot1'`

## 確認方法

実際にどの値が使用されているかは、以下の方法で確認できます：

```bash
# 実行中のノード一覧を確認
ros2 node list

# パラメータ値を確認
ros2 param get /robot_name robot_id
```

## メリット

1. **再利用性**: 同じlaunchファイルを異なる設定で実行可能
2. **柔軟性**: 開発時と本番環境で異なる設定を適用
3. **保守性**: 設定値を一箇所で管理
4. **エラー防止**: デフォルト値により引数なしでも実行可能

## 注意点

- デフォルト値を設定しておくことで、launchファイルを引数なしで実行してもエラーにならない
- 複数の`LaunchConfiguration`で同じ引数名を参照する場合、一つの`DeclareLaunchArgument`で宣言すれば十分
- 引数名は他のlaunchファイルから参照される可能性があるため、意味のある名前を付ける

Launch configurationにより、同じlaunchファイルを異なる設定で柔軟に実行できるようになり、ROS2システムの開発効率と保守性が大幅に向上します。
